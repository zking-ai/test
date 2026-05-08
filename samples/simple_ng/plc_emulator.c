/** \file
 * \brief Orin PLC emulator skeleton.
 *
 * This program is the first safe step for replacing the original PLC/IPC stack:
 *   - scan EtherCAT slaves with SOEM;
 *   - optionally connect to the original IPC TCP server as the old PLC did;
 *   - optionally parse PLC command frames from the IPC;
 *   - optionally send a PLC-like status frame back.
 *
 * Motion commands are guarded by explicit command-line modes and conservative
 * bench limits.
 */

#include "plc_protocol.h"
#include "plc_state.h"
#include "plc_tcp_client.h"
#include "soem/soem.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_PC_HOST "192.168.1.200"
#define DEFAULT_PC_PORT 4000
#define DEFAULT_STATUS_PERIOD_MS 200
#define DEFAULT_IDLE_PERIOD_MS 1000
#define IO_MAP_SIZE 4096
#define MODE_PP 1
#define MODE_PV 3
#define JOG_MAX_ABS_VELOCITY 100000
#define JOG_MAX_DURATION_MS 2000
#define JOG_PROFILE_ACCEL 8000000
#define JOG_PROFILE_DECEL 8000000
#define MOVE_MAX_ABS_DELTA_COUNTS 200000
#define MOVE_MAX_TIMEOUT_MS 5000
#define MOVE_POSITION_TOLERANCE_COUNTS 500
#define MOVE_SLOWDOWN_COUNTS 20000
#define MOVE_MIN_ABS_VELOCITY 3000
#define AXIS_ZERO_FILE "plc_axis_zero.cfg"
#define AXIS_SCALE_FILE "plc_axis_scale.cfg"
#define CALIBRATE_MIN_ABS_USER_COUNTS 1000

static volatile sig_atomic_t g_running = 1;

typedef enum {
   MODE_IDLE = 0,
   MODE_AXES,
   MODE_STATUS,
   MODE_OP_STATUS,
   MODE_FAULT_RESET,
   MODE_ENABLE_DRYRUN,
   MODE_JOG_VELOCITY,
   MODE_MOVE_RELATIVE,
   MODE_MOVE_ABSOLUTE,
   MODE_AXIS_STATUS,
   MODE_SET_ZERO,
   MODE_MOVE_USER_ABSOLUTE,
   MODE_PLC_TABLE,
   MODE_SCALE_STATUS,
   MODE_SET_SCALE,
   MODE_CALIBRATE_SCALE,
   MODE_MOVE_PLC_ABSOLUTE,
   MODE_MOVE_PLC_SLOT,
   MODE_MOVE_PLC_GROUP,
   MODE_TCP
} AppMode;

typedef struct {
   int present, bit_offset, bit_length;
} PdoEntry;

typedef struct {
   PdoEntry controlword, statusword, error_code;
   PdoEntry modes_of_operation, mode_display;
   PdoEntry target_velocity, target_position, actual_position;
   PdoEntry max_profile_velocity;
} ServoMap;

typedef struct {
   uint16 slave;
   ServoMap map;
} EmuAxis;

typedef struct {
   ecx_contextt ctx;
   uint8 iomap[IO_MAP_SIZE];
   int initialized;
   int slave_count;
   int axis_count;
   EmuAxis axis[PLC_AXIS_COUNT];
} EthercatScan;

static EthercatScan *g_pdo_bus;

typedef struct {
   const char *name;
   int axis_no;
} AxisAlias;

typedef struct {
   int valid;
   int32 zero_raw;
} AxisZero;

typedef struct {
   int valid;
   double counts_per_unit;
} AxisScale;

typedef struct {
   int axis_no;
   const char *slot;
   const char *plc_value;
   const char *note;
} PlcPositionEntry;

static const char *g_axis_names[PLC_AXIS_COUNT] = {
   "updown",
   "flex",
   "dituo_x",
   "dituo_y"
};

static const AxisAlias g_axis_aliases[] = {
   { "1", 1 }, { "axis1", 1 }, { "slave1", 1 }, { "updown", 1 }, { "lift", 1 }, { "tisheng", 1 }, { "Axis_updown", 1 },
   { "2", 2 }, { "axis2", 2 }, { "slave2", 2 }, { "flex", 2 }, { "shensuo", 2 }, { "Axis_flex", 2 },
   { "3", 3 }, { "axis3", 3 }, { "slave3", 3 }, { "dituo_x", 3 }, { "dt_x", 3 }, { "Axis_dt_x", 3 },
   { "4", 4 }, { "axis4", 4 }, { "slave4", 4 }, { "dituo_y", 4 }, { "dt_y", 4 }, { "Axis_dt_y", 4 }
};

static const PlcPositionEntry g_plc_positions[] = {
   { 1, "home", "0", "updown_set.home_position" },
   { 1, "pos0", "0", "updown_set.pos_value[0]" },
   { 1, "pos1", "-100", "updown_set.pos_value[1]" },
   { 1, "pos2", "0", "updown_set.pos_value[2]" },
   { 2, "home", "0", "flex_set.home_position" },
   { 2, "pos0", "0", "flex Auto_pos[0]" },
   { 2, "pos1", "-15", "flex_set.pos_value[1]" },
   { 3, "retract", "0", "dituo_x retract/home" },
   { 3, "extend", "35", "dituo_x extend" },
   { 4, "retract", "0", "dituo_y retract/home" },
   { 4, "extend", "70", "dituo_y extend" }
};

static int sdo_read_i32(ecx_contextt *ctx, uint16 slave, uint16 idx, uint8 sub, int32 *value);

static void on_signal(int sig)
{
   (void)sig;
   g_running = 0;
}

static long elapsed_ms(const struct timespec *a, const struct timespec *b)
{
   return (long)((b->tv_sec - a->tv_sec) * 1000 +
                 (b->tv_nsec - a->tv_nsec) / 1000000);
}

static const char *axis_name(int axis_no)
{
   if (axis_no < 1 || axis_no > PLC_AXIS_COUNT) return "unknown";
   return g_axis_names[axis_no - 1];
}

static int parse_axis_name(const char *text, int *axis_no)
{
   size_t i;

   for (i = 0; i < sizeof(g_axis_aliases) / sizeof(g_axis_aliases[0]); i++) {
      if (strcmp(text, g_axis_aliases[i].name) == 0) {
         *axis_no = g_axis_aliases[i].axis_no;
         return 1;
      }
   }

   return 0;
}

static int parse_i32_arg(const char *text, int32 *out)
{
   char *endp;
   long value = strtol(text, &endp, 0);

   if (*endp) return 0;
   *out = (int32)value;
   return 1;
}

static int parse_double_arg(const char *text, double *out)
{
   char *endp;
   double value = strtod(text, &endp);

   if (*endp) return 0;
   if (value != value || value < -1.0e12 || value > 1.0e12) return 0;
   *out = value;
   return 1;
}

static int load_axis_zero(AxisZero zero[PLC_AXIS_COUNT])
{
   FILE *fp;
   char line[128];

   for (int i = 0; i < PLC_AXIS_COUNT; i++) {
      zero[i].valid = 0;
      zero[i].zero_raw = 0;
   }

   fp = fopen(AXIS_ZERO_FILE, "r");
   if (!fp) return 0;

   while (fgets(line, sizeof(line), fp)) {
      char name[64];
      long raw;
      int axis_no;

      if (line[0] == '#') continue;
      if (sscanf(line, "%63s %ld", name, &raw) != 2) continue;
      if (parse_axis_name(name, &axis_no) && axis_no >= 1 && axis_no <= PLC_AXIS_COUNT) {
         zero[axis_no - 1].valid = 1;
         zero[axis_no - 1].zero_raw = (int32)raw;
      }
   }

   fclose(fp);
   return 1;
}

static int save_axis_zero(const AxisZero zero[PLC_AXIS_COUNT])
{
   FILE *fp = fopen(AXIS_ZERO_FILE, "w");

   if (!fp) {
      printf("axis-zero: failed to write %s\n", AXIS_ZERO_FILE);
      return 0;
   }

   fprintf(fp, "# plc_emulator axis zero file\n");
   for (int i = 0; i < PLC_AXIS_COUNT; i++) {
      if (zero[i].valid)
         fprintf(fp, "%s %d\n", axis_name(i + 1), zero[i].zero_raw);
   }

   fclose(fp);
   return 1;
}

static int load_axis_scale(AxisScale scale[PLC_AXIS_COUNT])
{
   FILE *fp;
   char line[128];

   for (int i = 0; i < PLC_AXIS_COUNT; i++) {
      scale[i].valid = 0;
      scale[i].counts_per_unit = 0.0;
   }

   fp = fopen(AXIS_SCALE_FILE, "r");
   if (!fp) return 0;

   while (fgets(line, sizeof(line), fp)) {
      char name[64];
      double value;
      int axis_no;

      if (line[0] == '#') continue;
      if (sscanf(line, "%63s %lf", name, &value) != 2) continue;
      if (parse_axis_name(name, &axis_no) && axis_no >= 1 && axis_no <= PLC_AXIS_COUNT) {
         scale[axis_no - 1].valid = 1;
         scale[axis_no - 1].counts_per_unit = value;
      }
   }

   fclose(fp);
   return 1;
}

static int save_axis_scale(const AxisScale scale[PLC_AXIS_COUNT])
{
   FILE *fp = fopen(AXIS_SCALE_FILE, "w");

   if (!fp) {
      printf("axis-scale: failed to write %s\n", AXIS_SCALE_FILE);
      return 0;
   }

   fprintf(fp, "# plc_emulator PLC engineering unit to encoder counts scale\n");
   fprintf(fp, "# format: axis_name counts_per_plc_unit\n");
   for (int i = 0; i < PLC_AXIS_COUNT; i++) {
      if (scale[i].valid)
         fprintf(fp, "%s %.12g\n", axis_name(i + 1), scale[i].counts_per_unit);
   }

   fclose(fp);
   return 1;
}

static void print_axis_aliases(void)
{
   printf("Axis aliases:\n");
   printf("  1 / axis1 / slave1 / updown / lift / tisheng / Axis_updown\n");
   printf("  2 / axis2 / slave2 / flex / shensuo / Axis_flex\n");
   printf("  3 / axis3 / slave3 / dituo_x / dt_x / Axis_dt_x\n");
   printf("  4 / axis4 / slave4 / dituo_y / dt_y / Axis_dt_y\n");
   printf("Note: logical names follow EtherCAT slave order until confirmed on the real machine.\n");
}

static void print_axis_map(const EthercatScan *bus)
{
   printf("Logical axis map:\n");
   for (int i = 0; i < PLC_AXIS_COUNT; i++) {
      if (i < bus->axis_count) {
         const ec_slavet *s = &bus->ctx.slavelist[bus->axis[i].slave];
         printf("  axis %d %-8s -> slave %u '%s' (connected)\n",
                i + 1, axis_name(i + 1), bus->axis[i].slave, s->name);
      } else {
         printf("  axis %d %-8s -> not connected in this EtherCAT chain\n",
                i + 1, axis_name(i + 1));
      }
   }
   print_axis_aliases();
}

static int read_axis_actual_sdo(EthercatScan *bus, int axis_no, int32 *actual)
{
   if (axis_no < 1 || axis_no > bus->axis_count) return 0;
   return sdo_read_i32(&bus->ctx, bus->axis[axis_no - 1].slave, 0x6064, 0x00, actual);
}

static void print_plc_position_table(void)
{
   printf("Original PLC position table (engineering values, not encoder counts yet):\n");
   for (size_t i = 0; i < sizeof(g_plc_positions) / sizeof(g_plc_positions[0]); i++) {
      const PlcPositionEntry *p = &g_plc_positions[i];
      printf("  axis %d/%-8s %-8s plc_value=%-8s %s\n",
             p->axis_no, axis_name(p->axis_no), p->slot, p->plc_value, p->note);
   }
   printf("Note: calibrate PLC-unit-to-counts scaling before using these as motion targets.\n");
}

static void print_axis_scale_status(int selected_axis)
{
   AxisScale scale[PLC_AXIS_COUNT];

   load_axis_scale(scale);
   printf("scale-status: scale file %s\n", AXIS_SCALE_FILE);
   for (int i = 0; i < PLC_AXIS_COUNT; i++) {
      int axis_no = i + 1;

      if (selected_axis != 0 && selected_axis != axis_no) continue;
      printf("  axis %d/%s\n", axis_no, axis_name(axis_no));
      if (scale[i].valid) {
         printf("    counts_per_plc_unit: %.12g\n", scale[i].counts_per_unit);
      } else {
         printf("    counts_per_plc_unit: not set\n");
      }
      printf("    plc slots:\n");
      for (size_t j = 0; j < sizeof(g_plc_positions) / sizeof(g_plc_positions[0]); j++) {
         const PlcPositionEntry *p = &g_plc_positions[j];
         if (p->axis_no == axis_no) {
            printf("      %-8s plc_value=%-8s %s\n", p->slot, p->plc_value, p->note);
         }
      }
   }
}

static int set_axis_scale(int axis_no, double counts_per_unit)
{
   AxisScale scale[PLC_AXIS_COUNT];

   if (axis_no < 1 || axis_no > PLC_AXIS_COUNT) {
      printf("set-scale: invalid axis %d, configured max axes: 1..%d\n",
             axis_no, PLC_AXIS_COUNT);
      return 0;
   }
   if (counts_per_unit == 0.0 ||
       counts_per_unit < -1.0e9 || counts_per_unit > 1.0e9) {
      printf("set-scale: counts_per_plc_unit must be non-zero and within +/- 1e9\n");
      return 0;
   }

   load_axis_scale(scale);
   scale[axis_no - 1].valid = 1;
   scale[axis_no - 1].counts_per_unit = counts_per_unit;
   if (!save_axis_scale(scale)) return 0;

   printf("set-scale: axis %d/%s counts_per_plc_unit=%.12g saved to %s\n",
          axis_no, axis_name(axis_no), counts_per_unit, AXIS_SCALE_FILE);
   return 1;
}

static int plc_position_lookup(int axis_no, const char *slot, double *plc_value)
{
   for (size_t i = 0; i < sizeof(g_plc_positions) / sizeof(g_plc_positions[0]); i++) {
      const PlcPositionEntry *p = &g_plc_positions[i];
      if (p->axis_no == axis_no && strcmp(p->slot, slot) == 0) {
         return parse_double_arg(p->plc_value, plc_value);
      }
   }
   return 0;
}

static void print_plc_slots_for_axis(int axis_no)
{
   printf("PLC slots for axis %d/%s:\n", axis_no, axis_name(axis_no));
   for (size_t i = 0; i < sizeof(g_plc_positions) / sizeof(g_plc_positions[0]); i++) {
      const PlcPositionEntry *p = &g_plc_positions[i];
      if (p->axis_no == axis_no) {
         printf("  %-8s plc_value=%-8s %s\n", p->slot, p->plc_value, p->note);
      }
   }
}

static void axis_status_user(EthercatScan *bus, int selected_axis)
{
   AxisZero zero[PLC_AXIS_COUNT];
   AxisScale scale[PLC_AXIS_COUNT];

   load_axis_zero(zero);
   load_axis_scale(scale);
   printf("axis-status: zero file %s, scale file %s\n", AXIS_ZERO_FILE, AXIS_SCALE_FILE);
   for (int i = 0; i < bus->axis_count; i++) {
      int axis_no = i + 1;
      int32 actual = 0;

      if (selected_axis != 0 && selected_axis != axis_no) continue;
      printf("  axis %d/%s slave %u\n", axis_no, axis_name(axis_no), bus->axis[i].slave);
      if (!read_axis_actual_sdo(bus, axis_no, &actual)) {
         printf("    raw_position: read failed\n");
         continue;
      }
      printf("    raw_position : %d\n", actual);
      if (zero[i].valid) {
         int32 user_position = actual - zero[i].zero_raw;

         printf("    zero_raw     : %d\n", zero[i].zero_raw);
         printf("    user_position: %d counts\n", user_position);
         if (scale[i].valid) {
            printf("    plc_position : %.6f\n",
                   (double)user_position / scale[i].counts_per_unit);
            printf("    scale        : %.12g counts/plc_unit\n",
                   scale[i].counts_per_unit);
         } else {
            printf("    plc_position : unavailable, run set-scale or calibrate-scale first\n");
         }
      } else {
         printf("    zero_raw     : not set\n");
         printf("    user_position: unavailable, run set-zero first\n");
      }
   }
}

static int set_axis_zero(EthercatScan *bus, int axis_no, int32 user_position)
{
   AxisZero zero[PLC_AXIS_COUNT];
   int32 actual = 0;

   if (axis_no < 1 || axis_no > bus->axis_count) {
      printf("set-zero: invalid axis %d, available axes: 1..%d\n", axis_no, bus->axis_count);
      return 0;
   }
   if (!read_axis_actual_sdo(bus, axis_no, &actual)) {
      printf("set-zero: failed to read actual position for axis %d/%s\n",
             axis_no, axis_name(axis_no));
      return 0;
   }

   load_axis_zero(zero);
   zero[axis_no - 1].valid = 1;
   zero[axis_no - 1].zero_raw = actual - user_position;
   if (!save_axis_zero(zero)) return 0;

   printf("set-zero: axis %d/%s raw=%d user_position=%d zero_raw=%d saved to %s\n",
          axis_no, axis_name(axis_no), actual, user_position,
          zero[axis_no - 1].zero_raw, AXIS_ZERO_FILE);
   return 1;
}

static int calibrate_axis_scale(EthercatScan *bus, int axis_no, double known_plc_position)
{
   AxisZero zero[PLC_AXIS_COUNT];
   int32 actual = 0;
   int32 user_position;
   double counts_per_unit;

   if (axis_no < 1 || axis_no > bus->axis_count) {
      printf("calibrate-scale: invalid axis %d, available axes: 1..%d\n",
             axis_no, bus->axis_count);
      return 0;
   }
   if (known_plc_position == 0.0) {
      printf("calibrate-scale: known PLC position must not be 0\n");
      return 0;
   }
   if (!read_axis_actual_sdo(bus, axis_no, &actual)) {
      printf("calibrate-scale: failed to read actual position for axis %d/%s\n",
             axis_no, axis_name(axis_no));
      return 0;
   }

   load_axis_zero(zero);
   if (!zero[axis_no - 1].valid) {
      printf("calibrate-scale: axis %d/%s has no zero. Run: set-zero %s first\n",
             axis_no, axis_name(axis_no), axis_name(axis_no));
      return 0;
   }

   user_position = actual - zero[axis_no - 1].zero_raw;
   if (user_position < CALIBRATE_MIN_ABS_USER_COUNTS &&
       user_position > -CALIBRATE_MIN_ABS_USER_COUNTS) {
      printf("calibrate-scale: current user position is only %d counts, too close to zero for reliable calibration\n",
             user_position);
      printf("calibrate-scale: move the axis to a known non-zero PLC position first, then run this command again\n");
      return 0;
   }

   counts_per_unit = (double)user_position / known_plc_position;
   printf("calibrate-scale: axis=%d/%s raw=%d zero_raw=%d user_position=%d known_plc_position=%.12g\n",
          axis_no, axis_name(axis_no), actual, zero[axis_no - 1].zero_raw,
          user_position, known_plc_position);
   return set_axis_scale(axis_no, counts_per_unit);
}

static void usage(const char *prog)
{
   printf("Usage:\n");
   printf("  %s IFACE\n", prog);
   printf("  %s IFACE axes\n", prog);
   printf("  %s IFACE status\n", prog);
   printf("  %s IFACE op-status\n", prog);
   printf("  %s IFACE fault-reset\n", prog);
   printf("  %s IFACE enable-dryrun\n", prog);
   printf("  %s IFACE axis-status [AXIS_OR_NAME]\n", prog);
   printf("  %s IFACE set-zero AXIS_OR_NAME [USER_POSITION]\n", prog);
   printf("  %s IFACE scale-status [AXIS_OR_NAME]\n", prog);
   printf("  %s IFACE set-scale AXIS_OR_NAME COUNTS_PER_PLC_UNIT\n", prog);
   printf("  %s IFACE calibrate-scale AXIS_OR_NAME KNOWN_PLC_POSITION\n", prog);
   printf("  %s IFACE jog-velocity AXIS_OR_NAME VELOCITY DURATION_MS\n", prog);
   printf("  %s IFACE move-relative AXIS_OR_NAME DELTA_COUNTS VELOCITY TIMEOUT_MS\n", prog);
   printf("  %s IFACE move-absolute AXIS_OR_NAME TARGET_POSITION VELOCITY TIMEOUT_MS\n", prog);
   printf("  %s IFACE move-user-absolute AXIS_OR_NAME USER_POSITION VELOCITY TIMEOUT_MS\n", prog);
   printf("  %s IFACE move-plc-absolute AXIS_OR_NAME PLC_POSITION VELOCITY TIMEOUT_MS\n", prog);
   printf("  %s IFACE move-plc-slot AXIS_OR_NAME PLC_SLOT VELOCITY TIMEOUT_MS\n", prog);
   printf("  %s IFACE move-plc-group PLC_SLOT VELOCITY TIMEOUT_MS\n", prog);
   printf("  %s IFACE plc-table\n", prog);
   printf("  %s IFACE --tcp [pc_ip] [pc_port] [status_ms]\n", prog);
   printf("\nExample:\n");
   printf("  %s eno1\n", prog);
   printf("  %s eno1 axes\n", prog);
   printf("  %s eno1 status\n", prog);
   printf("  %s eno1 op-status\n", prog);
   printf("  %s eno1 fault-reset\n", prog);
   printf("  %s eno1 enable-dryrun\n", prog);
   printf("  %s eno1 axis-status\n", prog);
   printf("  %s eno1 set-zero flex\n", prog);
   printf("  %s eno1 set-scale flex 3333.333\n", prog);
   printf("  %s eno1 calibrate-scale flex -15\n", prog);
   printf("  %s eno1 scale-status flex\n", prog);
   printf("  %s eno1 jog-velocity 1 1000 1000\n", prog);
   printf("  %s eno1 jog-velocity flex 50000 1000\n", prog);
   printf("  %s eno1 move-relative flex 50000 50000 3000\n", prog);
   printf("  %s eno1 move-absolute flex 1014500000 50000 3000\n", prog);
   printf("  %s eno1 move-user-absolute flex 50000 50000 3000\n", prog);
   printf("  %s eno1 move-plc-absolute flex -15 50000 3000\n", prog);
   printf("  %s eno1 move-plc-slot flex pos1 50000 3000\n", prog);
   printf("  %s eno1 move-plc-group pos1 50000 3000\n", prog);
   printf("  %s eno1 plc-table\n", prog);
   printf("  %s eno1 --tcp 192.168.1.200 4000 200\n", prog);
   printf("\nSafety:\n");
   printf("  Default mode scans EtherCAT and idles locally.\n");
   printf("  status mode scans EtherCAT and reads SDO diagnostics only.\n");
   printf("  op-status mode enters EtherCAT OP and reads diagnostics only.\n");
   printf("  fault-reset mode only toggles CiA402 fault reset, no enable, no motion.\n");
   printf("  enable-dryrun enables CiA402 with zero targets, then disables before exit.\n");
   printf("  jog-velocity runs one axis in PV mode; |velocity|<=%d, duration<=%d ms.\n",
          JOG_MAX_ABS_VELOCITY, JOG_MAX_DURATION_MS);
   printf("  move-relative uses PV mode with position feedback; |delta|<=%d counts, timeout<=%d ms.\n",
          MOVE_MAX_ABS_DELTA_COUNTS, MOVE_MAX_TIMEOUT_MS);
   printf("  move-absolute uses PP/Profile Position mode with raw 0x6064 encoder counts; absolute move delta is limited to +/- %d counts.\n",
          MOVE_MAX_ABS_DELTA_COUNTS);
   printf("  move-user-absolute uses saved zero offsets from %s.\n", AXIS_ZERO_FILE);
   printf("  move-plc-* uses %s plus %s, then runs PP/Profile Position mode.\n",
          AXIS_ZERO_FILE, AXIS_SCALE_FILE);
   printf("  move-plc-group moves connected axes sequentially, not synchronized interpolation.\n");
   printf("  calibrate-scale reads the current axis position after you already moved there; it does not command motion.\n");
   printf("  calibrate-scale updates %s only when the axis is far enough from zero.\n",
          AXIS_SCALE_FILE);
   printf("  plc-table prints the PLC engineering values extracted from the old PLC project.\n");
   printf("  --tcp mode is only for compatibility with the old IPC software.\n");
   printf("  It does not enable servo drives and does not command motion.\n");
   print_axis_aliases();
}

static const char *cia402_state_name(uint16 sw)
{
   if (sw & 0x0008) return "FAULT";
   if ((sw & 0x004f) == 0x0040) return "switch-on-disabled";
   if ((sw & 0x006f) == 0x0021) return "ready-to-switch-on";
   if ((sw & 0x006f) == 0x0023) return "switched-on";
   if ((sw & 0x006f) == 0x0027) return "operation-enabled";
   return "unknown";
}

static int sdo_read_u8(ecx_contextt *ctx, uint16 slave, uint16 idx, uint8 sub, uint8 *value)
{
   int size = 1;
   return ecx_SDOread(ctx, slave, idx, sub, FALSE, &size, value, EC_TIMEOUTRXM) > 0;
}

static int sdo_read_u16(ecx_contextt *ctx, uint16 slave, uint16 idx, uint8 sub, uint16 *value)
{
   int size = 2;
   if (ecx_SDOread(ctx, slave, idx, sub, FALSE, &size, value, EC_TIMEOUTRXM) <= 0) return 0;
   *value = etohs(*value);
   return 1;
}

static int sdo_read_i32(ecx_contextt *ctx, uint16 slave, uint16 idx, uint8 sub, int32 *value)
{
   int size = 4;
   if (ecx_SDOread(ctx, slave, idx, sub, FALSE, &size, value, EC_TIMEOUTRXM) <= 0) return 0;
   *value = etohl(*value);
   return 1;
}

static int sdo_write_u8(ecx_contextt *ctx, uint16 slave, uint16 idx, uint8 sub, uint8 value)
{
   return ecx_SDOwrite(ctx, slave, idx, sub, FALSE, 1, &value, EC_TIMEOUTRXM) > 0;
}

static int sdo_write_u16(ecx_contextt *ctx, uint16 slave, uint16 idx, uint8 sub, uint16 value)
{
   uint16 v = htoes(value);
   return ecx_SDOwrite(ctx, slave, idx, sub, FALSE, 2, &v, EC_TIMEOUTRXM) > 0;
}

static int sdo_write_i32(ecx_contextt *ctx, uint16 slave, uint16 idx, uint8 sub, int32 value)
{
   int32 v = htoel(value);
   return ecx_SDOwrite(ctx, slave, idx, sub, FALSE, 4, &v, EC_TIMEOUTRXM) > 0;
}

static void drain_ecat_errors(ecx_contextt *ctx)
{
   while (ctx->ecaterror) ecx_elist2string(ctx);
}

static uint8 *pdo_ptr(ec_slavet *s, const PdoEntry *e, int out)
{
   if (!e->present || (e->bit_offset & 7)) return NULL;
   return (out ? s->outputs : s->inputs) + e->bit_offset / 8;
}

static void pdo_write_u16(ec_slavet *s, const PdoEntry *e, uint16 value)
{
   uint8 *p = pdo_ptr(s, e, 1);
   if (p && e->bit_length == 16) {
      uint16 v = htoes(value);
      memcpy(p, &v, 2);
   }
}

static void pdo_write_i8(ec_slavet *s, const PdoEntry *e, int8 value)
{
   uint8 *p = pdo_ptr(s, e, 1);
   if (p && e->bit_length == 8) memcpy(p, &value, 1);
}

static uint16 pdo_read_u16(ec_slavet *s, const PdoEntry *e)
{
   uint8 *p = pdo_ptr(s, e, 0);
   uint16 value = 0;

   if (p && e->bit_length == 16) {
      memcpy(&value, p, 2);
      value = etohs(value);
   }
   return value;
}

static int32 pdo_read_i32(ec_slavet *s, const PdoEntry *e)
{
   uint8 *p = pdo_ptr(s, e, 0);
   int32 value = 0;

   if (p && e->bit_length == 32) {
      memcpy(&value, p, 4);
      value = etohl(value);
   }
   return value;
}

static int8 pdo_read_i8(ec_slavet *s, const PdoEntry *e)
{
   uint8 *p = pdo_ptr(s, e, 0);
   int8 value = 0;

   if (p && e->bit_length == 8) memcpy(&value, p, 1);
   return value;
}

static void pdo_write_i32(ec_slavet *s, const PdoEntry *e, int32 value)
{
   uint8 *p = pdo_ptr(s, e, 1);
   if (p && e->bit_length == 32) {
      int32 v = htoel(value);
      memcpy(p, &v, 4);
   }
}

static void remember_pdo_entry(ServoMap *m, uint16 idx, uint8 sub, int off, int len)
{
   PdoEntry *e = NULL;

   if (sub != 0) return;
   switch (idx) {
   case 0x603f: e = &m->error_code; break;
   case 0x6040: e = &m->controlword; break;
   case 0x6041: e = &m->statusword; break;
   case 0x6060: e = &m->modes_of_operation; break;
   case 0x6061: e = &m->mode_display; break;
   case 0x6064: e = &m->actual_position; break;
   case 0x607a: e = &m->target_position; break;
   case 0x607f: e = &m->max_profile_velocity; break;
   case 0x60ff: e = &m->target_velocity; break;
   }
   if (e) {
      e->present = 1;
      e->bit_offset = off;
      e->bit_length = len;
   }
}

static int read_pdo_map(ecx_contextt *ctx, uint16 slave, ServoMap *map, uint16 assign, int bit_offset)
{
   uint8 pdo_count = 0;
   int size = 1;

   if (ecx_SDOread(ctx, slave, assign, 0x00, FALSE, &size, &pdo_count, EC_TIMEOUTRXM) <= 0)
      return bit_offset;

   for (int i = 1; i <= pdo_count; i++) {
      uint16 pdo_index = 0;
      uint8 entry_count = 0;

      size = 2;
      if (ecx_SDOread(ctx, slave, assign, (uint8)i, FALSE, &size, &pdo_index, EC_TIMEOUTRXM) <= 0)
         return bit_offset;
      pdo_index = etohs(pdo_index);

      size = 1;
      if (ecx_SDOread(ctx, slave, pdo_index, 0x00, FALSE, &size, &entry_count, EC_TIMEOUTRXM) <= 0)
         return bit_offset;

      for (int j = 1; j <= entry_count; j++) {
         uint32 entry = 0;
         uint16 idx;
         uint8 sub, bit_len;

         size = 4;
         if (ecx_SDOread(ctx, slave, pdo_index, (uint8)j, FALSE, &size, &entry, EC_TIMEOUTRXM) <= 0)
            return bit_offset;
         entry = etohl(entry);
         idx = (uint16)(entry >> 16);
         sub = (uint8)((entry >> 8) & 0xff);
         bit_len = (uint8)(entry & 0xff);
         printf("  %04x:%02x bit:%d len:%u\n", idx, sub, bit_offset, bit_len);
         remember_pdo_entry(map, idx, sub, bit_offset, bit_len);
         bit_offset += bit_len;
      }
   }
   return bit_offset;
}

static int find_and_assign_pv_pdo(ecx_contextt *ctx, uint16 slave)
{
   int size;

   printf("slave %u: scanning fixed RxPDOs for 0x6040+0x60FF...\n", slave);
   for (uint16 idx = 0x1600; idx <= 0x17ff; idx++) {
      uint8 count = 0;
      int has_6040 = 0;
      int has_60ff = 0;

      size = 1;
      if (ecx_SDOread(ctx, slave, idx, 0x00, FALSE, &size, &count, EC_TIMEOUTRXM) <= 0) {
         drain_ecat_errors(ctx);
         continue;
      }

      for (int j = 1; j <= count; j++) {
         uint32 entry = 0;
         size = 4;
         if (ecx_SDOread(ctx, slave, idx, (uint8)j, FALSE, &size, &entry, EC_TIMEOUTRXM) > 0) {
            entry = etohl(entry);
            if ((entry >> 16) == 0x6040) has_6040 = 1;
            if ((entry >> 16) == 0x60ff) has_60ff = 1;
         }
      }

      if (has_6040 && has_60ff) {
         printf("  using RxPDO 0x%04x\n", idx);
         if (!sdo_write_u8(ctx, slave, 0x1c12, 0x00, 0)) { drain_ecat_errors(ctx); return 0; }
         if (!sdo_write_u16(ctx, slave, 0x1c12, 0x01, idx)) { drain_ecat_errors(ctx); return 0; }
         if (!sdo_write_u8(ctx, slave, 0x1c12, 0x00, 1)) { drain_ecat_errors(ctx); return 0; }
         return 1;
      }

      drain_ecat_errors(ctx);
   }

   printf("  no PV-compatible fixed RxPDO found\n");
   return 0;
}

static int pv_pdo_config_cb(ecx_contextt *ctx, uint16 slave)
{
   if (!g_pdo_bus) return 0;
   for (int i = 0; i < g_pdo_bus->axis_count; i++) {
      if (g_pdo_bus->axis[i].slave == slave)
         return find_and_assign_pv_pdo(ctx, slave);
   }
   return 0;
}

static int ethercat_scan(EthercatScan *bus, const char *ifname)
{
   int i;

   memset(bus, 0, sizeof(*bus));
   printf("ethercat: opening %s\n", ifname);
   if (!ecx_init(&bus->ctx, ifname)) {
      printf("ethercat: failed to open interface %s\n", ifname);
      return 0;
   }

   bus->initialized = 1;
   bus->slave_count = ecx_config_init(&bus->ctx);
   if (bus->slave_count <= 0) {
      printf("ethercat: no slaves found\n");
      return 0;
   }

   bus->ctx.manualstatechange = 1;
   bus->axis_count = bus->slave_count < PLC_AXIS_COUNT ? bus->slave_count : PLC_AXIS_COUNT;
   g_pdo_bus = bus;

   printf("ethercat: found %d slave(s)\n", bus->slave_count);
   for (i = 1; i <= bus->slave_count; i++) {
      ec_slavet *s = &bus->ctx.slavelist[i];
      printf("  slave %d: name='%s' eep_man=0x%08x eep_id=0x%08x Obits=%d Ibits=%d state=0x%02x\n",
             i, s->name, (unsigned)s->eep_man, (unsigned)s->eep_id,
             s->Obits, s->Ibits, s->state);
   }

   if (bus->slave_count != PLC_AXIS_COUNT) {
      printf("ethercat: warning: original drawing expected %d servo slaves, found %d\n",
             PLC_AXIS_COUNT, bus->slave_count);
      printf("ethercat: this is OK for a bench with only part of the old machine connected\n");
   }

   /*
    * Map PDOs so that future versions can read status words here.
    * We still keep drives not enabled and do not write any motion outputs.
    */
   for (i = 0; i < bus->axis_count; i++) {
      bus->axis[i].slave = (uint16)(i + 1);
      bus->ctx.slavelist[bus->axis[i].slave].PO2SOconfig = pv_pdo_config_cb;
   }
   ecx_config_map_group(&bus->ctx, bus->iomap, 0);

   for (i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      printf("axis %d/%s slave %u RxPDO:\n", i + 1, axis_name(i + 1), axis->slave);
      read_pdo_map(&bus->ctx, axis->slave, &axis->map, 0x1c12, 0);
      printf("axis %d/%s slave %u TxPDO:\n", i + 1, axis_name(i + 1), axis->slave);
      read_pdo_map(&bus->ctx, axis->slave, &axis->map, 0x1c13, 0);
      drain_ecat_errors(&bus->ctx);
   }

   ecx_configdc(&bus->ctx);
   for (i = 1; i <= bus->slave_count; i++) {
      if (bus->ctx.slavelist[i].hasdc)
         ecx_dcsync0(&bus->ctx, i, TRUE, 1000000, 0);
   }
   printf("ethercat: PDO map prepared, not switching to servo enable\n");
   return 1;
}

static int ethercat_roundtrip(EthercatScan *bus)
{
   ecx_send_processdata(&bus->ctx);
   return ecx_receive_processdata(&bus->ctx, EC_TIMEOUTRET);
}

static int ethercat_wait_state(EthercatScan *bus, uint16 state, const char *name, int loops)
{
   int i;

   for (i = 0; i < loops; i++) {
      ethercat_roundtrip(bus);
      ecx_statecheck(&bus->ctx, 0, state, 1000);
      if (bus->ctx.slavelist[0].state == state) {
         printf("ethercat: all slaves reached %s\n", name);
         return 1;
      }
      osal_usleep(1000);
   }

   printf("ethercat: failed to reach %s, current state=0x%02x\n",
          name, bus->ctx.slavelist[0].state);
   for (i = 1; i <= bus->slave_count; i++) {
      ec_slavet *s = &bus->ctx.slavelist[i];
      printf("  slave %d state=0x%02x ALstatuscode=0x%04x\n",
             i, s->state, s->ALstatuscode);
   }
   return 0;
}

static void ethercat_ack_errors(EthercatScan *bus)
{
   int i;
   int need_ack = 0;

   ecx_readstate(&bus->ctx);
   for (i = 1; i <= bus->slave_count; i++) {
      if (bus->ctx.slavelist[i].state & EC_STATE_ERROR) {
         need_ack = 1;
         printf("ethercat: ACK error on slave %d (state=0x%02x ALstatuscode=0x%04x)\n",
                i,
                bus->ctx.slavelist[i].state,
                bus->ctx.slavelist[i].ALstatuscode);
         bus->ctx.slavelist[i].state =
            (bus->ctx.slavelist[i].state & 0x0f) | EC_STATE_ACK;
         ecx_writestate(&bus->ctx, (uint16)i);
      }
   }

   if (!need_ack) return;

   osal_usleep(10000);
   ecx_readstate(&bus->ctx);
}

static int ethercat_enter_preop(EthercatScan *bus)
{
   ethercat_ack_errors(bus);
   printf("ethercat: requesting PRE_OP\n");
   bus->ctx.slavelist[0].state = EC_STATE_PRE_OP;
   ecx_writestate(&bus->ctx, 0);
   return ethercat_wait_state(bus, EC_STATE_PRE_OP, "PRE_OP", 500);
}

static int ethercat_enter_op(EthercatScan *bus)
{
   int expected_wkc;

   if (!ethercat_enter_preop(bus)) return 0;

   for (int i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      uint16 sm = htoes(2);
      uint32 cycle = htoel(1000000);
      ecx_SDOwrite(&bus->ctx, axis->slave, 0x1c32, 0x01, FALSE, 2, &sm, EC_TIMEOUTRXM);
      ecx_SDOwrite(&bus->ctx, axis->slave, 0x1c33, 0x01, FALSE, 2, &sm, EC_TIMEOUTRXM);
      ecx_SDOwrite(&bus->ctx, axis->slave, 0x1c32, 0x02, FALSE, 4, &cycle, EC_TIMEOUTRXM);
      ecx_SDOwrite(&bus->ctx, axis->slave, 0x1c33, 0x02, FALSE, 4, &cycle, EC_TIMEOUTRXM);
   }
   drain_ecat_errors(&bus->ctx);

   for (int i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      pdo_write_i8(slave, &axis->map.modes_of_operation, MODE_PV);
      pdo_write_u16(slave, &axis->map.controlword, 0x0000);
      pdo_write_i32(slave, &axis->map.target_velocity, 0);
   }

   printf("ethercat: requesting SAFE_OP\n");
   bus->ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(&bus->ctx, 0);
   if (!ethercat_wait_state(bus, EC_STATE_SAFE_OP, "SAFE_OP", 200)) return 0;

   expected_wkc = bus->ctx.grouplist[0].outputsWKC * 2 + bus->ctx.grouplist[0].inputsWKC;
   printf("ethercat: expected WKC=%d (outputsWKC=%d inputsWKC=%d)\n",
          expected_wkc,
          bus->ctx.grouplist[0].outputsWKC,
          bus->ctx.grouplist[0].inputsWKC);

   /*
    * Send a few zeroed process-data frames before OP. The IOmap was zeroed
    * in ethercat_scan(), so this does not request enable or motion.
    */
   for (int i = 0; i < 50; i++) {
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }

   printf("ethercat: requesting OP\n");
   bus->ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(&bus->ctx, 0);
   if (!ethercat_wait_state(bus, EC_STATE_OPERATIONAL, "OP", 500)) return 0;

   for (int i = 0; i < 20; i++) {
      int wkc = ethercat_roundtrip(bus);
      if (i == 0 || i == 19) printf("ethercat: processdata WKC=%d\n", wkc);
      osal_usleep(1000);
   }

   return 1;
}

static void ethercat_request_state(EthercatScan *bus, uint16 state, const char *name)
{
   printf("ethercat: returning to %s\n", name);
   bus->ctx.slavelist[0].state = state;
   ecx_writestate(&bus->ctx, 0);
   if (!ethercat_wait_state(bus, state, name, 200)) {
      ethercat_ack_errors(bus);
      bus->ctx.slavelist[0].state = state;
      ecx_writestate(&bus->ctx, 0);
      ethercat_wait_state(bus, state, name, 200);
   }
}

static void ethercat_close(EthercatScan *bus)
{
   if (!bus->initialized) return;

   ecx_readstate(&bus->ctx);
   if (bus->ctx.slavelist[0].state == EC_STATE_OPERATIONAL) {
      for (int cycle = 0; cycle < 20; cycle++) {
         for (int i = 0; i < bus->axis_count; i++) {
            EmuAxis *axis = &bus->axis[i];
            ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
            pdo_write_i8(slave, &axis->map.modes_of_operation, MODE_PV);
            pdo_write_u16(slave, &axis->map.controlword, 0x0000);
            pdo_write_i32(slave, &axis->map.target_velocity, 0);
         }
         ethercat_roundtrip(bus);
         osal_usleep(1000);
      }
      ethercat_request_state(bus, EC_STATE_SAFE_OP, "SAFE_OP");
      ethercat_request_state(bus, EC_STATE_PRE_OP, "PRE_OP");
      ethercat_request_state(bus, EC_STATE_INIT, "INIT");
   } else if (bus->ctx.slavelist[0].state == EC_STATE_SAFE_OP) {
      ethercat_request_state(bus, EC_STATE_PRE_OP, "PRE_OP");
      ethercat_request_state(bus, EC_STATE_INIT, "INIT");
   } else if (bus->ctx.slavelist[0].state == EC_STATE_PRE_OP) {
      ethercat_request_state(bus, EC_STATE_INIT, "INIT");
   }
   ecx_close(&bus->ctx);
   bus->initialized = 0;
}

static void print_status_sdo(EthercatScan *bus)
{
   int i;

   ecx_readstate(&bus->ctx);
   printf("plc-emulator: SDO status read, no servo enable, no motion\n");
   for (i = 1; i <= bus->slave_count; i++) {
      ec_slavet *s = &bus->ctx.slavelist[i];
      uint16 statusword = 0;
      uint16 error_code = 0;
      uint8 mode_display = 0;
      int32 actual_position = 0;

      if (i <= PLC_AXIS_COUNT)
         printf("axis %d/%s slave %d: name='%s' state=0x%02x\n",
                i, axis_name(i), i, s->name, s->state);
      else
         printf("slave %d: name='%s' state=0x%02x\n", i, s->name, s->state);

      if (sdo_read_u16(&bus->ctx, (uint16)i, 0x6041, 0x00, &statusword))
         printf("  0x6041 statusword      : 0x%04x (%s)\n",
                statusword, cia402_state_name(statusword));
      else
         printf("  0x6041 statusword      : read failed\n");

      if (sdo_read_u16(&bus->ctx, (uint16)i, 0x603f, 0x00, &error_code))
         printf("  0x603f error_code      : 0x%04x\n", error_code);
      else
         printf("  0x603f error_code      : read failed\n");

      if (sdo_read_u8(&bus->ctx, (uint16)i, 0x6061, 0x00, &mode_display))
         printf("  0x6061 mode_display    : %d\n", (int8)mode_display);
      else
         printf("  0x6061 mode_display    : read failed\n");

      if (sdo_read_i32(&bus->ctx, (uint16)i, 0x6064, 0x00, &actual_position))
         printf("  0x6064 actual_position : %d\n", actual_position);
      else
         printf("  0x6064 actual_position : read failed\n");
   }
}

static void print_status_pdo(EthercatScan *bus)
{
   int i;

   for (int cycle = 0; cycle < 5; cycle++) {
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }

   ecx_readstate(&bus->ctx);
   printf("plc-emulator: PDO status read in OP\n");
   for (i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      ec_slavet *s = &bus->ctx.slavelist[axis->slave];
      uint16 statusword = pdo_read_u16(s, &axis->map.statusword);
      uint16 error_code = pdo_read_u16(s, &axis->map.error_code);
      int8 mode_display = pdo_read_i8(s, &axis->map.mode_display);
      int32 actual_position = pdo_read_i32(s, &axis->map.actual_position);

      printf("axis %d/%s slave %u: name='%s' state=0x%02x\n",
             i + 1, axis_name(i + 1), axis->slave, s->name, s->state);
      printf("  PDO 0x6041 statusword      : 0x%04x (%s)\n",
             statusword, cia402_state_name(statusword));
      printf("  PDO 0x603f error_code      : 0x%04x\n", error_code);
      if (axis->map.mode_display.present)
         printf("  PDO 0x6061 mode_display    : %d\n", mode_display);
      else
         printf("  PDO 0x6061 mode_display    : not mapped\n");
      printf("  PDO 0x6064 actual_position : %d\n", actual_position);

      ethercat_roundtrip(bus);
   }
}

static void fault_reset_axes(EthercatScan *bus)
{
   printf("plc-emulator: CiA402 fault reset only, no enable, no motion\n");

   for (int i = 0; i < 10; i++) {
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }

   for (int pulse = 0; pulse < 20; pulse++) {
      for (int axis_no = 0; axis_no < bus->axis_count; axis_no++) {
         EmuAxis *axis = &bus->axis[axis_no];
         ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
         uint16 statusword = pdo_read_u16(slave, &axis->map.statusword);

         pdo_write_i8(slave, &axis->map.modes_of_operation, MODE_PV);
         pdo_write_i32(slave, &axis->map.target_velocity, 0);
         if (statusword & 0x0008) {
            if (pulse == 0)
               printf("  axis %d/%s slave %u fault reset pulse (statusword=0x%04x)\n",
                      axis_no + 1, axis_name(axis_no + 1), axis->slave, statusword);
            pdo_write_u16(slave, &axis->map.controlword, 0x0080);
         } else {
            if (pulse == 0)
               printf("  axis %d/%s slave %u not in fault (statusword=0x%04x), skip reset pulse\n",
                      axis_no + 1, axis_name(axis_no + 1), axis->slave, statusword);
            pdo_write_u16(slave, &axis->map.controlword, 0x0000);
         }
      }
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }

   for (int i = 0; i < 50; i++) {
      for (int axis_no = 0; axis_no < bus->axis_count; axis_no++) {
         EmuAxis *axis = &bus->axis[axis_no];
         ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
         pdo_write_i8(slave, &axis->map.modes_of_operation, MODE_PV);
         pdo_write_u16(slave, &axis->map.controlword, 0x0000);
         pdo_write_i32(slave, &axis->map.target_velocity, 0);
      }
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }
}

static int all_axes_operation_enabled(EthercatScan *bus)
{
   int ok = 1;

   for (int axis_no = 0; axis_no < bus->axis_count; axis_no++) {
      EmuAxis *axis = &bus->axis[axis_no];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      uint16 statusword = pdo_read_u16(slave, &axis->map.statusword);

      if ((statusword & 0x006f) != 0x0027) ok = 0;
   }
   return ok;
}

static void write_all_axes_control(EthercatScan *bus, uint16 controlword)
{
   for (int axis_no = 0; axis_no < bus->axis_count; axis_no++) {
      EmuAxis *axis = &bus->axis[axis_no];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];

      pdo_write_i8(slave, &axis->map.modes_of_operation, MODE_PV);
      pdo_write_i32(slave, &axis->map.max_profile_velocity, JOG_MAX_ABS_VELOCITY);
      pdo_write_i32(slave, &axis->map.target_velocity, 0);
      pdo_write_u16(slave, &axis->map.controlword, controlword);
   }
}

static void enable_dryrun_axes(EthercatScan *bus)
{
   const uint16 seq[] = { 0x0006, 0x0007, 0x000f };
   const char *name[] = { "shutdown", "switch-on", "enable-operation" };

   printf("plc-emulator: CiA402 enable dry-run, target_velocity=0, no motion command\n");

   for (int step = 0; step < 3; step++) {
      printf("  step %d: %s controlword=0x%04x\n", step + 1, name[step], seq[step]);
      for (int cycle = 0; cycle < 500; cycle++) {
         write_all_axes_control(bus, seq[step]);
         ethercat_roundtrip(bus);
         osal_usleep(1000);

         if (step == 2 && all_axes_operation_enabled(bus)) {
            printf("  all axes reached operation-enabled\n");
            print_status_pdo(bus);
            goto hold_enabled;
         }
      }
      print_status_pdo(bus);
   }

hold_enabled:
   printf("  holding enable dry-run for 1000 ms with zero target velocity\n");
   for (int cycle = 0; cycle < 1000; cycle++) {
      write_all_axes_control(bus, 0x000f);
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }
   print_status_pdo(bus);

   printf("  disabling axes before exit\n");
   for (int cycle = 0; cycle < 200; cycle++) {
      write_all_axes_control(bus, 0x0000);
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }
   print_status_pdo(bus);
}

static int enable_axes_zero_velocity(EthercatScan *bus)
{
   const uint16 seq[] = { 0x0006, 0x0007, 0x000f };
   const char *name[] = { "shutdown", "switch-on", "enable-operation" };

   for (int step = 0; step < 3; step++) {
      printf("  enable step %d: %s controlword=0x%04x\n",
             step + 1, name[step], seq[step]);
      for (int cycle = 0; cycle < 1000; cycle++) {
         write_all_axes_control(bus, seq[step]);
         ethercat_roundtrip(bus);
         osal_usleep(1000);
         if (step == 2 && all_axes_operation_enabled(bus)) {
            printf("  all axes reached operation-enabled\n");
            return 1;
         }
      }
      print_status_pdo(bus);
   }

   return all_axes_operation_enabled(bus);
}

static void disable_axes_zero_velocity(EthercatScan *bus)
{
   printf("  disabling axes, target_velocity=0\n");
   for (int cycle = 0; cycle < 300; cycle++) {
      write_all_axes_control(bus, 0x0000);
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }
}

static void write_jog_outputs(EthercatScan *bus, int axis_index, int32 velocity)
{
   for (int i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];

      pdo_write_i8(slave, &axis->map.modes_of_operation, MODE_PV);
      pdo_write_i32(slave, &axis->map.max_profile_velocity, JOG_MAX_ABS_VELOCITY);
      pdo_write_i32(slave, &axis->map.target_velocity, i == axis_index ? velocity : 0);
      pdo_write_u16(slave, &axis->map.controlword, 0x000f);
   }
}

static void write_pp_outputs(EthercatScan *bus, int axis_index, int32 target, uint16 controlword)
{
   for (int i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      int32 hold_position = pdo_read_i32(slave, &axis->map.actual_position);
      uint16 cw = i == axis_index ? controlword : (uint16)(controlword & ~(0x0010 | 0x0020 | 0x0040));

      pdo_write_i8(slave, &axis->map.modes_of_operation, MODE_PP);
      pdo_write_i32(slave, &axis->map.target_velocity, 0);
      pdo_write_i32(slave, &axis->map.max_profile_velocity, JOG_MAX_ABS_VELOCITY);
      pdo_write_i32(slave, &axis->map.target_position, i == axis_index ? target : hold_position);
      pdo_write_u16(slave, &axis->map.controlword, cw);
   }
}

static int enable_axes_pp_hold(EthercatScan *bus, int axis_index, int32 target)
{
   const uint16 seq[] = { 0x0006, 0x0007, 0x000f };
   const char *name[] = { "shutdown", "switch-on", "enable-operation" };

   for (int step = 0; step < 3; step++) {
      printf("  PP enable step %d: %s controlword=0x%04x\n",
             step + 1, name[step], seq[step]);
      for (int cycle = 0; cycle < 1000; cycle++) {
         write_pp_outputs(bus, axis_index, target, seq[step]);
         ethercat_roundtrip(bus);
         osal_usleep(1000);
         if (step == 2 && all_axes_operation_enabled(bus)) {
            printf("  all axes reached operation-enabled\n");
            return 1;
         }
      }
      print_status_pdo(bus);
   }

   return all_axes_operation_enabled(bus);
}

static void disable_axes_pp(EthercatScan *bus)
{
   printf("  disabling axes in PP mode\n");
   for (int cycle = 0; cycle < 300; cycle++) {
      for (int i = 0; i < bus->axis_count; i++) {
         EmuAxis *axis = &bus->axis[i];
         ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
         int32 hold_position = pdo_read_i32(slave, &axis->map.actual_position);

         pdo_write_i8(slave, &axis->map.modes_of_operation, MODE_PP);
         pdo_write_i32(slave, &axis->map.target_position, hold_position);
         pdo_write_u16(slave, &axis->map.controlword, 0x0000);
      }
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }
}

static int any_axis_fault(EthercatScan *bus)
{
   int fault = 0;

   for (int i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      uint16 statusword = pdo_read_u16(slave, &axis->map.statusword);

      if (statusword & 0x0008) {
         printf("  axis %d/%s slave %u FAULT statusword=0x%04x error=0x%04x\n",
                i + 1, axis_name(i + 1), axis->slave, statusword,
                pdo_read_u16(slave, &axis->map.error_code));
         fault = 1;
      }
   }
   return fault;
}

static int configure_jog_profile(EthercatScan *bus)
{
   int ok = 1;

   printf("plc-emulator: configure PV profile accel=%d decel=%d max_velocity=%d\n",
          JOG_PROFILE_ACCEL, JOG_PROFILE_DECEL, JOG_MAX_ABS_VELOCITY);
   for (int i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      int axis_ok = 1;

      if (!sdo_write_u8(&bus->ctx, axis->slave, 0x6060, 0x00, MODE_PV)) axis_ok = 0;
      if (!sdo_write_i32(&bus->ctx, axis->slave, 0x6083, 0x00, JOG_PROFILE_ACCEL)) axis_ok = 0;
      if (!sdo_write_i32(&bus->ctx, axis->slave, 0x6084, 0x00, JOG_PROFILE_DECEL)) axis_ok = 0;
      if (!sdo_write_i32(&bus->ctx, axis->slave, 0x607f, 0x00, JOG_MAX_ABS_VELOCITY)) axis_ok = 0;
      drain_ecat_errors(&bus->ctx);

      if (!axis_ok) {
         printf("  axis %d/%s slave %u: PV profile SDO write failed\n",
                i + 1, axis_name(i + 1), axis->slave);
         ok = 0;
      } else {
         printf("  axis %d/%s slave %u: PV profile OK\n",
                i + 1, axis_name(i + 1), axis->slave);
      }
   }

   return ok;
}

static int configure_pp_profile(EthercatScan *bus, int32 profile_velocity)
{
   int ok = 1;

   printf("plc-emulator: configure PP profile velocity=%d accel=%d decel=%d max_velocity=%d\n",
          profile_velocity, JOG_PROFILE_ACCEL, JOG_PROFILE_DECEL, JOG_MAX_ABS_VELOCITY);
   for (int i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      int axis_ok = 1;

      if (!sdo_write_u8(&bus->ctx, axis->slave, 0x6060, 0x00, MODE_PP)) axis_ok = 0;
      if (!sdo_write_i32(&bus->ctx, axis->slave, 0x6081, 0x00, profile_velocity)) axis_ok = 0;
      if (!sdo_write_i32(&bus->ctx, axis->slave, 0x6083, 0x00, JOG_PROFILE_ACCEL)) axis_ok = 0;
      if (!sdo_write_i32(&bus->ctx, axis->slave, 0x6084, 0x00, JOG_PROFILE_DECEL)) axis_ok = 0;
      if (!sdo_write_i32(&bus->ctx, axis->slave, 0x607f, 0x00, JOG_MAX_ABS_VELOCITY)) axis_ok = 0;
      drain_ecat_errors(&bus->ctx);

      if (!axis_ok) {
         printf("  axis %d/%s slave %u: PP profile SDO write failed\n",
                i + 1, axis_name(i + 1), axis->slave);
         ok = 0;
      } else {
         printf("  axis %d/%s slave %u: PP profile OK\n",
                i + 1, axis_name(i + 1), axis->slave);
      }
   }

   return ok;
}

static int validate_jog_limits(int axis_no, int32 velocity, int duration_ms)
{
   if (axis_no < 1 || axis_no > PLC_AXIS_COUNT) {
      printf("jog: invalid axis %d, configured max axes: 1..%d\n", axis_no, PLC_AXIS_COUNT);
      return 0;
   }
   if (velocity > JOG_MAX_ABS_VELOCITY || velocity < -JOG_MAX_ABS_VELOCITY) {
      printf("jog: velocity %d exceeds safety limit +/- %d\n",
             velocity, JOG_MAX_ABS_VELOCITY);
      return 0;
   }
   if (duration_ms < 1 || duration_ms > JOG_MAX_DURATION_MS) {
      printf("jog: duration %d ms exceeds safety limit 1..%d ms\n",
             duration_ms, JOG_MAX_DURATION_MS);
      return 0;
   }
   return 1;
}

static int validate_move_limits(int axis_no, int32 delta, int32 velocity, int timeout_ms)
{
   if (axis_no < 1 || axis_no > PLC_AXIS_COUNT) {
      printf("move-relative: invalid axis %d, configured max axes: 1..%d\n",
             axis_no, PLC_AXIS_COUNT);
      return 0;
   }
   if (delta == 0) {
      printf("move-relative: delta must not be 0\n");
      return 0;
   }
   if ((long long)delta > MOVE_MAX_ABS_DELTA_COUNTS ||
       (long long)delta < -MOVE_MAX_ABS_DELTA_COUNTS) {
      printf("move-relative: delta %d exceeds safety limit +/- %d counts\n",
             delta, MOVE_MAX_ABS_DELTA_COUNTS);
      return 0;
   }
   if (velocity <= 0 || velocity > JOG_MAX_ABS_VELOCITY) {
      printf("move-relative: velocity %d exceeds safety limit 1..%d\n",
             velocity, JOG_MAX_ABS_VELOCITY);
      return 0;
   }
   if (timeout_ms < 1 || timeout_ms > MOVE_MAX_TIMEOUT_MS) {
      printf("move-relative: timeout %d ms exceeds safety limit 1..%d ms\n",
             timeout_ms, MOVE_MAX_TIMEOUT_MS);
      return 0;
   }
   return 1;
}

static int validate_absolute_move_limits(int axis_no, int32 target, int32 velocity, int timeout_ms)
{
   (void)target;
   if (axis_no < 1 || axis_no > PLC_AXIS_COUNT) {
      printf("move-absolute: invalid axis %d, configured max axes: 1..%d\n",
             axis_no, PLC_AXIS_COUNT);
      return 0;
   }
   if (velocity <= 0 || velocity > JOG_MAX_ABS_VELOCITY) {
      printf("move-absolute: velocity %d exceeds safety limit 1..%d\n",
             velocity, JOG_MAX_ABS_VELOCITY);
      return 0;
   }
   if (timeout_ms < 1 || timeout_ms > MOVE_MAX_TIMEOUT_MS) {
      printf("move-absolute: timeout %d ms exceeds safety limit 1..%d ms\n",
             timeout_ms, MOVE_MAX_TIMEOUT_MS);
      return 0;
   }
   return 1;
}

static int32 move_command_velocity(int32 velocity, long long remaining_abs)
{
   long long cmd = velocity;

   if (remaining_abs < MOVE_SLOWDOWN_COUNTS) {
      long long scaled = remaining_abs * 2;
      if (scaled < MOVE_MIN_ABS_VELOCITY) scaled = MOVE_MIN_ABS_VELOCITY;
      if (scaled < cmd) cmd = scaled;
   }

   if (cmd > velocity) cmd = velocity;
   if (cmd > JOG_MAX_ABS_VELOCITY) cmd = JOG_MAX_ABS_VELOCITY;
   return (int32)cmd;
}

static int jog_velocity_axis(EthercatScan *bus, int axis_no, int32 velocity, int duration_ms)
{
   int axis_index = axis_no - 1;
   int32 start_pos[PLC_AXIS_COUNT] = { 0 };
   int32 last_report_pos = 0;

   if (axis_no < 1 || axis_no > bus->axis_count) {
      printf("jog: invalid axis %d, available axes: 1..%d\n", axis_no, bus->axis_count);
      return 0;
   }
   if (!validate_jog_limits(axis_no, velocity, duration_ms)) return 0;

   printf("plc-emulator: jog velocity axis=%d/%s velocity=%d duration=%d ms\n",
          axis_no, axis_name(axis_no), velocity, duration_ms);
   printf("plc-emulator: PV mode, single axis only, other axes target_velocity=0\n");

   if (!enable_axes_zero_velocity(bus)) {
      printf("jog: enable failed, aborting without motion command\n");
      print_status_pdo(bus);
      disable_axes_zero_velocity(bus);
      return 0;
   }

   print_status_pdo(bus);
   for (int i = 0; i < bus->axis_count; i++) {
      EmuAxis *axis = &bus->axis[i];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      start_pos[i] = pdo_read_i32(slave, &axis->map.actual_position);
   }
   last_report_pos = start_pos[axis_index];

   if (any_axis_fault(bus)) {
      printf("jog: fault detected before motion, aborting\n");
      disable_axes_zero_velocity(bus);
      return 0;
   }

   printf("jog: running. Ctrl-C requests stop.\n");
   for (int elapsed = 0; elapsed < duration_ms && g_running; elapsed++) {
      write_jog_outputs(bus, axis_index, velocity);
      ethercat_roundtrip(bus);
      osal_usleep(1000);
      if ((elapsed % 100) == 0) {
         EmuAxis *axis = &bus->axis[axis_index];
         ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
         int32 pos = pdo_read_i32(slave, &axis->map.actual_position);
         printf("  jog t=%d ms axis=%d/%s pos=%d delta=%d step_delta=%d\n",
                elapsed, axis_no, axis_name(axis_no), pos,
                pos - start_pos[axis_index], pos - last_report_pos);
         last_report_pos = pos;
         if (any_axis_fault(bus)) break;
      }
   }

   printf("jog: stopping target velocity\n");
   for (int cycle = 0; cycle < 500; cycle++) {
      write_jog_outputs(bus, axis_index, 0);
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }
   print_status_pdo(bus);
   {
      EmuAxis *axis = &bus->axis[axis_index];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      int32 end_pos = pdo_read_i32(slave, &axis->map.actual_position);
      printf("jog: final axis=%d/%s start_pos=%d end_pos=%d delta=%d\n",
             axis_no, axis_name(axis_no), start_pos[axis_index],
             end_pos, end_pos - start_pos[axis_index]);
   }
   disable_axes_zero_velocity(bus);
   print_status_pdo(bus);
   return 1;
}

static int move_axis_to_target(EthercatScan *bus, int axis_no, int32 start_pos,
                               long long target_pos, int32 velocity, int timeout_ms,
                               const char *label, int32 target_delta)
{
   int axis_index = axis_no - 1;
   int reached = 0;
   int fault = 0;
   int32 last_report_pos = 0;
   long long initial_remaining = target_pos - (long long)start_pos;
   int direction = initial_remaining > 0 ? 1 : -1;

   if (any_axis_fault(bus)) {
      printf("%s: fault detected before motion, aborting\n", label);
      return 0;
   }

   if (initial_remaining == 0) {
      printf("%s: already at target start=%d target=%lld\n", label, start_pos, target_pos);
      return 1;
   }

   last_report_pos = start_pos;
   printf("%s: start=%d target=%lld target_delta=%d\n",
          label, start_pos, target_pos, target_delta);
   for (int elapsed = 0; elapsed < timeout_ms && g_running; elapsed++) {
      EmuAxis *axis = &bus->axis[axis_index];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      int32 pos = pdo_read_i32(slave, &axis->map.actual_position);
      long long remaining = target_pos - (long long)pos;
      long long remaining_abs = remaining >= 0 ? remaining : -remaining;
      int32 cmd_abs;
      int32 cmd;

      if ((direction > 0 && remaining <= MOVE_POSITION_TOLERANCE_COUNTS) ||
          (direction < 0 && remaining >= -MOVE_POSITION_TOLERANCE_COUNTS)) {
         reached = 1;
         printf("%s: target reached at t=%d ms pos=%d remaining=%lld\n",
                label, elapsed, pos, remaining);
         break;
      }

      cmd_abs = move_command_velocity(velocity, remaining_abs);
      cmd = direction > 0 ? cmd_abs : -cmd_abs;
      write_jog_outputs(bus, axis_index, cmd);
      ethercat_roundtrip(bus);
      osal_usleep(1000);

      if ((elapsed % 100) == 0) {
         printf("  %s t=%d ms axis=%d/%s pos=%d moved=%d remaining=%lld cmd=%d step_delta=%d\n",
                label, elapsed, axis_no, axis_name(axis_no), pos,
                pos - start_pos, remaining, cmd, pos - last_report_pos);
         last_report_pos = pos;
         if (any_axis_fault(bus)) {
            fault = 1;
            break;
         }
      }
   }

   if (!reached && !fault) {
      printf("%s: timeout/stop before target, commanding zero velocity\n", label);
   }

   for (int cycle = 0; cycle < 500; cycle++) {
      write_jog_outputs(bus, axis_index, 0);
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }
   print_status_pdo(bus);
   {
      EmuAxis *axis = &bus->axis[axis_index];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      int32 end_pos = pdo_read_i32(slave, &axis->map.actual_position);
      printf("%s: final axis=%d/%s start_pos=%d end_pos=%d delta=%d target_delta=%d error=%lld reached=%d\n",
             label, axis_no, axis_name(axis_no), start_pos, end_pos, end_pos - start_pos,
             target_delta, target_pos - (long long)end_pos, reached);
   }
   return reached && !fault;
}

static int move_relative_axis(EthercatScan *bus, int axis_no, int32 delta, int32 velocity, int timeout_ms)
{
   int axis_index = axis_no - 1;
   int32 start_pos = 0;
   int ok;

   if (axis_no < 1 || axis_no > bus->axis_count) {
      printf("move-relative: invalid axis %d, available axes: 1..%d\n",
             axis_no, bus->axis_count);
      return 0;
   }
   if (!validate_move_limits(axis_no, delta, velocity, timeout_ms)) return 0;

   printf("plc-emulator: move-relative axis=%d/%s delta=%d velocity=%d timeout=%d ms\n",
          axis_no, axis_name(axis_no), delta, velocity, timeout_ms);
   printf("plc-emulator: PV mode with master-side position stop, tolerance=%d counts\n",
          MOVE_POSITION_TOLERANCE_COUNTS);

   if (!enable_axes_zero_velocity(bus)) {
      printf("move-relative: enable failed, aborting without motion command\n");
      print_status_pdo(bus);
      disable_axes_zero_velocity(bus);
      return 0;
   }

   print_status_pdo(bus);
   {
      EmuAxis *axis = &bus->axis[axis_index];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      start_pos = pdo_read_i32(slave, &axis->map.actual_position);
   }

   ok = move_axis_to_target(bus, axis_no, start_pos, (long long)start_pos + delta,
                            velocity, timeout_ms, "move-relative", delta);
   disable_axes_zero_velocity(bus);
   print_status_pdo(bus);
   return ok;
}

static int move_absolute_axis(EthercatScan *bus, int axis_no, int32 target, int32 velocity, int timeout_ms)
{
   int axis_index = axis_no - 1;
   int32 start_pos = 0;
   int32 end_pos = 0;
   long long delta;
   int reached = 0;
   int fault = 0;

   if (axis_no < 1 || axis_no > bus->axis_count) {
      printf("move-absolute: invalid axis %d, available axes: 1..%d\n",
             axis_no, bus->axis_count);
      return 0;
   }
   if (!validate_absolute_move_limits(axis_no, target, velocity, timeout_ms)) return 0;

   printf("plc-emulator: move-absolute axis=%d/%s target=%d velocity=%d timeout=%d ms\n",
          axis_no, axis_name(axis_no), target, velocity, timeout_ms);
   printf("plc-emulator: PP/Profile Position mode, tolerance=%d counts\n",
          MOVE_POSITION_TOLERANCE_COUNTS);

   {
      EmuAxis *axis = &bus->axis[axis_index];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];

      if (!axis->map.target_position.present || !axis->map.controlword.present ||
          !axis->map.statusword.present || !axis->map.modes_of_operation.present) {
         printf("move-absolute: axis %d/%s missing PP PDO entries 0x607A/0x6040/0x6041/0x6060\n",
                axis_no, axis_name(axis_no));
         return 0;
      }
      start_pos = pdo_read_i32(slave, &axis->map.actual_position);
   }
   delta = (long long)target - (long long)start_pos;
   if (delta > MOVE_MAX_ABS_DELTA_COUNTS || delta < -MOVE_MAX_ABS_DELTA_COUNTS) {
      printf("move-absolute: target too far from current position start=%d target=%d delta=%lld limit=+/- %d\n",
             start_pos, target, delta, MOVE_MAX_ABS_DELTA_COUNTS);
      return 0;
   }

   if (delta == 0) {
      printf("move-absolute: already at target start=%d target=%d\n", start_pos, target);
      return 1;
   }

   if (!enable_axes_pp_hold(bus, axis_index, target)) {
      printf("move-absolute: PP enable failed, aborting without motion command\n");
      print_status_pdo(bus);
      disable_axes_pp(bus);
      return 0;
   }

   print_status_pdo(bus);
   if (any_axis_fault(bus)) {
      printf("move-absolute: fault detected before PP set-point trigger, aborting\n");
      disable_axes_pp(bus);
      return 0;
   }

   printf("move-absolute: PP trigger start=%d target=%d delta=%lld\n",
          start_pos, target, delta);
   for (int cycle = 0; cycle < 20; cycle++) {
      write_pp_outputs(bus, axis_index, target, 0x000f);
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }
   for (int cycle = 0; cycle < 50; cycle++) {
      write_pp_outputs(bus, axis_index, target, 0x003f);
      ethercat_roundtrip(bus);
      osal_usleep(1000);
   }

   for (int elapsed = 0; elapsed < timeout_ms && g_running; elapsed++) {
      EmuAxis *axis = &bus->axis[axis_index];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      uint16 statusword;
      int32 pos;
      long long error;

      write_pp_outputs(bus, axis_index, target, 0x000f);
      ethercat_roundtrip(bus);
      osal_usleep(1000);

      statusword = pdo_read_u16(slave, &axis->map.statusword);
      pos = pdo_read_i32(slave, &axis->map.actual_position);
      error = (long long)target - (long long)pos;
      if (error < 0) error = -error;

      if (statusword & 0x0008) {
         printf("move-absolute: FAULT at t=%d ms statusword=0x%04x error_code=0x%04x\n",
                elapsed, statusword, pdo_read_u16(slave, &axis->map.error_code));
         fault = 1;
         break;
      }

      if ((elapsed % 100) == 0) {
         printf("  move-absolute PP t=%d ms axis=%d/%s pos=%d target=%d error=%lld sw=0x%04x target_reached=%d setpoint_ack=%d\n",
                elapsed, axis_no, axis_name(axis_no), pos, target, error, statusword,
                (statusword & 0x0400) ? 1 : 0,
                (statusword & 0x1000) ? 1 : 0);
      }

      if (error <= MOVE_POSITION_TOLERANCE_COUNTS && (statusword & 0x0400)) {
         reached = 1;
         printf("move-absolute: PP target reached at t=%d ms pos=%d error=%lld sw=0x%04x\n",
                elapsed, pos, error, statusword);
         break;
      }
   }

   if (!reached && !fault) {
      printf("move-absolute: timeout/stop before PP target reached\n");
   }

   print_status_pdo(bus);
   {
      EmuAxis *axis = &bus->axis[axis_index];
      ec_slavet *slave = &bus->ctx.slavelist[axis->slave];
      end_pos = pdo_read_i32(slave, &axis->map.actual_position);
      printf("move-absolute: final axis=%d/%s start_pos=%d end_pos=%d delta=%d target=%d error=%lld reached=%d\n",
             axis_no, axis_name(axis_no), start_pos, end_pos, end_pos - start_pos,
             target, (long long)target - (long long)end_pos, reached);
   }
   disable_axes_pp(bus);
   print_status_pdo(bus);
   return reached && !fault;
}

static int move_user_absolute_axis(EthercatScan *bus, int axis_no, int32 user_target,
                                   int32 velocity, int timeout_ms)
{
   AxisZero zero[PLC_AXIS_COUNT];
   long long raw_target_ll;
   int32 raw_target;

   if (axis_no < 1 || axis_no > bus->axis_count) {
      printf("move-user-absolute: invalid axis %d, available axes: 1..%d\n",
             axis_no, bus->axis_count);
      return 0;
   }

   load_axis_zero(zero);
   if (!zero[axis_no - 1].valid) {
      printf("move-user-absolute: axis %d/%s has no zero. Run: set-zero %s\n",
             axis_no, axis_name(axis_no), axis_name(axis_no));
      return 0;
   }

   raw_target_ll = (long long)zero[axis_no - 1].zero_raw + (long long)user_target;
   if (raw_target_ll < (-2147483647LL - 1LL) || raw_target_ll > 2147483647LL) {
      printf("move-user-absolute: raw target out of int32 range zero_raw=%d user_target=%d raw=%lld\n",
             zero[axis_no - 1].zero_raw, user_target, raw_target_ll);
      return 0;
   }
   raw_target = (int32)raw_target_ll;

   printf("move-user-absolute: axis=%d/%s user_target=%d zero_raw=%d raw_target=%d\n",
          axis_no, axis_name(axis_no), user_target, zero[axis_no - 1].zero_raw, raw_target);
   return move_absolute_axis(bus, axis_no, raw_target, velocity, timeout_ms);
}

static int round_double_to_i32(double value, int32 *out)
{
   double rounded;

   if (value < -2147483648.0 || value > 2147483647.0) return 0;
   rounded = (value >= 0.0) ? value + 0.5 : value - 0.5;
   if (rounded < -2147483648.0 || rounded > 2147483647.0) return 0;
   *out = (int32)rounded;
   return 1;
}

static int move_plc_absolute_axis(EthercatScan *bus, int axis_no, double plc_target,
                                  int32 velocity, int timeout_ms)
{
   AxisScale scale[PLC_AXIS_COUNT];
   double user_target_d;
   int32 user_target;

   if (axis_no < 1 || axis_no > bus->axis_count) {
      printf("move-plc-absolute: invalid axis %d, available axes: 1..%d\n",
             axis_no, bus->axis_count);
      return 0;
   }

   load_axis_scale(scale);
   if (plc_target == 0.0) {
      user_target = 0;
      printf("move-plc-absolute: axis=%d/%s plc_target=0 user_target=0 (scale not needed)\n",
             axis_no, axis_name(axis_no));
      return move_user_absolute_axis(bus, axis_no, user_target, velocity, timeout_ms);
   }

   if (!scale[axis_no - 1].valid) {
      printf("move-plc-absolute: axis %d/%s has no scale. Run: set-scale %s COUNTS_PER_PLC_UNIT\n",
             axis_no, axis_name(axis_no), axis_name(axis_no));
      return 0;
   }

   user_target_d = plc_target * scale[axis_no - 1].counts_per_unit;
   if (!round_double_to_i32(user_target_d, &user_target)) {
      printf("move-plc-absolute: user target out of int32 range plc_target=%.12g scale=%.12g user=%.12g\n",
             plc_target, scale[axis_no - 1].counts_per_unit, user_target_d);
      return 0;
   }

   printf("move-plc-absolute: axis=%d/%s plc_target=%.12g counts_per_unit=%.12g user_target=%d\n",
          axis_no, axis_name(axis_no), plc_target,
          scale[axis_no - 1].counts_per_unit, user_target);
   return move_user_absolute_axis(bus, axis_no, user_target, velocity, timeout_ms);
}

static int move_plc_slot_axis(EthercatScan *bus, int axis_no, const char *slot,
                              int32 velocity, int timeout_ms)
{
   double plc_target;

   if (!plc_position_lookup(axis_no, slot, &plc_target)) {
      printf("move-plc-slot: unknown PLC slot '%s' for axis %d/%s\n",
             slot, axis_no, axis_name(axis_no));
      print_plc_slots_for_axis(axis_no);
      return 0;
   }

   printf("move-plc-slot: axis=%d/%s slot=%s plc_target=%.12g\n",
          axis_no, axis_name(axis_no), slot, plc_target);
   return move_plc_absolute_axis(bus, axis_no, plc_target, velocity, timeout_ms);
}

static int preflight_plc_group_slot(EthercatScan *bus, const char *slot)
{
   AxisZero zero[PLC_AXIS_COUNT];
   AxisScale scale[PLC_AXIS_COUNT];
   int matched = 0;
   int ok = 1;

   load_axis_zero(zero);
   load_axis_scale(scale);

   printf("move-plc-group: preflight slot=%s\n", slot);
   for (int i = 0; i < bus->axis_count; i++) {
      int axis_no = i + 1;
      double plc_target;
      double user_target_d;
      int32 user_target;

      if (!plc_position_lookup(axis_no, slot, &plc_target)) continue;
      matched = 1;
      printf("  axis %d/%s plc_target=%.12g\n",
             axis_no, axis_name(axis_no), plc_target);

      if (!zero[i].valid) {
         printf("    missing zero: run set-zero %s first\n", axis_name(axis_no));
         ok = 0;
         continue;
      }
      if (plc_target == 0.0) {
         user_target = 0;
         printf("    zero_raw=%d user_target=0 (scale not needed)\n",
                zero[i].zero_raw);
         continue;
      }

      if (!scale[i].valid) {
         printf("    missing scale: run set-scale %s COUNTS_PER_PLC_UNIT first\n",
                axis_name(axis_no));
         ok = 0;
         continue;
      }

      user_target_d = plc_target * scale[i].counts_per_unit;
      if (!round_double_to_i32(user_target_d, &user_target)) {
         printf("    user target out of range plc_target=%.12g scale=%.12g user=%.12g\n",
                plc_target, scale[i].counts_per_unit, user_target_d);
         ok = 0;
         continue;
      }
      printf("    zero_raw=%d scale=%.12g user_target=%d\n",
             zero[i].zero_raw, scale[i].counts_per_unit, user_target);
   }

   if (!matched) {
      printf("move-plc-group: no connected axis has PLC slot '%s'\n", slot);
      printf("move-plc-group: run plc-table to see available slots\n");
      return 0;
   }

   return ok;
}

static int move_plc_group_slot(EthercatScan *bus, const char *slot,
                               int32 velocity, int timeout_ms)
{
   int moved = 0;

   printf("move-plc-group: sequential move slot=%s velocity=%d timeout=%d ms\n",
          slot, velocity, timeout_ms);
   for (int i = 0; i < bus->axis_count; i++) {
      int axis_no = i + 1;
      double plc_target;

      if (!plc_position_lookup(axis_no, slot, &plc_target)) continue;
      printf("move-plc-group: moving axis %d/%s\n", axis_no, axis_name(axis_no));
      if (!move_plc_slot_axis(bus, axis_no, slot, velocity, timeout_ms)) {
         printf("move-plc-group: failed on axis %d/%s, stop sequence\n",
                axis_no, axis_name(axis_no));
         return 0;
      }
      moved++;
   }

   printf("move-plc-group: completed slot=%s axes_moved=%d\n", slot, moved);
   return moved > 0;
}

static void print_command(const uint8_t *buf, size_t len, const PlcCommand *cmd)
{
   printf("tcp rx: ");
   plc_protocol_dump_frame(buf, len);
   printf(" -> %s raw0=0x%04x raw1=0x%04x raw2=0x%04x raw3=0x%04x value=%d axis=%d\n",
          plc_command_name(cmd->type),
          cmd->raw0, cmd->raw1, cmd->raw2, cmd->raw3,
          cmd->value, cmd->axis);
}

static void handle_rx(PlcState *state, const uint8_t *buf, size_t len)
{
   PlcCommand cmd;
   PlcParseResult ret;

   ret = plc_protocol_parse(buf, len, &cmd);
   if (ret == PLC_PARSE_INCOMPLETE) {
      printf("tcp rx: incomplete frame len=%zu\n", len);
      state->parse_error_count++;
      return;
   }
   if (ret == PLC_PARSE_BAD_FRAME) {
      printf("tcp rx: bad frame len=%zu: ", len);
      plc_protocol_dump_frame(buf, len);
      printf("\n");
      state->parse_error_count++;
      return;
   }

   state->pc_rx_count++;
   clock_gettime(CLOCK_MONOTONIC, &state->last_rx_time);
   plc_protocol_apply_command(state, &cmd);
   print_command(buf, len, &cmd);
}

static void tcp_service_loop(PlcState *state, const char *host, int port, int status_ms)
{
   PlcTcpClient tcp;
   uint8_t rx[256];
   uint8_t tx[PLC_FRAME_STATUS_BYTES];
   struct timespec last_tx;

   plc_tcp_init(&tcp, host, port);
   clock_gettime(CLOCK_MONOTONIC, &last_tx);

   while (g_running) {
      struct timespec now;
      int n;

      if (!tcp.connected) {
         state->connected_to_pc = 0;
         if (!plc_tcp_connect(&tcp)) {
            sleep(1);
            continue;
         }
         state->connected_to_pc = 1;
      }

      n = plc_tcp_recv(&tcp, rx, sizeof(rx), 20);
      if (n > 0) handle_rx(state, rx, (size_t)n);
      else if (n < 0) {
         state->connected_to_pc = 0;
         continue;
      }

      clock_gettime(CLOCK_MONOTONIC, &now);
      if (elapsed_ms(&last_tx, &now) >= status_ms) {
         size_t out_len = plc_protocol_build_status(state, tx, sizeof(tx));
         if (out_len > 0 && plc_tcp_send_all(&tcp, tx, out_len)) {
            state->pc_tx_count++;
         }
         last_tx = now;
      }
   }

   plc_tcp_close(&tcp);
}

static void local_idle_loop(PlcState *state, const EthercatScan *bus)
{
   int tick = 0;

   printf("plc-emulator: local Orin controller mode\n");
   printf("plc-emulator: no old IPC TCP connection will be opened\n");
   printf("plc-emulator: EtherCAT slaves=%d. Press Ctrl-C to exit.\n", bus->slave_count);

   while (g_running) {
      sleep(DEFAULT_IDLE_PERIOD_MS / 1000);
      tick++;
      if ((tick % 5) == 0) {
         printf("plc-emulator: idle, slaves=%d rx=%d tx=%d enable_req=%d last_cmd=%s\n",
                bus->slave_count,
                state->pc_rx_count,
                state->pc_tx_count,
                state->enable_request,
                plc_command_name(state->last_cmd.type));
      }
   }
}

int main(int argc, char **argv)
{
   const char *ifname;
   const char *pc_host = DEFAULT_PC_HOST;
   int pc_port = DEFAULT_PC_PORT;
   int status_ms = DEFAULT_STATUS_PERIOD_MS;
   int jog_axis = 0;
   int jog_duration_ms = 0;
   int32 jog_velocity = 0;
   int move_axis = 0;
   int move_timeout_ms = 0;
   int32 move_delta = 0;
   int32 move_velocity = 0;
   int abs_axis = 0;
   int abs_timeout_ms = 0;
   int32 abs_target = 0;
   int32 abs_velocity = 0;
   int axis_status_axis = 0;
   int set_zero_axis = 0;
   int32 set_zero_user_position = 0;
   int user_abs_axis = 0;
   int user_abs_timeout_ms = 0;
   int32 user_abs_target = 0;
   int32 user_abs_velocity = 0;
   int scale_status_axis = 0;
   int set_scale_axis = 0;
   double set_scale_counts_per_unit = 0.0;
   int calibrate_scale_axis = 0;
   double calibrate_scale_known_position = 0.0;
   int plc_abs_axis = 0;
   double plc_abs_target = 0.0;
   int32 plc_abs_velocity = 0;
   int plc_abs_timeout_ms = 0;
   int plc_slot_axis = 0;
   const char *plc_slot = NULL;
   int32 plc_slot_velocity = 0;
   int plc_slot_timeout_ms = 0;
   const char *plc_group_slot = NULL;
   int32 plc_group_velocity = 0;
   int plc_group_timeout_ms = 0;
   AppMode mode = MODE_IDLE;
   EthercatScan bus;
   PlcState state;

   if (argc < 2) {
      usage(argv[0]);
      return 1;
   }

   ifname = argv[1];
   if (argc >= 3) {
      if (strcmp(argv[2], "status") == 0) {
         mode = MODE_STATUS;
      } else if (strcmp(argv[2], "axes") == 0) {
         mode = MODE_AXES;
      } else if (strcmp(argv[2], "op-status") == 0) {
         mode = MODE_OP_STATUS;
      } else if (strcmp(argv[2], "fault-reset") == 0) {
         mode = MODE_FAULT_RESET;
      } else if (strcmp(argv[2], "enable-dryrun") == 0) {
         mode = MODE_ENABLE_DRYRUN;
      } else if (strcmp(argv[2], "axis-status") == 0) {
         mode = MODE_AXIS_STATUS;
         if (argc >= 4 && !parse_axis_name(argv[3], &axis_status_axis)) {
            printf("axis-status: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
      } else if (strcmp(argv[2], "set-zero") == 0) {
         if (argc < 4) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_SET_ZERO;
         if (!parse_axis_name(argv[3], &set_zero_axis)) {
            printf("set-zero: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         if (argc >= 5 && !parse_i32_arg(argv[4], &set_zero_user_position)) {
            usage(argv[0]);
            return 1;
         }
      } else if (strcmp(argv[2], "scale-status") == 0) {
         mode = MODE_SCALE_STATUS;
         if (argc >= 4 && !parse_axis_name(argv[3], &scale_status_axis)) {
            printf("scale-status: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
      } else if (strcmp(argv[2], "set-scale") == 0) {
         if (argc < 5) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_SET_SCALE;
         if (!parse_axis_name(argv[3], &set_scale_axis)) {
            printf("set-scale: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         if (!parse_double_arg(argv[4], &set_scale_counts_per_unit)) {
            usage(argv[0]);
            return 1;
         }
      } else if (strcmp(argv[2], "calibrate-scale") == 0) {
         if (argc < 5) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_CALIBRATE_SCALE;
         if (!parse_axis_name(argv[3], &calibrate_scale_axis)) {
            printf("calibrate-scale: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         if (!parse_double_arg(argv[4], &calibrate_scale_known_position)) {
            usage(argv[0]);
            return 1;
         }
      } else if (strcmp(argv[2], "jog-velocity") == 0) {
         char *endp;
         long value;

         if (argc < 6) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_JOG_VELOCITY;
         if (!parse_axis_name(argv[3], &jog_axis)) {
            printf("jog: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         value = strtol(argv[4], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         jog_velocity = (int32)value;
         value = strtol(argv[5], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         jog_duration_ms = (int)value;
      } else if (strcmp(argv[2], "move-relative") == 0) {
         char *endp;
         long value;

         if (argc < 7) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_MOVE_RELATIVE;
         if (!parse_axis_name(argv[3], &move_axis)) {
            printf("move-relative: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         value = strtol(argv[4], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         move_delta = (int32)value;
         value = strtol(argv[5], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         move_velocity = (int32)value;
         value = strtol(argv[6], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         move_timeout_ms = (int)value;
      } else if (strcmp(argv[2], "move-absolute") == 0) {
         char *endp;
         long value;

         if (argc < 7) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_MOVE_ABSOLUTE;
         if (!parse_axis_name(argv[3], &abs_axis)) {
            printf("move-absolute: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         value = strtol(argv[4], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         abs_target = (int32)value;
         value = strtol(argv[5], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         abs_velocity = (int32)value;
         value = strtol(argv[6], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         abs_timeout_ms = (int)value;
      } else if (strcmp(argv[2], "move-user-absolute") == 0) {
         char *endp;
         long value;

         if (argc < 7) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_MOVE_USER_ABSOLUTE;
         if (!parse_axis_name(argv[3], &user_abs_axis)) {
            printf("move-user-absolute: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         value = strtol(argv[4], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         user_abs_target = (int32)value;
         value = strtol(argv[5], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         user_abs_velocity = (int32)value;
         value = strtol(argv[6], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         user_abs_timeout_ms = (int)value;
      } else if (strcmp(argv[2], "move-plc-absolute") == 0) {
         char *endp;
         long value;

         if (argc < 7) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_MOVE_PLC_ABSOLUTE;
         if (!parse_axis_name(argv[3], &plc_abs_axis)) {
            printf("move-plc-absolute: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         if (!parse_double_arg(argv[4], &plc_abs_target)) {
            usage(argv[0]);
            return 1;
         }
         value = strtol(argv[5], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         plc_abs_velocity = (int32)value;
         value = strtol(argv[6], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         plc_abs_timeout_ms = (int)value;
      } else if (strcmp(argv[2], "move-plc-slot") == 0) {
         char *endp;
         long value;

         if (argc < 7) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_MOVE_PLC_SLOT;
         if (!parse_axis_name(argv[3], &plc_slot_axis)) {
            printf("move-plc-slot: unknown axis '%s'\n", argv[3]);
            print_axis_aliases();
            return 1;
         }
         plc_slot = argv[4];
         value = strtol(argv[5], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         plc_slot_velocity = (int32)value;
         value = strtol(argv[6], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         plc_slot_timeout_ms = (int)value;
      } else if (strcmp(argv[2], "move-plc-group") == 0) {
         char *endp;
         long value;

         if (argc < 6) {
            usage(argv[0]);
            return 1;
         }
         mode = MODE_MOVE_PLC_GROUP;
         plc_group_slot = argv[3];
         value = strtol(argv[4], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         plc_group_velocity = (int32)value;
         value = strtol(argv[5], &endp, 0);
         if (*endp) { usage(argv[0]); return 1; }
         plc_group_timeout_ms = (int)value;
      } else if (strcmp(argv[2], "plc-table") == 0) {
         mode = MODE_PLC_TABLE;
      } else if (strcmp(argv[2], "--tcp") == 0) {
         mode = MODE_TCP;
         if (argc >= 4) pc_host = argv[3];
         if (argc >= 5) pc_port = atoi(argv[4]);
         if (argc >= 6) status_ms = atoi(argv[5]);
      } else {
         usage(argv[0]);
         return 1;
      }
   }
   if (status_ms <= 0) status_ms = DEFAULT_STATUS_PERIOD_MS;

   if (mode == MODE_JOG_VELOCITY &&
       !validate_jog_limits(jog_axis, jog_velocity, jog_duration_ms)) {
      return 1;
   }
   if (mode == MODE_MOVE_RELATIVE &&
       !validate_move_limits(move_axis, move_delta, move_velocity, move_timeout_ms)) {
      return 1;
   }
   if (mode == MODE_MOVE_ABSOLUTE &&
       !validate_absolute_move_limits(abs_axis, abs_target, abs_velocity, abs_timeout_ms)) {
      return 1;
   }
   if (mode == MODE_MOVE_USER_ABSOLUTE &&
       !validate_absolute_move_limits(user_abs_axis, user_abs_target,
                                      user_abs_velocity, user_abs_timeout_ms)) {
      return 1;
   }
   if (mode == MODE_MOVE_PLC_ABSOLUTE &&
       !validate_absolute_move_limits(plc_abs_axis, 0,
                                      plc_abs_velocity, plc_abs_timeout_ms)) {
      return 1;
   }
   if (mode == MODE_MOVE_PLC_SLOT &&
       !validate_absolute_move_limits(plc_slot_axis, 0,
                                      plc_slot_velocity, plc_slot_timeout_ms)) {
      return 1;
   }
   if (mode == MODE_MOVE_PLC_GROUP &&
       !validate_absolute_move_limits(1, 0,
                                      plc_group_velocity, plc_group_timeout_ms)) {
      return 1;
   }
   if (mode == MODE_PLC_TABLE) {
      print_plc_position_table();
      return 0;
   }
   if (mode == MODE_SCALE_STATUS) {
      print_axis_scale_status(scale_status_axis);
      return 0;
   }
   if (mode == MODE_SET_SCALE) {
      return set_axis_scale(set_scale_axis, set_scale_counts_per_unit) ? 0 : 1;
   }

   signal(SIGINT, on_signal);
   signal(SIGTERM, on_signal);

   plc_state_init(&state);

   if (!ethercat_scan(&bus, ifname)) {
      ethercat_close(&bus);
      printf("ethercat: scan failed; exit\n");
      return 1;
   }

   if (mode == MODE_AXES) {
      print_axis_map(&bus);
   } else if (mode == MODE_AXIS_STATUS) {
      axis_status_user(&bus, axis_status_axis);
   } else if (mode == MODE_SET_ZERO) {
      if (!set_axis_zero(&bus, set_zero_axis, set_zero_user_position)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_CALIBRATE_SCALE) {
      if (!calibrate_axis_scale(&bus, calibrate_scale_axis,
                                calibrate_scale_known_position)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_STATUS) {
      print_status_sdo(&bus);
   } else if (mode == MODE_OP_STATUS) {
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
   } else if (mode == MODE_FAULT_RESET) {
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      fault_reset_axes(&bus);
      print_status_pdo(&bus);
   } else if (mode == MODE_ENABLE_DRYRUN) {
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      enable_dryrun_axes(&bus);
   } else if (mode == MODE_JOG_VELOCITY) {
      if (!configure_jog_profile(&bus)) {
         ethercat_close(&bus);
         return 1;
      }
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      if (!jog_velocity_axis(&bus, jog_axis, jog_velocity, jog_duration_ms)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_MOVE_RELATIVE) {
      if (!configure_jog_profile(&bus)) {
         ethercat_close(&bus);
         return 1;
      }
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      if (!move_relative_axis(&bus, move_axis, move_delta, move_velocity, move_timeout_ms)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_MOVE_ABSOLUTE) {
      if (!configure_pp_profile(&bus, abs_velocity)) {
         ethercat_close(&bus);
         return 1;
      }
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      if (!move_absolute_axis(&bus, abs_axis, abs_target, abs_velocity, abs_timeout_ms)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_MOVE_USER_ABSOLUTE) {
      if (!configure_pp_profile(&bus, user_abs_velocity)) {
         ethercat_close(&bus);
         return 1;
      }
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      if (!move_user_absolute_axis(&bus, user_abs_axis, user_abs_target,
                                   user_abs_velocity, user_abs_timeout_ms)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_MOVE_PLC_ABSOLUTE) {
      if (!configure_pp_profile(&bus, plc_abs_velocity)) {
         ethercat_close(&bus);
         return 1;
      }
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      if (!move_plc_absolute_axis(&bus, plc_abs_axis, plc_abs_target,
                                  plc_abs_velocity, plc_abs_timeout_ms)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_MOVE_PLC_SLOT) {
      if (!configure_pp_profile(&bus, plc_slot_velocity)) {
         ethercat_close(&bus);
         return 1;
      }
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      if (!move_plc_slot_axis(&bus, plc_slot_axis, plc_slot,
                              plc_slot_velocity, plc_slot_timeout_ms)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_MOVE_PLC_GROUP) {
      if (!preflight_plc_group_slot(&bus, plc_group_slot)) {
         ethercat_close(&bus);
         return 1;
      }
      if (!configure_pp_profile(&bus, plc_group_velocity)) {
         ethercat_close(&bus);
         return 1;
      }
      if (!ethercat_enter_op(&bus)) {
         print_status_sdo(&bus);
         ethercat_close(&bus);
         return 1;
      }
      print_status_pdo(&bus);
      if (!move_plc_group_slot(&bus, plc_group_slot,
                               plc_group_velocity, plc_group_timeout_ms)) {
         ethercat_close(&bus);
         return 1;
      }
   } else if (mode == MODE_TCP) {
      printf("plc-emulator: old IPC compatibility mode enabled\n");
      printf("plc-emulator: TCP target %s:%d, status period %d ms\n",
             pc_host, pc_port, status_ms);
      printf("plc-emulator: waiting for IPC commands. Press Ctrl-C to exit.\n");
      tcp_service_loop(&state, pc_host, pc_port, status_ms);
   } else {
      local_idle_loop(&state, &bus);
   }

   printf("plc-emulator: rx=%d tx=%d parse_errors=%d unknown=%d\n",
          state.pc_rx_count, state.pc_tx_count,
          state.parse_error_count, state.unknown_cmd_count);
   ethercat_close(&bus);
   return 0;
}
