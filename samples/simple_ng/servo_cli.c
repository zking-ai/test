/** \file
 * \brief 简单 EtherCAT 伺服速度控制程序（PV 模式）。
 *
 * 加减速由驱动器内部根据轮廓加速度（0x6083）
 * 和轮廓减速度（0x6084）完成。
 * 主站只负责写目标速度（0x60FF）。
 *
 * 使用方法：
 *   servo_cli IFACE enable [cycle_us]       使能伺服，并等待输入速度
 *   servo_cli IFACE status [cycle_us]       查看伺服诊断信息
 *   servo_cli IFACE fault-reset [cycle_us]  复位伺服故障
 *
 * 操作流程：
 *   1. 执行 "enable"：初始化总线，进入 OP，完成 CiA402 使能，等待输入
 *   2. 输入速度值（例如 50000）：电机按该速度运行
 *   3. 运行过程中按 Ctrl-C：停止当前运动，保持 OP 状态，回到命令行
 *   4. 命令行下按 Ctrl-C 或输入 'q'：执行安全退出
 */

#include "soem/soem.h"
#include <errno.h>
#include <ctype.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <unistd.h>

#define SERVO_AXES        2
#define FIRST_SERVO_SLAVE 1
#define DEFAULT_CYCLE_US  1000
#define IO_MAP_SIZE       4096

#define MODE_PV   3
#define MODE_CSV  9

#define MAX_VELOCITY  6000000
#define ACCEL_DEFAULT 9000000
#define DECEL_DEFAULT 8000000

#define CYCLIC_RT_PRIORITY 90

/* ---- 信号处理相关的全局状态 ---- */
static volatile sig_atomic_t g_running    = 1;  /* 主程序运行标志 */
static volatile sig_atomic_t g_in_motion  = 0;  /* 电机运动中标志 */
static volatile sig_atomic_t g_stop_motion = 0; /* 请求停止当前运动 */

static void on_signal(int sig)
{
   (void)sig;
   if (g_in_motion) g_stop_motion = 1;
   else             g_running = 0;
}

/* ---- PDO 映射结构 ---- */
typedef struct {
   int present, bit_offset, bit_length;
} PdoEntry;

typedef struct {
   PdoEntry controlword, statusword, error_code;
   PdoEntry modes_of_operation, mode_display;
   PdoEntry target_velocity, target_position, actual_position;
   PdoEntry digital_outputs, max_profile_velocity;
} ServoMap;

typedef struct {
   uint16          slave;
   ServoMap        map;
   int             enabled;
   int32           target_velocity;
} ServoAxis;

typedef struct {
   ecx_contextt    ctx;
   uint8           iomap[IO_MAP_SIZE];
   ServoAxis       axis[SERVO_AXES];
   int             axis_count;
   pthread_t       thread;
   volatile int    thread_run;
   int             expected_wkc, wkc, cycle, bad_wkc;
   uint32          cycle_us, cycle_ns;
} ServoBus;

static ServoBus *g_pdo_bus;

static void lock_process_memory(void)
{
   if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
      printf("Warning: mlockall failed: %s\n", strerror(errno));
}

static void set_thread_fifo(pthread_t thread, int priority, const char *name)
{
   struct sched_param sp;
   memset(&sp, 0, sizeof(sp));
   sp.sched_priority = priority;
   int ret = pthread_setschedparam(thread, SCHED_FIFO, &sp);
   if (ret != 0)
      printf("Warning: set %s SCHED_FIFO/%d failed: %s\n", name, priority, strerror(ret));
}

/* ---- 基础辅助函数 ---- */
static int roundtrip(ServoBus *b)
   { ecx_send_processdata(&b->ctx); return ecx_receive_processdata(&b->ctx, EC_TIMEOUTRET); }

static void sleep_cyc(ServoBus *b, int n) {
   for (int i = 0; i < n; i++) { if (!b->thread_run) roundtrip(b); osal_usleep(b->cycle_us); }
}

static void drain_errs(ecx_contextt *c) { while (c->ecaterror) ecx_elist2string(c); }

/* ---- PDO 读写接口 ---- */
static uint8 *e_ptr(ec_slavet *s, const PdoEntry *e, int out) {
   if (!e->present || (e->bit_offset & 7)) return NULL;
   return (out ? s->outputs : s->inputs) + e->bit_offset / 8;
}
static void pdo_w16(ec_slavet *s, const PdoEntry *e, uint16 v) {
   uint8 *p = e_ptr(s, e, 1); if (p && e->bit_length == 16) { uint16 x = htoes(v); memcpy(p, &x, 2); }
}
static void pdo_w32(ec_slavet *s, const PdoEntry *e, int32 v) {
   uint8 *p = e_ptr(s, e, 1); if (p && e->bit_length == 32) { int32 x = htoel(v); memcpy(p, &x, 4); }
}
static void pdo_w8(ec_slavet *s, const PdoEntry *e, int8 v) {
   uint8 *p = e_ptr(s, e, 1); if (p && e->bit_length == 8) memcpy(p, &v, 1);
}
static uint16 pdo_r16(ec_slavet *s, const PdoEntry *e) {
   uint8 *p = e_ptr(s, e, 0); uint16 v = 0;
   if (p && e->bit_length == 16) { memcpy(&v, p, 2); v = etohs(v); } return v;
}
static int32 pdo_r32(ec_slavet *s, const PdoEntry *e) {
   uint8 *p = e_ptr(s, e, 0); int32 v = 0;
   if (p && e->bit_length == 32) { memcpy(&v, p, 4); v = etohl(v); } return v;
}

/* ---- SDO 读写接口 ---- */
static int sdo_r8 (ecx_contextt *c, uint16 slave, uint16 i, uint8 su, uint8 *v)
   { int sz=1; return ecx_SDOread(c,slave,i,su,FALSE,&sz,v,EC_TIMEOUTRXM)>0; }
static int sdo_r16(ecx_contextt *c, uint16 slave, uint16 i, uint8 su, uint16 *v)
   { int sz=2; if(ecx_SDOread(c,slave,i,su,FALSE,&sz,v,EC_TIMEOUTRXM)<=0)return 0; *v=etohs(*v); return 1; }
static int sdo_r32(ecx_contextt *c, uint16 slave, uint16 i, uint8 su, uint32 *v)
   { int sz=4; if(ecx_SDOread(c,slave,i,su,FALSE,&sz,v,EC_TIMEOUTRXM)<=0)return 0; *v=etohl(*v); return 1; }
static int sdo_w8 (ecx_contextt *c, uint16 slave, uint16 i, uint8 su, uint8 v)
   { return ecx_SDOwrite(c,slave,i,su,FALSE,1,&v,EC_TIMEOUTRXM)>0; }
static int sdo_w16(ecx_contextt *c, uint16 slave, uint16 i, uint8 su, uint16 v)
   { uint16 x=htoes(v); return ecx_SDOwrite(c,slave,i,su,FALSE,2,&x,EC_TIMEOUTRXM)>0; }
static int sdo_w32(ecx_contextt *c, uint16 slave, uint16 i, uint8 su, uint32 v)
   { uint32 x=htoel(v); return ecx_SDOwrite(c,slave,i,su,FALSE,4,&x,EC_TIMEOUTRXM)>0; }

/* ---- CiA402 状态辅助函数 ---- */
static const char *cia402_str(uint16 sw) {
   if (sw & 0x0008) return "FAULT";
   if ((sw & 0x004f) == 0x0040) return "switch-on-disabled";
   if ((sw & 0x006f) == 0x0021) return "ready-to-switch-on";
   if ((sw & 0x006f) == 0x0023) return "switched-on";
   if ((sw & 0x006f) == 0x0027) return "operation-enabled";
   return "?";
}
static int is_oe(uint16 sw) { return (sw & 0x006f) == 0x0027; }
static int is_rts(uint16 sw) { return (sw & 0x006f) == 0x0021; }
static int is_so(uint16 sw) { return (sw & 0x006f) == 0x0023; }

/* ---- PDO 映射解析 ---- */
static void remember(ServoMap *m, uint16 idx, uint8 sub, int off, int len) {
   if (sub != 0) return;
   PdoEntry *e = NULL;
   switch (idx) {
   case 0x603f: e = &m->error_code; break;
   case 0x6040: e = &m->controlword; break;
   case 0x6041: e = &m->statusword; break;
   case 0x6060: e = &m->modes_of_operation; break;
   case 0x6061: e = &m->mode_display; break;
   case 0x60ff: e = &m->target_velocity; break;
   case 0x607a: e = &m->target_position; break;
   case 0x6064: e = &m->actual_position; break;
   case 0x607f: e = &m->max_profile_velocity; break;
   case 0x60fe: e = &m->digital_outputs; break;
   }
   if (e) { e->present = 1; e->bit_offset = off; e->bit_length = len; }
}

static int read_map(ecx_contextt *c, uint16 slave, ServoMap *m, uint16 assign, int boff) {
   uint8 pc = 0, ec = 0; uint16 pi = 0; uint32 mp = 0; int sz;
   sz = 1; if (ecx_SDOread(c, slave, assign, 0x00, FALSE, &sz, &pc, EC_TIMEOUTRXM) <= 0) return boff;
   for (int i = 1; i <= pc; i++) {
      sz = 2; pi = 0;
      if (ecx_SDOread(c, slave, assign, (uint8)i, FALSE, &sz, &pi, EC_TIMEOUTRXM) <= 0) return boff;
      pi = etohs(pi);
      sz = 1; ec = 0;
      if (ecx_SDOread(c, slave, pi, 0x00, FALSE, &sz, &ec, EC_TIMEOUTRXM) <= 0) return boff;
      for (int j = 1; j <= ec; j++) {
         sz = 4; mp = 0;
         if (ecx_SDOread(c, slave, pi, (uint8)j, FALSE, &sz, &mp, EC_TIMEOUTRXM) <= 0) return boff;
         mp = etohl(mp);
         uint16 idx = (uint16)(mp >> 16);
         uint8  sub = (uint8)((mp >> 8) & 0xff);
         uint8  blen = (uint8)(mp & 0xff);
         printf("  %04x:%02x bit:%d len:%u\n", idx, sub, boff, blen);
         remember(m, idx, sub, boff, blen);
         boff += blen;
      }
   }
   return boff;
}

/* ---- 扫描固定 RxPDO，查找同时包含 0x6040 和 0x60FF 的 PDO ---- */
static int find_and_assign_pv_pdo(ecx_contextt *c, uint16 slave)
{
   int sz;
   printf("Slave %u: scanning fixed RxPDOs for 0x6040+0x60FF...\n", slave);
   for (uint16 idx = 0x1600; idx <= 0x17ff; idx++) {
      uint8 cnt = 0; sz = 1;
      if (ecx_SDOread(c, slave, idx, 0x00, FALSE, &sz, &cnt, EC_TIMEOUTRXM) <= 0) {
         drain_errs(c); continue;
      }
      int has_6040 = 0, has_60ff = 0;
      for (int j = 1; j <= cnt && !(has_6040 && has_60ff); j++) {
         uint32 m = 0; sz = 4;
         if (ecx_SDOread(c, slave, idx, (uint8)j, FALSE, &sz, &m, EC_TIMEOUTRXM) > 0) {
            m = etohl(m);
            if ((m >> 16) == 0x6040) has_6040 = 1;
            if ((m >> 16) == 0x60ff) has_60ff = 1;
         }
      }
      if (has_6040 && has_60ff) {
         printf("  Found PV-compatible PDO 0x%04x\n", idx);
         if (!sdo_w8(c, slave, 0x1c12, 0x00, 0)) { drain_errs(c); return 0; }
         if (!sdo_w16(c, slave, 0x1c12, 0x01, idx)) { drain_errs(c); return 0; }
         if (!sdo_w8(c, slave, 0x1c12, 0x00, 1)) { drain_errs(c); return 0; }
         return 1;
      }
      drain_errs(c);
   }
   printf("  No fixed PDO with 0x60FF found 鈥?drive may need manual PDO config\n");
   return 0;
}

static int pv_pdo_cb(ecx_contextt *c, uint16 slave)
{
   if (!g_pdo_bus) return 0;
   for (int i = 0; i < g_pdo_bus->axis_count; i++)
      if (g_pdo_bus->axis[i].slave == slave)
         return find_and_assign_pv_pdo(c, slave);
   return 0;
}

/* ---- EtherCAT 周期通信线程 ---- */
static void add_ns(ec_timet *t, int64 ns) {
   ec_timet a; a.tv_nsec = ns % 1000000000; a.tv_sec = (ns - a.tv_nsec) / 1000000000;
   osal_timespecadd(t, &a, t);
}
static OSAL_THREAD_FUNC_RT cyclic_thread(void *arg) {
   ServoBus *b = arg; ec_timet ts; int ht;
   osal_get_monotonic_time(&ts);
   ht = (ts.tv_nsec / 1000000) + 1; ts.tv_nsec = ht * 1000000;
   ecx_send_processdata(&b->ctx);
   while (b->thread_run) {
      add_ns(&ts, b->cycle_ns); osal_monotonic_sleep(&ts);
      b->wkc = ecx_receive_processdata(&b->ctx, EC_TIMEOUTRET);
      if (b->wkc != b->expected_wkc) b->bad_wkc++;
      ecx_send_processdata(&b->ctx);
      b->cycle++;
   }
}
static int start_thread(ServoBus *b) {
   b->thread_run = 1;
   if (!osal_thread_create_rt(&b->thread, 128000, &cyclic_thread, b)) {
      printf("Warning: RT thread failed, using normal priority\n");
      if (!osal_thread_create(&b->thread, 128000, &cyclic_thread, b)) { b->thread_run = 0; return 0; }
   }
   set_thread_fifo(b->thread, CYCLIC_RT_PRIORITY, "cyclic thread");
   return 1;
}
static void stop_thread(ServoBus *b) {
   if (!b->thread_run) return;
   b->thread_run = 0; pthread_join(b->thread, NULL);
}

static ec_slavet *axis_slave(ServoBus *b, ServoAxis *a)
{
   return &b->ctx.slavelist[a->slave];
}

static int axis_ready(ServoAxis *a)
{
   return a->map.controlword.present &&
          a->map.statusword.present &&
          a->map.target_velocity.present;
}

static void axis_write_idle(ServoBus *b, ServoAxis *a, uint16 controlword)
{
   ec_slavet *s = axis_slave(b, a);

   pdo_w32(s, &a->map.target_velocity, 0);
   pdo_w16(s, &a->map.controlword, controlword);
}

static uint16 axis_output_controlword(ServoBus *b, ServoAxis *a)
{
   ec_slavet *s = axis_slave(b, a);
   uint8 *p = e_ptr(s, &a->map.controlword, 1);
   uint16 v = 0;

   if (p && a->map.controlword.bit_length == 16) {
      memcpy(&v, p, 2);
      v = etohs(v);
   }
   return v;
}

static int32 axis_output_target_velocity(ServoBus *b, ServoAxis *a)
{
   ec_slavet *s = axis_slave(b, a);
   uint8 *p = e_ptr(s, &a->map.target_velocity, 1);
   int32 v = 0;

   if (p && a->map.target_velocity.bit_length == 32) {
      memcpy(&v, p, 4);
      v = etohl(v);
   }
   return v;
}

/* ---- EtherCAT 总线初始化 ---- */
static int bus_init(ServoBus *b, const char *iface, uint32 cyc_us)
{
   ecx_contextt *c = &b->ctx;
   memset(b, 0, sizeof(*b));
   b->cycle_us = cyc_us ? cyc_us : DEFAULT_CYCLE_US;
   b->cycle_ns = b->cycle_us * 1000;
   b->axis_count = SERVO_AXES;
   for (int i = 0; i < b->axis_count; i++)
      b->axis[i].slave = FIRST_SERVO_SLAVE + i;

   printf("Opening %s, cycle=%u us\n", iface, b->cycle_us);
   if (!ecx_init(c, iface)) { printf("No socket\n"); return 0; }
   if (ecx_config_init(c) <= 0) { printf("No slaves\n"); return 0; }
   if (c->slavecount < b->axis_count) { printf("Need %d servo slaves, found %d\n", b->axis_count, c->slavecount); return 0; }

   c->manualstatechange = 1;
   g_pdo_bus = b;
   for (int i = 0; i < b->axis_count; i++)
      c->slavelist[b->axis[i].slave].PO2SOconfig = pv_pdo_cb;
   ecx_config_map_group(c, b->iomap, 0);

   for (int i = 0; i < b->axis_count; i++) {
      ServoAxis *a = &b->axis[i];
      printf("Axis %d (slave %u) RxPDO:\n", i + 1, a->slave);
      read_map(c, a->slave, &a->map, 0x1c12, 0);
      printf("Axis %d (slave %u) TxPDO:\n", i + 1, a->slave);
      read_map(c, a->slave, &a->map, 0x1c13, 0);
      drain_errs(c);

      if (!axis_ready(a)) {
         printf("Axis %d: need 0x6040/0x6041/0x60FF PDO entries\n", i + 1);
         return 0;
      }
   }

   /* 配置 DC 同步模式 */
   for (int i = 0; i < b->axis_count; i++) {
      ServoAxis *a = &b->axis[i];
      uint16 sm = htoes(2); uint32 cy = htoel(b->cycle_ns);
      ecx_SDOwrite(c, a->slave, 0x1c32, 0x01, FALSE, 2, &sm, EC_TIMEOUTRXM);
      ecx_SDOwrite(c, a->slave, 0x1c33, 0x01, FALSE, 2, &sm, EC_TIMEOUTRXM);
      ecx_SDOwrite(c, a->slave, 0x1c32, 0x02, FALSE, 4, &cy, EC_TIMEOUTRXM);
      ecx_SDOwrite(c, a->slave, 0x1c33, 0x02, FALSE, 4, &cy, EC_TIMEOUTRXM);
   }
   drain_errs(c);

   ecx_configdc(c);
   for (int i = 1; i <= c->slavecount; i++)
      if (c->slavelist[i].hasdc) ecx_dcsync0(c, i, TRUE, b->cycle_ns, 0);

   /* 切换状态：SAFE_OP -> OP */
   c->slavelist[0].state = EC_STATE_SAFE_OP; ecx_writestate(c, 0);
   ecx_statecheck(c, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   if (c->slavelist[0].state != EC_STATE_SAFE_OP) { printf("FAIL SAFE_OP\n"); return 0; }
   sleep_cyc(b, 200);

   c->slavelist[0].state = EC_STATE_OPERATIONAL; ecx_writestate(c, 0);
   for (int i = 0; i < 500; i++) {
      roundtrip(b); ecx_statecheck(c, 0, EC_STATE_OPERATIONAL, b->cycle_us);
      if (c->slavelist[0].state == EC_STATE_OPERATIONAL) {
         b->expected_wkc = c->grouplist[0].outputsWKC * 2 + c->grouplist[0].inputsWKC;
         printf("OP OK, expected WKC=%d\n", b->expected_wkc);
         /* 预置安全输出，避免启动瞬间产生非预期动作 */
         for (int j = 0; j < b->axis_count; j++) {
            ServoAxis *a = &b->axis[j];
            ec_slavet *s = axis_slave(b, a);
            pdo_w16(s, &a->map.controlword, 0x0006);
            if (a->map.modes_of_operation.present)
               pdo_w8(s, &a->map.modes_of_operation, 0);
            pdo_w32(s, &a->map.target_velocity, 0);
         }
         roundtrip(b);
         return 1;
      }
      osal_usleep(b->cycle_us);
   }
   printf("FAIL OP\n"); return 0;
}

/* ---- CiA402 使能流程（PV 模式） ---- */
static int servo_enable_axis(ServoBus *b, ServoAxis *a)
{
   ecx_contextt *c = &b->ctx;
   ec_slavet *s = axis_slave(b, a);
   uint16 sw = pdo_r16(s, &a->map.statusword);
   printf("Axis slave %u: enable PV mode\n", a->slave);

   /* 如有故障，先执行故障复位 */
   if (sw & 0x0008) {
      printf("Fault (sw=0x%04x err=0x%04x), resetting...\n",
             sw, a->map.error_code.present ? pdo_r16(s, &a->map.error_code) : 0);
      for (int j = 0; j < 3; j++) {
         pdo_w16(s, &a->map.controlword, 0x0080); sleep_cyc(b, 100);
         pdo_w16(s, &a->map.controlword, 0x0000); sleep_cyc(b, 100);
         sw = pdo_r16(s, &a->map.statusword);
         if (!(sw & 0x0008)) { printf("Fault cleared\n"); goto ok; }
      }
      sdo_w32(c, a->slave, 0x203f, 0x00, 0); sleep_cyc(b, 100);
      pdo_w16(s, &a->map.controlword, 0x0080); sleep_cyc(b, 100);
      pdo_w16(s, &a->map.controlword, 0x0000); sleep_cyc(b, 100);
      sw = pdo_r16(s, &a->map.statusword);
      if (sw & 0x0008) { printf("Fault persists - use InoDriveShop or power cycle to reset\n"); return 0; }
      printf("Fault cleared (SDO)\n");
   }
ok:

   /* 设置 PV 模式及轮廓加减速度 */
   printf("Mode -> PV (3)\n"); sdo_w8(c, a->slave, 0x6060, 0x00, MODE_PV);
   printf("0x6083=%u 0x6084=%u\n", ACCEL_DEFAULT, DECEL_DEFAULT);
   sdo_w32(c, a->slave, 0x6083, 0x00, ACCEL_DEFAULT);50
   sdo_w32(c, a->slave, 0x6084, 0x00, DECEL_DEFAULT);
   drain_errs(c); sleep_cyc(b, 20);
   {
      uint8 mode = 0;
      if (sdo_r8(c, a->slave, 0x6061, 0x00, &mode))
         printf("Mode display: %d\n", (int8)mode);
   }

   /* 通过 SDO/PDO 写入最大轮廓速度 */
   sdo_w32(c, a->slave, 0x607f, 0x00, MAX_VELOCITY);
   if (a->map.max_profile_velocity.present)
      pdo_w32(s, &a->map.max_profile_velocity, MAX_VELOCITY);

   /* 从当前状态启动 CiA402 状态机。
    * 非故障状态下，Shutdown（0x0006）可进入 ready-to-switch-on。
    * 驱动器进入 OP 后，功率级、编码器、DC 同步等内部初始化耗时不固定，
    * 因此这里允许多次重试。 */
   for (int attempt = 0; attempt < 30; attempt++) {
      if (attempt > 0) {
         printf("Retry %d/30, waiting 1000ms...\n", attempt + 1);
         sleep_cyc(b, 1000);
      }

      sw = pdo_r16(s, &a->map.statusword);
      printf("Attempt %d: sw=0x%04x (%s)\n", attempt + 1, sw, cia402_str(sw));
      if (sw & 0x0008) { printf("  Fault detected, aborting\n"); return 0; }
      if (is_oe(sw)) {
         axis_write_idle(b, a, 0x000f);
         a->enabled = 1;
         printf("Enabled (already OP), sw=0x%04x\n", sw);
         return 1;
      }

      /* 状态机流程：shutdown -> switch-on -> enable-operation */

      /* shutdown -> ready-to-switch-on */
      int ok_rts = 0;
      if (is_so(sw)) {
         ok_rts = 1;
      } else if (is_rts(sw)) {
         /*
          * 有些驱动虽然状态字已经是 ready-to-switch-on，但仍需要主站稳定
          * 保持一小段 shutdown(0x0006)，再切到 switch-on(0x0007)。
          * 直接跳过 0x0006 会让状态长期卡在 0x1231。
         */
         for (int i = 0; i < 50; i++) {
            axis_write_idle(b, a, 0x0006);
            sleep_cyc(b, 1);
         }
         sw = pdo_r16(s, &a->map.statusword);
         ok_rts = is_rts(sw) || is_so(sw) || is_oe(sw);
      } else {
         for (int i = 0; i < 2000; i++) {
            axis_write_idle(b, a, 0x0006);
            sleep_cyc(b, 1);
            sw = pdo_r16(s, &a->map.statusword);
            if (sw & 0x0008) { printf("  Fault detected during shutdown, sw=0x%04x\n", sw); return 0; }
            if (is_oe(sw) || is_so(sw) || is_rts(sw)) { ok_rts = 1; break; }
         }
      }
      if (!ok_rts) { printf("  shutdown timeout (sw=0x%04x)\n", sw); continue; }

      sw = pdo_r16(s, &a->map.statusword);
      if (is_oe(sw)) {
         axis_write_idle(b, a, 0x000f);
         a->enabled = 1;
         printf("Enabled (OP), sw=0x%04x\n", sw);
         return 1;
      }

      /* switch-on -> switched-on */
      int ok_so = is_so(sw);
      if (!ok_so) {
         for (int i = 0; i < 2000; i++) {
            axis_write_idle(b, a, 0x0007);
            sleep_cyc(b, 1);
            sw = pdo_r16(s, &a->map.statusword);
            if (sw & 0x0008) { printf("  Fault detected during switch-on, sw=0x%04x\n", sw); return 0; }
            if (is_oe(sw) || is_so(sw)) { ok_so = 1; break; }
         }
      }
      if (!ok_so) {
         printf("  switch-on timeout (sw=0x%04x cw_out=0x%04x wkc=%d bad=%d)\n",
                sw, axis_output_controlword(b, a), b->wkc, b->bad_wkc);
         continue;
      }

      sw = pdo_r16(s, &a->map.statusword);
      if (is_oe(sw)) {
         axis_write_idle(b, a, 0x000f);
         a->enabled = 1;
         printf("Enabled (OP), sw=0x%04x\n", sw);
         return 1;
      }

      /* enable-operation -> operation-enabled */
      for (int i = 0; i < 2000; i++) {
         axis_write_idle(b, a, 0x000f);
         sleep_cyc(b, 1);
         sw = pdo_r16(s, &a->map.statusword);
         if (sw & 0x0008) {
            printf("  FAULT during enable (sw=0x%04x err=0x%04x)\n",
                   sw, a->map.error_code.present ? pdo_r16(s, &a->map.error_code) : 0);
            return 0;
         }
         if (is_oe(sw)) {
            printf("Enabled (OP), sw=0x%04x\n", sw);
            a->enabled = 1;
            return 1;
         }
      }
      printf("  enable timeout (sw=0x%04x)\n", sw);
   }
   printf("FAIL: could not enable after 30 attempts\n");
   return 0;
}

static void axis_write_velocity(ServoBus *b, ServoAxis *a, int32 vel)
{
   ec_slavet *s = axis_slave(b, a);

   if (a->map.modes_of_operation.present)
      pdo_w8(s, &a->map.modes_of_operation, MODE_PV);
   if (a->map.max_profile_velocity.present)
      pdo_w32(s, &a->map.max_profile_velocity, MAX_VELOCITY);
   pdo_w32(s, &a->map.target_velocity, vel);
   pdo_w16(s, &a->map.controlword, 0x000f);
   a->target_velocity = vel;
}

/* ---- PV 速度运行控制 ---- */
static void motor_run_selected(ServoBus *b, int mask, const int32 vel[])
{
   g_in_motion  = 1;
   g_stop_motion = 0;
   printf("Running selected axes (drive handles ramp). Ctrl-C to stop.\n");

   /*
    * 使能阶段已经证明：主线程独占 PDO 写入时最稳定。
    * 运动阶段同样暂停后台周期线程，由主线程按周期写速度并收发 PDO，
    * 避免主线程改输出缓冲、周期线程同时发包造成目标速度不稳定。
    */
   stop_thread(b);

   for (int i = 0; i < b->axis_count; i++)
      if (mask & (1 << i))
         axis_write_velocity(b, &b->axis[i], vel[i]);
   sleep_cyc(b, 5);

   int tick = 0;
   while (!g_stop_motion) {
      for (int i = 0; i < b->axis_count; i++) {
         if (!(mask & (1 << i))) continue;
         ServoAxis *a = &b->axis[i];
         ec_slavet *s = axis_slave(b, a);
         uint16 sw = pdo_r16(s, &a->map.statusword);
         if (sw & 0x0008) { printf("* Axis %d FAULT 0x%04x\n", i + 1, sw); g_stop_motion = 1; break; }
         if (!is_oe(sw))  { printf("Axis %d lost OP (0x%04x)\n", i + 1, sw); g_stop_motion = 1; break; }
         axis_write_velocity(b, a, vel[i]);
      }
      sleep_cyc(b, 1);

      if (++tick >= 1000) {
         tick = 0;
         for (int i = 0; i < b->axis_count; i++) {
            if (!(mask & (1 << i))) continue;
            ServoAxis *a = &b->axis[i];
            ec_slavet *s = axis_slave(b, a);
            printf("Axis %d run: sw=0x%04x cw=0x%04x tv=%d pos=%d wkc=%d bad=%d\n",
                   i + 1,
                   pdo_r16(s, &a->map.statusword),
                   axis_output_controlword(b, a),
                   axis_output_target_velocity(b, a),
                   a->map.actual_position.present ? pdo_r32(s, &a->map.actual_position) : 0,
                   b->wkc,
                   b->bad_wkc);
         }
      }
   }

   printf("Stopping...\n");
   for (int i = 0; i < b->axis_count; i++)
      if (mask & (1 << i))
         axis_write_velocity(b, &b->axis[i], 0);
   sleep_cyc(b, 500);
   g_in_motion = 0;
   printf("Stopped (still in OP)\n");
   if (!start_thread(b))
      printf("Warning: cyclic thread restart failed after motion\n");
}

static void motor_run_axis(ServoBus *b, ServoAxis *a, int axis_no, int32 vel)
{
   int32 v[SERVO_AXES] = { 0 };
   v[axis_no - 1] = vel;
   printf("Axis %d (slave %u) target velocity %d\n", axis_no, a->slave, vel);
   motor_run_selected(b, 1 << (axis_no - 1), v);
}

/* ---- CiA402 去使能流程（周期线程仍在运行时调用） ---- */
static void servo_disable_axis(ServoBus *b, ServoAxis *a)
{
   ec_slavet *s = axis_slave(b, a);
   uint16 sw = pdo_r16(s, &a->map.statusword);
   printf("Axis slave %u: disabling CiA402...\n", a->slave);

   if ((sw & 0x006f) == 0x0040) { printf("  Already disabled\n"); return; }
   if (sw & 0x0008) { printf("  In fault, skipping\n"); return; }

   /* 第 1 步：operation-enabled -> switched-on */
   if ((sw & 0x006f) == 0x0027) {
      printf("  disable-op...\n");
      for (int i = 0; i < 500; i++) {
         pdo_w16(s, &a->map.controlword, 0x0007);
         pdo_w32(s, &a->map.target_velocity, 0);
         osal_usleep(b->cycle_us);
         sw = pdo_r16(s, &a->map.statusword);
         if ((sw & 0x006f) == 0x0023 || (sw & 0x006f) == 0x0021) break;
      }
   }
   /* 第 2 步：进入 switch-on-disabled */
   if ((sw & 0x006f) != 0x0040) {
      printf("  disable-voltage...\n");
      for (int i = 0; i < 500; i++) {
         pdo_w16(s, &a->map.controlword, 0x0000);
         pdo_w32(s, &a->map.target_velocity, 0);
         osal_usleep(b->cycle_us);
         if ((pdo_r16(s, &a->map.statusword) & 0x006f) == 0x0040) { printf("  OK\n"); return; }
      }
   }
   printf("  sw=0x%04x\n", pdo_r16(s, &a->map.statusword));
}

/* ---- 安全退出流程（参考 InoDriveShop 的下电顺序） ---- */
static void bus_shutdown(ServoBus *b)
{
   ecx_contextt *c = &b->ctx;
   printf("=== Safe shutdown ===\n");

   /* 1. 执行 CiA402 去使能，此时周期线程必须保持运行 */
   for (int i = 0; i < b->axis_count; i++)
      servo_disable_axis(b, &b->axis[i]);

   /* 2. 停止周期 PDO 通信 */
   printf("Stop cyclic...\n"); stop_thread(b);

   /* 3. EtherCAT 状态切换：OP(8) -> SAFE_OP(4) -> PRE_OP(2) -> INIT(1) */
   int states[] = { EC_STATE_SAFE_OP, EC_STATE_PRE_OP, EC_STATE_INIT };
   const char *names[] = { "SAFE_OP", "PRE_OP", "INIT" };
   for (int i = 0; i < 3; i++) {
      printf("  鈫?%s\n", names[i]);
      c->slavelist[0].state = states[i]; ecx_writestate(c, 0);
      for (int j = 0; j < 200; j++) {
         roundtrip(b); ecx_statecheck(c, 0, states[i], b->cycle_us);
         if (c->slavelist[0].state == states[i]) break;
         osal_usleep(b->cycle_us);
      }
   }

   /* 4. 关闭 DC 同步 */
   for (int i = 1; i <= c->slavecount; i++)
      if (c->slavelist[i].DCactive) ecx_dcsync0(c, i, FALSE, 0, 0);

   /* 5. 关闭 EtherCAT 主站连接 */
   ecx_close(c);
   printf("=== Done ===\n");
}

/* ---- 诊断信息打印 ---- */
static void show_status_axis(ServoBus *b, ServoAxis *a, int axis_no)
{
   ecx_contextt *c = &b->ctx;
   ec_slavet *s = axis_slave(b, a);
   uint16 sw = pdo_r16(s, &a->map.statusword);
   printf("Axis %d (slave %u)\n", axis_no, a->slave);
   printf("  Statusword: 0x%04x (%s)\n", sw, cia402_str(sw));
   if (a->map.error_code.present) printf("Error: 0x%04x\n", pdo_r16(s, &a->map.error_code));
   uint32 d; if (sdo_r32(c, a->slave, 0x203f, 0x00, &d)) printf("  0x203F: %08x\n", d);
   int8 m; if (sdo_r8(c, a->slave, 0x6061, 0x00, (uint8*)&m)) printf("  Mode display: %d\n", m);
   if (a->map.actual_position.present) printf("  Pos: %d\n", pdo_r32(s, &a->map.actual_position));
}

static void show_status(ServoBus *b)
{
   for (int i = 0; i < b->axis_count; i++)
      show_status_axis(b, &b->axis[i], i + 1);
   printf("WKC=%d bad=%d cycle=%d\n", b->wkc, b->bad_wkc, b->cycle);
   drain_errs(&b->ctx);
}

/* ---- 主函数入口 ---- */
static void usage(const char *p)
{
   printf("Usage:\n");
   printf("  %s IFACE enable [cycle_us]      Enable two servos, prompt for velocity\n", p);
   printf("  %s IFACE status [cycle_us]      Show diagnostics for both servos\n", p);
   printf("  %s IFACE fault-reset [cycle_us] Reset faults through enable sequence\n", p);
   printf("\nFlow:\n");
   printf("  1 50000             run axis 1\n");
   printf("  2 -50000            run axis 2\n");
   printf("  both 50000 -50000   run both axes together\n");
   printf("  all 0               run both axes at same velocity\n");
   printf("  status              show diagnostics\n");
   printf("  Ctrl-C during run   stops selected motion, stays in OP\n");
   printf("  q at prompt         safe shutdown\n");
}

static int enable_all_axes(ServoBus *b)
{
   for (int i = 0; i < b->axis_count; i++)
      if (!servo_enable_axis(b, &b->axis[i]))
         return 0;
   return 1;
}

static int axes_ok(ServoBus *b)
{
   for (int i = 0; i < b->axis_count; i++) {
      ServoAxis *a = &b->axis[i];
      ec_slavet *s = axis_slave(b, a);
      uint16 sw = pdo_r16(s, &a->map.statusword);
      if (sw & 0x0008) { printf("Axis %d fault (0x%04x)\n", i + 1, sw); return 0; }
      if (!is_oe(sw)) { printf("Axis %d not operation-enabled (0x%04x)\n", i + 1, sw); return 0; }
   }
   return 1;
}

static int parse_velocity(const char *text, int32 *vel)
{
   char *end;
   long v = strtol(text, &end, 0);
   while (*end && isspace((unsigned char)*end)) end++;
   if (end == text || *end) return 0;
   if (labs(v) > MAX_VELOCITY) {
      printf("Too large (max %d)\n", MAX_VELOCITY);
      return 0;
   }
   *vel = (int32)v;
   return 1;
}

int main(int argc, char **argv)
{
   ServoBus bus;
   const char *cmd;
   uint32 cyc;
   int ok = 1, started = 0;

   lock_process_memory();

   if (argc < 3) { usage(argv[0]); return 1; }
   cmd = argv[2];
   cyc = (argc >= 4) ? (uint32)strtoul(argv[3], NULL, 0) : DEFAULT_CYCLE_US;

   if (strcmp(cmd, "status") == 0) {
      if (!bus_init(&bus, argv[1], cyc)) { ecx_close(&bus.ctx); return 1; }
      started = 1; show_status(&bus);
   }
   else if (strcmp(cmd, "fault-reset") == 0) {
      if (!bus_init(&bus, argv[1], cyc)) { ecx_close(&bus.ctx); return 1; }
      started = 1;
      /* 故障复位逻辑已包含在 servo_enable 内部 */
      enable_all_axes(&bus);
      show_status(&bus);
   }
   else if (strcmp(cmd, "enable") == 0) {
      if (!bus_init(&bus, argv[1], cyc)) { ecx_close(&bus.ctx); return 1; }
      started = 1;
      if (!enable_all_axes(&bus)) { ok = 0; goto done; }
      if (!start_thread(&bus)) { printf("Thread fail\n"); ok = 0; goto done; }
      sleep_cyc(&bus, 200);

      /* ---- 命令行交互循环 ---- */
      signal(SIGINT, on_signal);
      signal(SIGTERM, on_signal);
      g_running = 1;
      printf("Ready. Commands: '1 vel', '2 vel', 'both v1 v2', 'all vel', 'status', 'q'.\n");

      while (g_running) {
         printf("> "); fflush(stdout);

         /* 读取一行命令 */
         char line[64]; int pos = 0;
         while (pos < (int)sizeof(line) - 1) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) <= 0) { g_running = 0; break; }
            if (ch == '\n') break;
            line[pos++] = ch;
         }
         line[pos] = '\0';
         if (!g_running) break;

         if (line[0] == 'q' || line[0] == 'Q') break;
         if (line[0] == '\0') continue;

         char *tok = strtok(line, " \t\r");
         if (!tok) continue;

         if (strcmp(tok, "status") == 0) {
            show_status(&bus);
            continue;
         }

         if (!axes_ok(&bus)) { g_running = 0; break; }

         if (strcmp(tok, "both") == 0) {
            char *t1 = strtok(NULL, " \t\r");
            char *t2 = strtok(NULL, " \t\r");
            int32 vel[SERVO_AXES] = { 0 };
            if (!t1 || !t2 || !parse_velocity(t1, &vel[0]) || !parse_velocity(t2, &vel[1])) {
               printf("? Usage: both <axis1_vel> <axis2_vel>\n");
               continue;
            }
            motor_run_selected(&bus, 0x03, vel);
            continue;
         }

         if (strcmp(tok, "all") == 0) {
            char *tv = strtok(NULL, " \t\r");
            int32 v, vel[SERVO_AXES] = { 0 };
            if (!tv || !parse_velocity(tv, &v)) {
               printf("? Usage: all <velocity>\n");
               continue;
            }
            for (int i = 0; i < bus.axis_count; i++) vel[i] = v;
            motor_run_selected(&bus, (1 << bus.axis_count) - 1, vel);
            continue;
         }

         if ((strcmp(tok, "1") == 0) || (strcmp(tok, "2") == 0)) {
            int axis_no = atoi(tok);
            char *tv = strtok(NULL, " \t\r");
            int32 vel;
            if (!tv || !parse_velocity(tv, &vel)) {
               printf("? Usage: %d <velocity>\n", axis_no);
               continue;
            }
            motor_run_axis(&bus, &bus.axis[axis_no - 1], axis_no, vel);
            continue;
         }

         {
            int32 vel;
            if (parse_velocity(tok, &vel)) {
               printf("Legacy input: applying velocity to axis 1\n");
               motor_run_axis(&bus, &bus.axis[0], 1, vel);
            } else {
               printf("? Commands: 1 vel, 2 vel, both v1 v2, all vel, status, q\n");
            }
         }
      }
   }
   else { usage(argv[0]); ok = 0; }

done:
   if (started) bus_shutdown(&bus);
   return ok ? 0 : 1;
}
