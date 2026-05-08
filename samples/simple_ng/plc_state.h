#ifndef PLC_STATE_H
#define PLC_STATE_H

#include <stdint.h>
#include <string.h>
#include <time.h>

#define PLC_AXIS_COUNT 4
#define PLC_STATUS_WORDS 50

typedef enum {
   PLC_CMD_NONE = 0,
   PLC_CMD_STOP_ALL,
   PLC_CMD_RESET,
   PLC_CMD_ENABLE,
   PLC_CMD_DISABLE,
   PLC_CMD_BELT_LOAD,
   PLC_CMD_BELT_UNLOAD,
   PLC_CMD_BELT_STOP,
   PLC_CMD_VACUUM_ON,
   PLC_CMD_VACUUM_OFF,
   PLC_CMD_UPDOWN_MOVE,
   PLC_CMD_FLEX_MOVE,
   PLC_CMD_DITUO_PRESET,
   PLC_CMD_LEVEL_HOME,
   PLC_CMD_LEVEL_AUTO,
   PLC_CMD_LOAD,
   PLC_CMD_UNLOAD,
   PLC_CMD_BRAKE_ON,
   PLC_CMD_BRAKE_OFF,
   PLC_CMD_LIGHT_STATE,
   PLC_CMD_FILL_LIGHT_ON,
   PLC_CMD_FILL_LIGHT_OFF,
   PLC_CMD_LASER_ON,
   PLC_CMD_LASER_OFF,
   PLC_CMD_CHASSIS_MANUAL_ON,
   PLC_CMD_CHASSIS_MANUAL_OFF,
   PLC_CMD_XP_BITMAP,
   PLC_CMD_UNKNOWN
} PlcCommandType;

typedef struct {
   PlcCommandType type;
   uint16_t raw0;
   uint16_t raw1;
   uint16_t raw2;
   uint16_t raw3;
   int32_t value;
   int axis;
} PlcCommand;

typedef struct {
   uint16_t statusword;
   uint16_t error_code;
   int32_t actual_position;
   int32_t target_position;
   int enabled;
   int fault;
} PlcAxisState;

typedef struct {
   int connected_to_pc;
   int pc_rx_count;
   int pc_tx_count;
   int parse_error_count;
   int unknown_cmd_count;

   int enable_request;
   int estop_ok;
   int reset_request;
   int load_request;
   int unload_request;
   int brake_request;
   int vacuum_request;
   int fill_light;
   int laser_light;
   int chassis_manual;
   uint16_t light_state;
   uint16_t sound_state;
   uint16_t xp[10];

   PlcAxisState axis[PLC_AXIS_COUNT];
   uint16_t status_words[PLC_STATUS_WORDS];

   PlcCommand last_cmd;
   struct timespec last_rx_time;
} PlcState;

static inline void plc_state_init(PlcState *s)
{
   int i;
   memset(s, 0, sizeof(*s));
   for (i = 0; i < PLC_STATUS_WORDS; i++) s->status_words[i] = 0;
   for (i = 0; i < PLC_AXIS_COUNT; i++) {
      s->axis[i].statusword = 0;
      s->axis[i].error_code = 0;
      s->axis[i].actual_position = 0;
      s->axis[i].target_position = 0;
      s->axis[i].enabled = 0;
      s->axis[i].fault = 0;
   }
   s->connected_to_pc = 0;
   s->pc_rx_count = 0;
   s->pc_tx_count = 0;
   s->parse_error_count = 0;
   s->unknown_cmd_count = 0;
   s->enable_request = 0;
   s->estop_ok = 0;
   s->reset_request = 0;
   s->load_request = 0;
   s->unload_request = 0;
   s->brake_request = 0;
   s->vacuum_request = 0;
   s->fill_light = 0;
   s->laser_light = 0;
   s->chassis_manual = 0;
   s->light_state = 0;
   s->sound_state = 0;
   for (i = 0; i < 10; i++) s->xp[i] = 0;
   s->last_cmd.type = PLC_CMD_NONE;
}

#endif
