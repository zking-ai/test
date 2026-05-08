#include "plc_protocol.h"
#include <stdio.h>
#include <string.h>

static uint16_t plc_word(const uint8_t *p)
{
   return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static void put_plc_word(uint8_t *p, uint16_t v)
{
   p[0] = (uint8_t)(v & 0xff);
   p[1] = (uint8_t)(v >> 8);
}

const char *plc_command_name(PlcCommandType type)
{
   switch (type) {
   case PLC_CMD_NONE: return "none";
   case PLC_CMD_STOP_ALL: return "stop-all";
   case PLC_CMD_RESET: return "reset";
   case PLC_CMD_ENABLE: return "enable";
   case PLC_CMD_DISABLE: return "disable";
   case PLC_CMD_BELT_LOAD: return "belt-load";
   case PLC_CMD_BELT_UNLOAD: return "belt-unload";
   case PLC_CMD_BELT_STOP: return "belt-stop";
   case PLC_CMD_VACUUM_ON: return "vacuum-on";
   case PLC_CMD_VACUUM_OFF: return "vacuum-off";
   case PLC_CMD_UPDOWN_MOVE: return "updown-move";
   case PLC_CMD_FLEX_MOVE: return "flex-move";
   case PLC_CMD_DITUO_PRESET: return "dituo-preset";
   case PLC_CMD_LEVEL_HOME: return "level-home";
   case PLC_CMD_LEVEL_AUTO: return "level-auto";
   case PLC_CMD_LOAD: return "load";
   case PLC_CMD_UNLOAD: return "unload";
   case PLC_CMD_BRAKE_ON: return "brake-on";
   case PLC_CMD_BRAKE_OFF: return "brake-off";
   case PLC_CMD_LIGHT_STATE: return "light-state";
   case PLC_CMD_FILL_LIGHT_ON: return "fill-light-on";
   case PLC_CMD_FILL_LIGHT_OFF: return "fill-light-off";
   case PLC_CMD_LASER_ON: return "laser-on";
   case PLC_CMD_LASER_OFF: return "laser-off";
   case PLC_CMD_CHASSIS_MANUAL_ON: return "chassis-manual-on";
   case PLC_CMD_CHASSIS_MANUAL_OFF: return "chassis-manual-off";
   case PLC_CMD_XP_BITMAP: return "xp-bitmap";
   case PLC_CMD_UNKNOWN: return "unknown";
   }
   return "?";
}

static PlcCommandType decode_h99(uint16_t op, uint16_t arg)
{
   (void)arg;
   switch (op) {
   case 0xffff: return PLC_CMD_STOP_ALL;
   case 0x0124: return PLC_CMD_RESET;
   case 0x0125: return PLC_CMD_ENABLE;
   case 0x0025: return PLC_CMD_DISABLE;
   case 0x0135: return PLC_CMD_BELT_LOAD;
   case 0x0136: return PLC_CMD_BELT_UNLOAD;
   case 0x3635: return PLC_CMD_BELT_STOP;
   case 0x0140: return PLC_CMD_VACUUM_ON;
   case 0x0040: return PLC_CMD_VACUUM_OFF;
   case 0x0151: return PLC_CMD_UPDOWN_MOVE;
   case 0x0159: return PLC_CMD_FLEX_MOVE;
   case 0x0052:
   case 0x0152:
   case 0x1152:
   case 0x2052:
   case 0x3152:
   case 0x3252:
   case 0x4152:
   case 0x4252: return PLC_CMD_DITUO_PRESET;
   case 0x0161: return PLC_CMD_LEVEL_HOME;
   case 0x0261: return PLC_CMD_LEVEL_AUTO;
   default: return PLC_CMD_UNKNOWN;
   }
}

static PlcCommandType decode_h199(uint16_t op)
{
   switch (op) {
   case 0x0121: return PLC_CMD_LOAD;
   case 0x0122: return PLC_CMD_UNLOAD;
   case 0x2221: return PLC_CMD_BELT_STOP;
   case 0x012a: return PLC_CMD_BRAKE_ON;
   case 0x002a: return PLC_CMD_BRAKE_OFF;
   case 0x012d: return PLC_CMD_FILL_LIGHT_ON;
   case 0x002d: return PLC_CMD_FILL_LIGHT_OFF;
   case 0x012e: return PLC_CMD_LASER_ON;
   case 0x002e: return PLC_CMD_LASER_OFF;
   case 0x0131: return PLC_CMD_CHASSIS_MANUAL_ON;
   case 0x0031: return PLC_CMD_CHASSIS_MANUAL_OFF;
   default:
      if ((op & 0x00ff) == 0x2b) return PLC_CMD_LIGHT_STATE;
      if ((op & 0x00ff) == 0x2c) return PLC_CMD_LIGHT_STATE;
      return PLC_CMD_UNKNOWN;
   }
}

PlcParseResult plc_protocol_parse(const uint8_t *buf, size_t len, PlcCommand *cmd)
{
   uint16_t head;
   uint16_t tail;

   memset(cmd, 0, sizeof(*cmd));
   cmd->type = PLC_CMD_NONE;

   if (len < PLC_FRAME_MIN_BYTES) return PLC_PARSE_INCOMPLETE;

   head = plc_word(&buf[0]);
   cmd->raw0 = head;
   cmd->raw1 = plc_word(&buf[2]);
   cmd->raw2 = plc_word(&buf[4]);
   cmd->raw3 = plc_word(&buf[6]);

   if (head == 0x0099 || head == 0x0199) {
      tail = cmd->raw3;
      if (tail != 0xff01) return PLC_PARSE_BAD_FRAME;
   } else if (head == 0x1099) {
      if (len < 12) return PLC_PARSE_INCOMPLETE;
      tail = plc_word(&buf[10]);
      if (tail != 0xff01) return PLC_PARSE_BAD_FRAME;
   } else if (head == 0x1199) {
      if (len < 24) return PLC_PARSE_INCOMPLETE;
      tail = plc_word(&buf[22]);
      if (tail != 0xff01) return PLC_PARSE_BAD_FRAME;
   } else {
      return PLC_PARSE_BAD_FRAME;
   }

   if (head == 0x0099) {
      cmd->type = decode_h99(cmd->raw1, cmd->raw2);
      cmd->value = (int16_t)cmd->raw2;
      if (cmd->type == PLC_CMD_UPDOWN_MOVE) cmd->axis = 0;
      else if (cmd->type == PLC_CMD_FLEX_MOVE) cmd->axis = 1;
   } else if (head == 0x0199) {
      cmd->type = decode_h199(cmd->raw1);
      cmd->value = cmd->raw2;
   } else if (head == 0x1099 || head == 0x1199) {
      cmd->type = PLC_CMD_XP_BITMAP;
      cmd->value = 0;
   }

   return PLC_PARSE_OK;
}

void plc_protocol_apply_command(PlcState *state, const PlcCommand *cmd)
{
   state->last_cmd = *cmd;
   switch (cmd->type) {
   case PLC_CMD_STOP_ALL:
      state->load_request = 0;
      state->unload_request = 0;
      state->vacuum_request = 0;
      break;
   case PLC_CMD_RESET:
      state->reset_request = 1;
      break;
   case PLC_CMD_ENABLE:
      state->enable_request = 1;
      break;
   case PLC_CMD_DISABLE:
      state->enable_request = 0;
      break;
   case PLC_CMD_LOAD:
      state->load_request = 1;
      state->unload_request = 0;
      break;
   case PLC_CMD_UNLOAD:
      state->load_request = 0;
      state->unload_request = 1;
      break;
   case PLC_CMD_VACUUM_ON:
      state->vacuum_request = 1;
      break;
   case PLC_CMD_VACUUM_OFF:
      state->vacuum_request = 0;
      break;
   case PLC_CMD_BRAKE_ON:
      state->brake_request = 1;
      break;
   case PLC_CMD_BRAKE_OFF:
      state->brake_request = 0;
      break;
   case PLC_CMD_FILL_LIGHT_ON:
      state->fill_light = 1;
      break;
   case PLC_CMD_FILL_LIGHT_OFF:
      state->fill_light = 0;
      break;
   case PLC_CMD_LASER_ON:
      state->laser_light = 1;
      break;
   case PLC_CMD_LASER_OFF:
      state->laser_light = 0;
      break;
   case PLC_CMD_CHASSIS_MANUAL_ON:
      state->chassis_manual = 1;
      break;
   case PLC_CMD_CHASSIS_MANUAL_OFF:
      state->chassis_manual = 0;
      break;
   case PLC_CMD_LIGHT_STATE:
      state->light_state = cmd->raw1;
      state->sound_state = cmd->raw2;
      break;
   case PLC_CMD_UPDOWN_MOVE:
   case PLC_CMD_FLEX_MOVE:
      if (cmd->axis >= 0 && cmd->axis < PLC_AXIS_COUNT)
         state->axis[cmd->axis].target_position = cmd->value;
      break;
   case PLC_CMD_XP_BITMAP:
      break;
   case PLC_CMD_UNKNOWN:
      state->unknown_cmd_count++;
      break;
   default:
      break;
   }
}

size_t plc_protocol_build_status(const PlcState *state, uint8_t *out, size_t cap)
{
   uint16_t words[PLC_STATUS_WORDS];
   int i;

   if (cap < PLC_FRAME_STATUS_BYTES) return 0;
   memset(words, 0, sizeof(words));

   words[0] = 0x0199;
   if (!state->estop_ok) words[1] |= (uint16_t)(1u << 0);
   if (state->load_request) words[1] |= (uint16_t)(1u << 4);
   if (state->unload_request) words[1] |= (uint16_t)(1u << 5);
   if (!state->load_request && !state->unload_request) words[1] |= (uint16_t)(1u << 6);
   if (state->enable_request) words[1] |= (uint16_t)(1u << 7);
   if (state->vacuum_request) words[1] |= (uint16_t)(1u << 14);
   if (state->brake_request) words[1] |= (uint16_t)(1u << 15);

   for (i = 0; i < PLC_AXIS_COUNT && i < 16; i++) {
      if (state->axis[i].fault) words[2] |= (uint16_t)(1u << i);
   }

   words[7] = state->estop_ok ? 0 : 0x0001;
   words[10] = state->status_words[10];
   words[11] = state->status_words[11];
   words[12] = state->status_words[12];
   words[13] = state->status_words[13];
   words[14] = state->status_words[14];
   words[20] = state->status_words[20];
   words[21] = state->status_words[21];
   words[25] = (uint16_t)state->axis[0].actual_position;
   words[49] = 0xff01;

   for (i = 0; i < PLC_STATUS_WORDS; i++) put_plc_word(&out[i * 2], words[i]);
   return PLC_FRAME_STATUS_BYTES;
}

void plc_protocol_dump_frame(const uint8_t *buf, size_t len)
{
   size_t i;
   for (i = 0; i < len; i++) {
      printf("%02x", buf[i]);
      if (i + 1 < len) printf(" ");
   }
}
