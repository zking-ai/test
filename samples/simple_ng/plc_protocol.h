#ifndef PLC_PROTOCOL_H
#define PLC_PROTOCOL_H

#include "plc_state.h"
#include <stddef.h>
#include <stdint.h>

#define PLC_FRAME_MIN_BYTES 8
#define PLC_FRAME_STATUS_BYTES 100

typedef enum {
   PLC_PARSE_OK = 0,
   PLC_PARSE_INCOMPLETE,
   PLC_PARSE_BAD_FRAME
} PlcParseResult;

const char *plc_command_name(PlcCommandType type);
PlcParseResult plc_protocol_parse(const uint8_t *buf, size_t len, PlcCommand *cmd);
void plc_protocol_apply_command(PlcState *state, const PlcCommand *cmd);
size_t plc_protocol_build_status(const PlcState *state, uint8_t *out, size_t cap);
void plc_protocol_dump_frame(const uint8_t *buf, size_t len);

#endif
