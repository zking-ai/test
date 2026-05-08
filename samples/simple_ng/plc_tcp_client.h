#ifndef PLC_TCP_CLIENT_H
#define PLC_TCP_CLIENT_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
   int fd;
   char host[64];
   int port;
   int connected;
} PlcTcpClient;

void plc_tcp_init(PlcTcpClient *client, const char *host, int port);
void plc_tcp_close(PlcTcpClient *client);
int plc_tcp_connect(PlcTcpClient *client);
int plc_tcp_recv(PlcTcpClient *client, uint8_t *buf, size_t cap, int timeout_ms);
int plc_tcp_send_all(PlcTcpClient *client, const uint8_t *buf, size_t len);

#endif
