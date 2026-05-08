#include "plc_tcp_client.h"
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

void plc_tcp_init(PlcTcpClient *client, const char *host, int port)
{
   memset(client, 0, sizeof(*client));
   client->fd = -1;
   snprintf(client->host, sizeof(client->host), "%s", host ? host : "192.168.1.200");
   client->port = port > 0 ? port : 4000;
}

void plc_tcp_close(PlcTcpClient *client)
{
   if (client->fd >= 0) close(client->fd);
   client->fd = -1;
   client->connected = 0;
}

int plc_tcp_connect(PlcTcpClient *client)
{
   struct sockaddr_in addr;
   int fd;

   plc_tcp_close(client);

   fd = socket(AF_INET, SOCK_STREAM, 0);
   if (fd < 0) {
      printf("tcp: socket failed: %s\n", strerror(errno));
      return 0;
   }

   memset(&addr, 0, sizeof(addr));
   addr.sin_family = AF_INET;
   addr.sin_port = htons((uint16_t)client->port);
   if (inet_pton(AF_INET, client->host, &addr.sin_addr) != 1) {
      printf("tcp: bad host %s\n", client->host);
      close(fd);
      return 0;
   }

   printf("tcp: connecting to %s:%d...\n", client->host, client->port);
   if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
      printf("tcp: connect failed: %s\n", strerror(errno));
      close(fd);
      return 0;
   }

   client->fd = fd;
   client->connected = 1;
   printf("tcp: connected\n");
   return 1;
}

int plc_tcp_recv(PlcTcpClient *client, uint8_t *buf, size_t cap, int timeout_ms)
{
   fd_set rfds;
   struct timeval tv;
   int ret;

   if (!client->connected || client->fd < 0) return -1;

   FD_ZERO(&rfds);
   FD_SET(client->fd, &rfds);
   tv.tv_sec = timeout_ms / 1000;
   tv.tv_usec = (timeout_ms % 1000) * 1000;

   ret = select(client->fd + 1, &rfds, NULL, NULL, &tv);
   if (ret < 0) {
      if (errno == EINTR) return 0;
      printf("tcp: select failed: %s\n", strerror(errno));
      plc_tcp_close(client);
      return -1;
   }
   if (ret == 0) return 0;

   ret = (int)recv(client->fd, buf, cap, 0);
   if (ret <= 0) {
      printf("tcp: peer closed or recv failed: %s\n", ret == 0 ? "closed" : strerror(errno));
      plc_tcp_close(client);
      return -1;
   }
   return ret;
}

int plc_tcp_send_all(PlcTcpClient *client, const uint8_t *buf, size_t len)
{
   size_t sent = 0;
   while (sent < len) {
      ssize_t n;
      if (!client->connected || client->fd < 0) return 0;
      n = send(client->fd, buf + sent, len - sent, 0);
      if (n <= 0) {
         printf("tcp: send failed: %s\n", strerror(errno));
         plc_tcp_close(client);
         return 0;
      }
      sent += (size_t)n;
   }
   return 1;
}
