#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"

#include "server.h"

#define HTTPD_DEBUG         LWIP_DBG_ON

static void ICACHE_FLASH_ATTR
tcp_server_recv(void *arg)
{
  struct netconn *conn = arg;
  struct netbuf *inbuf;
  char *buf;
  u16_t buflen;
  err_t err;

  printf("TCP: recv task resetting arduino\n");

  // start by resetting the arduino/arm
  reset_arduino();

  err = netconn_recv(conn, &inbuf);
  while (err == ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);
    printf("TCP: recv %d bytes\n", buflen);
    UART0_write(buf, buflen);
    netbuf_delete(inbuf);
    err = netconn_recv(conn, &inbuf);
  }

  /* Close the connection (server closes in HTTP) */
  netconn_close(conn);
  netconn_delete(conn); // TODO: this is not correct!
}

static void ICACHE_FLASH_ATTR
tcp_server_send(void *arg)
{
  struct netconn *conn = arg;
  uint8 buf[256];
  err_t err = ERR_OK;

  printf("TCP: send task starting\n");
  do {
    uint16 count = UART0_read(buf, 256, 1000); // wait up to one second
    if (count > 0) {
      err = netconn_write(conn, buf, count, NETCONN_COPY);
    }
  } while (err == ERR_OK);
  netconn_close(conn);
}

/** The main function, never returns! */
static void ICACHE_FLASH_ATTR
tcp_server_thread(void *arg)
{
  struct netconn *conn, *newconn;
  struct ip_addr addr;
  err_t err;
  LWIP_UNUSED_ARG(arg);
  printf("TCP: listener task starting\n");
  //vTaskDelay(20*1000/portTICK_RATE_MS);

  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  LWIP_ERROR("TCP: invalid conn", (conn != NULL), return);
  err = netconn_bind(conn, NULL, 23);
  LWIP_ERROR("TCP: cannot bind to port", (err == ERR_OK), return);

  /* Put the connection into LISTEN state */
  err = netconn_listen(conn);
  LWIP_ERROR("TCP: cannot listen", (err == ERR_OK), return);
  printf("TCP: listening on port 23\n");

  do {
    err = netconn_accept(conn, &newconn);
    if (err == ERR_OK) {
      printf("TCP: accept\n");
      u16_t port;
      netconn_getaddr(newconn, &addr, &port, 0);
      printf("TCP: new connection from %d.%d.%d.%d:%d\n", ip4_addr1(&addr), ip4_addr2(&addr),
          ip4_addr3(&addr), ip4_addr4(&addr), port);
      xTaskCreate(tcp_server_recv, "tcp-receiver", 4*configMINIMAL_STACK_SIZE, newconn, 2, NULL);
      xTaskCreate(tcp_server_send, "tcp-sender", 4*configMINIMAL_STACK_SIZE, newconn, 2, NULL);
    }
  } while(err == ERR_OK);
  printf("TCP: netconn_accept error %d, shutting down", err);
  netconn_close(conn);
  netconn_delete(conn);
}

/** Initialize the TCP server (start its thread) */
void ICACHE_FLASH_ATTR
tcp_server_init()
{
  printf("Creating tcp-server task\n");
  xTaskCreate(tcp_server_thread, "tcp-server", 4*configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  printf("Created\n");
}
