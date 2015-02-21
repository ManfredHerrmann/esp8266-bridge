#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"

#include "server.h"

#define HTTPD_DEBUG         LWIP_DBG_ON

static void
tcp_server_serve(struct netconn *conn)
{
  struct netbuf *inbuf;
  char *buf;
  u16_t buflen;
  err_t err;

  /* Read the data from the port, blocking if nothing yet there.
   We assume the request (the part we care about) is in one netbuf */
  err = netconn_recv(conn, &inbuf);

  while (err == ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);
    netconn_write(conn, buf, buflen, NETCONN_COPY);
    /* Delete the buffer (netconn_recv gives us ownership,
     so we have to make sure to deallocate the buffer) */
    netbuf_delete(inbuf);
    err = netconn_recv(conn, &inbuf);
  }

  /* Close the connection (server closes in HTTP) */
  netconn_close(conn);
}

/** The main function, never returns! */
static void
tcp_server_thread(void *arg)
{
  struct netconn *conn, *newconn;
  err_t err;
  LWIP_UNUSED_ARG(arg);

  printf("Server thread started\n");
  LWIP_DEBUGF(HTTPD_DEBUG, ("LWIP server thread started\n"));

  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  LWIP_ERROR("tcp_server: invalid conn", (conn != NULL), return;);

  /* Bind to port 80 (HTTP) with default IP address */
  netconn_bind(conn, NULL, 23);

  /* Put the connection into LISTEN state */
  netconn_listen(conn);

  do {
    err = netconn_accept(conn, &newconn);
    if (err == ERR_OK) {
      tcp_server_serve(newconn);
      netconn_delete(newconn);
    }
  } while(err == ERR_OK);
  LWIP_DEBUGF(HTTPD_DEBUG,
    ("tcp_server_thread: netconn_accept received error %d, shutting down",
    err));
  netconn_close(conn);
  netconn_delete(conn);
}

/** Initialize the TCP server (start its thread) */
void
tcp_server_init()
{
  printf("Creating tcp-server task\n");
  xTaskCreate(tcp_server_thread, "tcp-server", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}
