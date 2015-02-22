#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "uart.h"
#include "gpio.h"
#include "config.h"

void ICACHE_FLASH_ATTR
reset_arduino(void)
{
  printf("Resetting Arduino\n");
  vTaskDelay(1);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); // steal GPIO2 from uart_tx
  GPIO_AS_OUTPUT(GPIO_Pin_0|GPIO_Pin_2); // set both pins as output
  GPIO_OUTPUT(GPIO_Pin_0|GPIO_Pin_2, 0); // clear both pins, starts reset
  vTaskDelay(1);                         // 1 tick delay, will result in 10ms pulse?
  GPIO_OUTPUT(GPIO_Pin_0, 1);            // end reset, continue ISP
  vTaskDelay(1);                         // 1 tick delay, will result in 10ms pulse?
  GPIO_OUTPUT(GPIO_Pin_2, 1);            // end ISP for ARM, AVR doesn't care
  vTaskDelay(1);                         // 1 tick delay, will result in 10ms pulse?
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK); // switch GPIO2 back to uart1_tx

}

void ICACHE_FLASH_ATTR
show_status(void *arg)
{
    for (;;) {
      printf("Free heap size=%u\r\n", system_get_free_heap_size());
      vTaskDelay(10*1000 / portTICK_RATE_MS);
    }
}

// user code
void ICACHE_FLASH_ATTR
user_init(void)
{
    printf("Switching to UART1\n");
    UART1_init();
    printf("\n\nHello World!\n\n");
    UART0_init(BIT_RATE_115200);

    // setup wifi mode
    wifi_set_opmode(STATION_MODE);

    // wifi config
    {
        struct station_config *config =
          (struct station_config *)zalloc(sizeof(struct station_config));
        sprintf(config->ssid, WIFI_SSID);
        sprintf(config->password, WIFI_PWD);
        wifi_station_set_config(config);
        free(config);
    }

#ifdef STATIC_IP
    // static ip config
    {
        struct ip_info ipinfo;
        ipinfo.gw.addr = ipaddr_addr("192.168.1.1");
        ipinfo.ip.addr = ipaddr_addr("192.168.1.2");
        ipinfo.netmask.addr = ipaddr_addr("255.255.255.0");
        //wifi_set_ip_info(STATION_IF, &ipinfo);
    }
#endif


    // print system info
    printf("System Info:\r\n");
    printf("Time=%u\r\n", system_get_time());
    printf("RTC time=%u\r\n", system_get_rtc_time());
    printf("Chip id=0x%x\r\n", system_get_chip_id());
    printf("Free heap size=%u\r\n", system_get_free_heap_size());
    printf("Mem info:\r\n");
    system_print_meminfo();
    printf("\r\n");
    printf("SDK version:%d.%d.%d\n", SDK_VERSION_MAJOR, SDK_VERSION_MINOR, SDK_VERSION_REVISION);

    tcp_server_init();
    xTaskCreate(show_status, "show-status", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}
