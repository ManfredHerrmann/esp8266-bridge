#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "uart.h"

// setup gpios
#define LED2_GPIO 2
#define LED2_GPIO_MUX PERIPHS_IO_MUX_GPIO2_U
#define LED2_GPIO_FUNC FUNC_GPIO2

#define LED0_GPIO 0
#define LED0_GPIO_MUX PERIPHS_IO_MUX_GPIO2_U
#define LED0_GPIO_FUNC FUNC_GPIO2

#ifndef GPIO_OUTPUT_SET
#define GPIO_OUTPUT_SET(gpio_no, bit_value) \
        gpio_output_set(bit_value<<gpio_no, ((~bit_value)&0x01)<<gpio_no, 1<<gpio_no,0)
#endif

/*
// task to blink gpio2
void blinky2(void *pvParameters)
{
    const portTickType xDelay = 250 / portTICK_RATE_MS;
    static uint8_t state=0;
    PIN_FUNC_SELECT(LED2_GPIO_MUX, LED2_GPIO_FUNC);
    for(;;)
    {
        // set then invert state
        GPIO_OUTPUT_SET(LED2_GPIO, state);
        state ^=1;

        // pause
        vTaskDelay( xDelay);
    }
}
*/

// task to blink gpio0
void blinky0(void *pvParameters)
{
    const portTickType xDelay = 250 / portTICK_RATE_MS;
    static uint8_t state=0;
    PIN_FUNC_SELECT(LED0_GPIO_MUX, LED0_GPIO_FUNC);
    for(;;)
    {
        // set then invert state
        GPIO_OUTPUT_SET(LED0_GPIO, state);
        state ^=1;

        // pause
        vTaskDelay( xDelay);
    }
}

void ICACHE_FLASH_ATTR
uart1_init(void)
{
    UART_WaitTxFifoEmpty(UART1);

    UART_ConfigTypeDef uart_config;
    uart_config.baud_rate    = BIT_RATE_74880;
    uart_config.data_bits    = UART_WordLength_8b;
    uart_config.parity       = USART_Parity_None;
    uart_config.stop_bits    = USART_StopBits_1;
    uart_config.flow_ctrl    = USART_HardwareFlowControl_None;
    uart_config.UART_RxFlowThresh = 120;
    uart_config.UART_InverseMask = UART_None_Inverse;
    UART_ParamConfig(UART1, &uart_config);

    UART_IntrConfTypeDef uart_intr;
    uart_intr.UART_IntrEnMask = 0;
    uart_intr.UART_RX_FifoFullIntrThresh = 10;
    uart_intr.UART_RX_TimeOutIntrThresh = 2;
    uart_intr.UART_TX_FifoEmptyIntrThresh = 20;
    UART_IntrConfig(UART1, &uart_intr);

    UART_SetPrintPort(UART1);
    //UART_intr_handler_register(uart0_rx_intr_handler);
    //ETS_UART_INTR_ENABLE();

}

// user code
void ICACHE_FLASH_ATTR
user_init(void)
{
    printf("Switching to UART1\n");
    uart1_init();
    printf("\n\nHello World!\n\n");

    // setup wifi mode
    wifi_set_opmode(STATION_MODE);

    // wifi config
    {
        struct station_config *config = (struct station_config *)zalloc(sizeof(struct station_config));
        sprintf(config->ssid, "tve-home");
        sprintf(config->password, "foobar");
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

    // create tasks
    //xTaskCreate(blinky2, "bl2", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    //xTaskCreate(blinky0, "bl0", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    tcp_server_init();
}
