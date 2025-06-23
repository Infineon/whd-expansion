 /*
 * Test application for IMXRT1170 + H1A1 to join and ping an AP
 */
#if defined(WHD_CUSTOM_HAL) && defined(IMXRT)

#include <stdio.h>
#include <string.h>
#include "clock_config.h"
#include "board.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_sdio.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cyhal_gpio.h"
#include "cyhal_sdio.h"
#include "cyabs_rtos.h"
#include "whd_network_mw_core.h"
#include "whd_custom_hal_sdio.h"

/* Enable this for Soft AP bring up sample code */
#define SAP

#define WIFI_TASK_PRIO (3)
#define WIFI_TASK_STACK_SIZE (1024*4)

TaskHandle_t wifi_task_handler;
__attribute__((aligned(8))) uint8_t wifi_app_stack[WIFI_TASK_STACK_SIZE] = {0};

cy_thread_t app_thread;

static cyhal_sdio_t sdio_obj;
cyhal_sdio_t* obj = &sdio_obj;

whd_driver_t whd_driver;

whd_interface_t whd_ifs[2];

#ifdef SAP
typedef enum
{
    WHD_BAND_5GHZ   = 0xC0,    /**< 5-GHz radio band.   */
    WHD_BAND_2_4GHZ = 0x00,    /**< 2.4-GHz radio band. */
    WHD_BAND_6GHZ   = 0x80     /**< 6-GHz radio band.   */
} whd_band_t;

#define SOFTAP_SSID                                  "AP_SSID"
#define SOFTAP_PASSWORD                              "AP_PASSWORD"
#define SOFTAP_SECURITY_TYPE                         WHD_SECURITY_WPA2_AES_PSK
#define SOFTAP_CHANNEL                               36
#define SOFTAP_BAND                                  WHD_BAND_5GHZ
#define SOFTAP_IP_ADDRESS                            MAKE_IPV4_ADDRESS(192, 168, 0,  2)
#define SOFTAP_NETMASK                               MAKE_IPV4_ADDRESS(255, 255, 255, 0)
#define SOFTAP_GATEWAY                               MAKE_IPV4_ADDRESS(192, 168, 0,  2)

static const whd_network_static_ip_addr_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .addr, SOFTAP_IP_ADDRESS),
    INITIALISER_IPV4_ADDRESS( .netmask,    SOFTAP_NETMASK),
    INITIALISER_IPV4_ADDRESS( .gateway,    SOFTAP_GATEWAY),
};
#else
#define WIFI_SSID                                   "WIFI_SSID"
#define WIFI_PASSWORD                               "WIFI_PASSWORD"
#define WIFI_SECURITY                               WHD_SECURITY_WPA2_AES_PSK
#endif

#ifdef SAP
cy_rslt_t StartSoftAP()
{
    cy_rslt_t res = CY_RSLT_SUCCESS ;
    whd_nw_ip_address_t ipv4_addr;
    char ip_str[IP_STR_LEN];
    whd_ssid_t ssid;
    uint8_t *key = SOFTAP_PASSWORD;
    uint8_t keylen = strlen(SOFTAP_PASSWORD);
    whd_security_t ap_security = SOFTAP_SECURITY_TYPE;
    uint8_t channel = SOFTAP_CHANNEL;
    uint16_t chanspec = SOFTAP_BAND;
    whd_network_static_ip_addr_t static_ip;
    whd_network_interface_context *nw_if_ctx;

    memset(&ipv4_addr, 0, sizeof(whd_nw_ip_address_t));
    memset(ip_str, 0, sizeof(ip_str));
    memset(&static_ip, 0, sizeof(whd_network_static_ip_addr_t));

    ssid.length = strlen(SOFTAP_SSID);
    memcpy(ssid.value, SOFTAP_SSID, ssid.length + 1);

    static_ip.gateway.ip.v4 = ap_ip_settings.gateway.ip.v4;
    static_ip.addr.ip.v4    = ap_ip_settings.addr.ip.v4;
    static_ip.netmask.ip.v4 = ap_ip_settings.netmask.ip.v4;

    /* Add channel info */
    chanspec = ((chanspec << 8) | channel);

    /* set up the AP info */
    res = whd_wifi_init_ap(whd_ifs[CY_NET_INTERFACE_TYPE_AP], &ssid, ap_security, (const uint8_t *)key,
                          keylen, chanspec);

    if (res != CY_RSLT_SUCCESS)
    {
        if(res == WHD_UNSUPPORTED || res == WHD_WEP_NOT_ALLOWED)
        {
            return res;
        }
    }

    PRINTF("\n***** Starting '%s' AP ***** \n", SOFTAP_SSID);

    res = whd_wifi_start_ap(whd_ifs[CY_NET_INTERFACE_TYPE_AP]);
    if (res == CY_RSLT_SUCCESS)
    {
        res = whd_network_up(whd_ifs[CY_NET_INTERFACE_TYPE_AP], CY_NET_INTERFACE_TYPE_AP, &static_ip, &nw_if_ctx);
    }
    else
    {
        PRINTF("Start AP failed %ld\n" ,res);
        whd_wifi_stop_ap(whd_ifs[CY_NET_INTERFACE_TYPE_AP]);
        return res;
    }
    if(res == CY_RSLT_SUCCESS)
    {
        /* Get IPV4 address for AP */
        res = whd_network_get_ip_address(nw_if_ctx, &ipv4_addr);
    }
    else
    {
         PRINTF("Failed to bring up AP network\n");
         return res;
    }
    if(res == CY_RSLT_SUCCESS)
    {
        whd_nw_ntoa(&ipv4_addr, ip_str);
        PRINTF("IP Address %s assigned\n", ip_str);
    }
    else
    {
        PRINTF("\nFailed to get the IP address\n");
        return res;
    }
    return res;
}
#else
cy_rslt_t wifi_connect(void)
{
    whd_network_static_ip_addr_t static_ip;
    whd_nw_ip_address_t gateway_ipv4_addr;
    whd_nw_ip_address_t ipv4_addr;
    whd_network_interface_context *nw_if_ctx;
    whd_net_ip_address_t gateway_addr;
    uint32_t elapsed_time_ms;

    char ip_str[15];
    whd_ssid_t ssid;
    uint8_t *key = WIFI_PASSWORD;
    uint8_t keylen = strlen(WIFI_PASSWORD);
    whd_security_t security = WIFI_SECURITY;

    memset(ip_str, 0, sizeof(ip_str));

    ssid.length = strlen(WIFI_SSID);
    memcpy(ssid.value, WIFI_SSID, ssid.length + 1);

    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = whd_wifi_join(whd_ifs[CY_NET_INTERFACE_TYPE_STA] , &ssid, security, key, keylen);

    if(result == CY_RSLT_SUCCESS)
    {
        PRINTF("\nJoined %s successfully\n", ssid.value);
        cy_rtos_delay_milliseconds(2000);
        result = whd_network_ip_assign(whd_ifs[CY_NET_INTERFACE_TYPE_STA], &nw_if_ctx);
    }
    else
    {
        PRINTF("Failed to join %s\n", ssid.value);
        return result;
    }
    if(result == CY_RSLT_SUCCESS)
    {
        /* Get IPV4 address for STA */
        result = whd_network_get_ip_address(nw_if_ctx, &ipv4_addr);
    }
    else
    {
         PRINTF("Failed to gain IP address\n");
         return result;
    }
    if(result == CY_RSLT_SUCCESS)
    {
        whd_nw_ntoa(&ipv4_addr, ip_str);
        PRINTF("IP Address %s assigned\n", ip_str);
        result = whd_network_get_gateway_ip_address(nw_if_ctx, &gateway_ipv4_addr);
    }
    else
    {
        PRINTF("Failed to fetch IP address\n");
        return result;
    }
    if (result != CY_RSLT_SUCCESS)
    {
        PRINTF("Failed to get Gateway IP address: %ld \n", result);
    }
    else
    {
        whd_nw_ntoa(&gateway_ipv4_addr, ip_str);
        PRINTF("Gateway Address %s assigned\n", ip_str);
    }

    while(true)
    {
        result = whd_network_ping((void *)nw_if_ctx, &gateway_ipv4_addr, 3000, &elapsed_time_ms);
        if (result == CY_RSLT_SUCCESS)
        {
            PRINTF("Ping Successful. Time elapsed = %lu ms\n", elapsed_time_ms);
        }
        else
            PRINTF("Ping failed %ld\n", result);
    }
}
#endif

static void wifi_task(void *arg)
{
    PRINTF("====================================================\n");

    cy_rslt_t result = CY_RSLT_SUCCESS;

    whd_network_init();

    result = whd_custom_wifi_init(
#ifdef SAP
    &whd_ifs[CY_NET_INTERFACE_TYPE_AP]
#else
    &whd_ifs[CY_NET_INTERFACE_TYPE_STA]
#endif
    );

    if(result == CY_RSLT_SUCCESS)
    {
        PRINTF("\nIMXRT WIFI ON\n");
#ifdef SAP
        StartSoftAP();
        while(1)
        {
            vTaskDelay(500);
        }
#else
        wifi_connect();
#endif
    }
    else
        PRINTF("wifi init failed %ld\n", result);
}

int main(void)
{
    cy_rslt_t result;
    (void)result;
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitPinsM2();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    PRINTF("====================================================\n");
    PRINTF("\t\tIMXRT1170 H1 A1 Test\n");
    cy_rtos_create_thread(&app_thread, wifi_task, "App_Task", wifi_app_stack, WIFI_TASK_STACK_SIZE, CY_RTOS_PRIORITY_BELOWNORMAL, NULL);

    vTaskStartScheduler();

     return 0;
}
#endif /* WHD_CUSTOM_HAL && IMXRT */
