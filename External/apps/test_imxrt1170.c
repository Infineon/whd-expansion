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
#include "cy_network_mw_core.h"
#include "whd_custom_hal_sdio.h"
#include "wpa3_wcm_intf.h"
#ifdef WHD_USE_WCM
#include "cy_wcm.h"
#endif /* WHD_USE_WCM */

/* Enable this for Soft AP bring up sample code */
//#define SAP

#define WIFI_TASK_PRIO (3)
#define WIFI_TASK_STACK_SIZE (1024*10)
/* The interface count needs to match with WCM */
#define MAX_WHD_INTERFACES                           2

TaskHandle_t wifi_task_handler;
__attribute__((aligned(8))) uint8_t wifi_app_stack[WIFI_TASK_STACK_SIZE] = {0};

cy_thread_t app_thread;

static cyhal_sdio_t sdio_obj;
cyhal_sdio_t* obj = &sdio_obj;

whd_driver_t whd_driver;

extern whd_interface_t whd_ifs[MAX_WHD_INTERFACES];

#ifdef WHD_USE_WCM
/* wcm parameters */
static cy_wcm_config_t wcm_config;
static cy_wcm_connect_params_t conn_params;
#endif

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
#define SOFTAP_CHANNEL                               11
#ifdef WHD_USE_WCM
#define SOFTAP_BAND                                  CY_WCM_WIFI_BAND_2_4GHZ
#else
#define SOFTAP_BAND                                  WHD_BAND_2_4GHZ
#endif /* WHD_USE_WCM */
#define SOFTAP_IP_ADDRESS                            MAKE_IPV4_ADDRESS(192, 168, 0,  2)
#define SOFTAP_NETMASK                               MAKE_IPV4_ADDRESS(255, 255, 255, 0)
#define SOFTAP_GATEWAY                               MAKE_IPV4_ADDRESS(192, 168, 0,  2)
#ifdef WHD_USE_WCM
static const cy_wcm_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, SOFTAP_IP_ADDRESS),
    INITIALISER_IPV4_ADDRESS( .netmask,    SOFTAP_NETMASK),
    INITIALISER_IPV4_ADDRESS( .gateway,    SOFTAP_GATEWAY),
};
#else
static const whd_network_static_ip_addr_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .addr, SOFTAP_IP_ADDRESS),
    INITIALISER_IPV4_ADDRESS( .netmask,    SOFTAP_NETMASK),
    INITIALISER_IPV4_ADDRESS( .gateway,    SOFTAP_GATEWAY),
};
#endif /* WHD_USE_WCM */
#else
#define WIFI_SSID                                   "WIFI_SSID"
#define WIFI_PASSWORD                               "WIFI_PASSWORD"
#ifdef WHD_USE_WCM
#define WIFI_SECURITY                               CY_WCM_SECURITY_WPA2_AES_PSK
#else
#define WIFI_SECURITY                               WHD_SECURITY_WPA2_AES_PSK
#endif /* WHD_USE_WCM */
#define CONNECT_RETRY_COUNT                         5
#ifdef WHD_USE_WCM
#define WIFI_BAND                                   CY_WCM_WIFI_BAND_ANY
static void get_ip_string(char* buffer, uint32_t ip)
{
    sprintf(buffer, "%lu.%lu.%lu.%lu",
            (unsigned long)(ip      ) & 0xFF,
            (unsigned long)(ip >>  8) & 0xFF,
            (unsigned long)(ip >> 16) & 0xFF,
            (unsigned long)(ip >> 24) & 0xFF);
}
#endif /* WHD_USE_WCM */
#endif /* SAP */

#ifdef SAP
cy_rslt_t StartSoftAP()
{
#ifdef WHD_USE_WCM
    cy_rslt_t result = CY_RSLT_SUCCESS ;
    cy_wcm_ap_config_t ap_conf;
    cy_wcm_ip_address_t ip_addr;
    uint32_t elapsed_time_ms;
    char ipstr[IP_STR_LEN];
    memset(&ap_conf, 0, sizeof(cy_wcm_ap_config_t));
    memset(&ip_addr, 0, sizeof(cy_wcm_ip_address_t));

    ap_conf.channel = SOFTAP_CHANNEL;
    ap_conf.band = SOFTAP_BAND;
    memcpy(ap_conf.ap_credentials.SSID, SOFTAP_SSID, strlen(SOFTAP_SSID) + 1);
    memcpy(ap_conf.ap_credentials.password, SOFTAP_PASSWORD, strlen(SOFTAP_PASSWORD) + 1);
    ap_conf.ap_credentials.security = SOFTAP_SECURITY_TYPE;
    ap_conf.ip_settings.ip_address = ap_ip_settings.ip_address;
    ap_conf.ip_settings.netmask = ap_ip_settings.netmask;
    ap_conf.ip_settings.gateway = ap_ip_settings.gateway;

    PRINTF("\n***** Starting '%s' AP ***** \n", ap_conf.ap_credentials.SSID);

    result = cy_wcm_start_ap(&ap_conf);

    if (result == CY_RSLT_SUCCESS)
    {
        PRINTF("%s Started\n", ap_conf.ap_credentials.SSID);
    }
#else
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
#endif /* WHD_USE_WCM */
}
#else
cy_rslt_t wifi_connect(void)
{
#ifdef WHD_USE_WCM
    cy_rslt_t res ;

    const char *ssid = WIFI_SSID;
    const char *key = WIFI_PASSWORD;
    cy_wcm_wifi_band_t band = WIFI_BAND;
    cy_wcm_ip_address_t ip_addr;
    cy_wcm_ip_address_t gateway_ip_addr;
    char ipstr[IP_STR_LEN];
    uint32_t elapsed_time_ms;
    whd_network_interface_context *nw_if_ctx;

    memset(&conn_params, 0, sizeof(cy_wcm_connect_params_t));

    memcpy(&conn_params.ap_credentials.SSID, ssid, strlen(ssid) + 1);
    memcpy(&conn_params.ap_credentials.password, key, strlen(key) + 1);
    conn_params.ap_credentials.security = WIFI_SECURITY;
    conn_params.band = band;

    PRINTF("Attempting Connection\n");
    for(int i=1; i<= CONNECT_RETRY_COUNT; i++)
        {
            PRINTF("Connecting to %s, Attempt %d/%d\n", WIFI_SSID, i, CONNECT_RETRY_COUNT);
            res = cy_wcm_connect_ap(&conn_params, &ip_addr);
            if(res == CY_RSLT_SUCCESS)
                break;
            else if(i!=CONNECT_RETRY_COUNT)
            {
                PRINTF("Join failed, retrying...\n");
            }
            else
            {
                PRINTF("Connect failed, exceeded maximum retries\n");
            }
        }

    if(!res)
    {
        PRINTF("Successfully joined wifi network '%s , result = %ld'\n", ssid, (long)res);
        get_ip_string(ipstr, ip_addr.ip.v4);
        PRINTF("IP Address %s assigned\n", ipstr);

        res = cy_wcm_get_gateway_ip_address(CY_WCM_INTERFACE_TYPE_STA, &gateway_ip_addr);
        if(!res)
        {
            get_ip_string(ipstr, gateway_ip_addr.ip.v4);
            PRINTF("Gateway IP Address %s assigned\n", ipstr);
            while(true)
            {
                res = cy_wcm_ping(CY_WCM_INTERFACE_TYPE_STA, &gateway_ip_addr, 3000, &elapsed_time_ms);
                if (res == CY_RSLT_SUCCESS)
                {
                    PRINTF("Ping Successful. Time elapsed = %lu ms\n", elapsed_time_ms);
                }
                    else
                        PRINTF("Ping failed %ld\n", res);
            }
        }
    }

#else
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

    for(int i=1; i<= CONNECT_RETRY_COUNT; i++)
        {
            PRINTF("Connecting to %s, Attempt %d/%d\n", ssid.value, i, CONNECT_RETRY_COUNT);
            if(security == WHD_SECURITY_WPA3_SAE)
            {
                result = wpa3_supplicant_sae_start(ssid.value, ssid.length, key, keylen);
                if(result != CY_RSLT_SUCCESS)
                {
                    PRINTF("SAE Failed\n");
                    continue;
                }
                else
                {
                    PRINTF("SAE successful\n");
                }
            }

            result = whd_wifi_join(whd_ifs[CY_NET_INTERFACE_TYPE_STA] , &ssid, security, key, keylen);
            if(result == CY_RSLT_SUCCESS)
                break;
            else if(i!=CONNECT_RETRY_COUNT)
            {
                PRINTF("Join failed, retrying...\n");
            }
            else
            {
                PRINTF("Connect failed, exceeded maximum retries\n");
            }
        }

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
#endif /* WHD_USE_WCM */
}
#endif /* SAP */

static void wifi_task(void *arg)
{
    PRINTF("====================================================\n");

    cy_rslt_t result = CY_RSLT_SUCCESS;

#ifdef WHD_USE_WCM
#ifdef SAP
    wcm_config.interface = CY_WCM_INTERFACE_TYPE_AP_STA;
#else
    wcm_config.interface = CY_WCM_INTERFACE_TYPE_STA;
#endif /* SAP */
    result = cy_wcm_init(&wcm_config);
#else
    whd_network_init();
    result = whd_custom_wifi_init(
#ifdef SAP
    &whd_ifs[CY_NET_INTERFACE_TYPE_AP]
#else
    &whd_ifs[CY_NET_INTERFACE_TYPE_STA]
#endif /* SAP */
    );
#endif /* WHD_USE_WCM */
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
#endif /* SAP */
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
