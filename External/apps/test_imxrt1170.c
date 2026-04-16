 /*
 * Command Console test application for IMXRT1170 + H1A1 to run Wifi / Cert / QT commands
 */
#if defined(IMXRT)

#include <stdio.h>
#include <string.h>
#include "clock_config.h"
#include "board.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_sdio.h"
#include "fsl_debug_console.h"
#include "cybsp_wifi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cyhal_gpio.h"
#include "cyhal_sdio.h"
#include "cyabs_rtos.h"
#include "whd_network_mw_core.h"
#include "whd_custom_hal_sdio.h"
#include "wpa3_wcm_intf.h"
#ifdef WHD_USE_WCM
#include "cy_wcm.h"
//#include "iperf_utility.h"
#ifdef ENABLE_SIGMA_TOOL
#include "wifi_app_intf.h"
#endif /* ENABLE_SIGMA_TOOL */
#endif /* WHD_USE_WCM */
#include "command_console.h"
#include "wifi_utility.h"

/* Default configurations for Command Console library */
#define CONSOLE_COMMAND_MAX_PARAMS     (32)
#define CONSOLE_COMMAND_MAX_LENGTH     (256) //Increased from 85 to 256 for Sigma
#define CONSOLE_COMMAND_HISTORY_LENGTH (10)
#define CONSOLE_THREAD_PRIORITY        (2)

const char *console_delimiter_string = ","; //Use comma instead of space for Sigma

static char command_buffer[CONSOLE_COMMAND_MAX_LENGTH];
static char command_history_buffer[CONSOLE_COMMAND_MAX_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];

/* Task parameters for Console task */
#define CONSOLE_TASK_PRIORITY          (1)
#define CONSOLE_TASK_STACK_SIZE        (1024 * 4)

/* The interface count needs to match with WCM */
#define MAX_WHD_INTERFACES             (2)

#ifdef WHD_USE_WCM
extern whd_interface_t whd_ifs[MAX_WHD_INTERFACES];
static cy_wcm_config_t wcm_config = {.interface = CY_WCM_INTERFACE_TYPE_AP_STA};
#else
whd_interface_t whd_ifs[MAX_WHD_INTERFACES];
whd_net_interface_t whd_interface_type = CY_NET_INTERFACE_TYPE_STA;
#endif /* WHD_USE_WCM */

cy_rslt_t command_console_add_command();

cy_rslt_t command_console_add_command(void)
{
    cy_command_console_cfg_t console_cfg;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    PRINTF("executing command_console_add_remove_command \n");
    console_cfg.serial = (void *)NULL;
    console_cfg.line_len = sizeof(command_buffer);
    console_cfg.buffer = command_buffer;
    console_cfg.history_len = CONSOLE_COMMAND_HISTORY_LENGTH;
    console_cfg.history_buffer_ptr = command_history_buffer;
    console_cfg.delimiter_string = console_delimiter_string;
    console_cfg.params_num = CONSOLE_COMMAND_MAX_PARAMS;
    console_cfg.thread_priority = CONSOLE_THREAD_PRIORITY;

    /* Initialize command console library */
    result = cy_command_console_init(&console_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        PRINTF("Error in initializing command console library : %ld \n", (long)result);
		return result;
    }

	/* Initialize Wi-Fi utility and add Wi-Fi commands */
    result = wifi_utility_init();
    if ( result != CY_RSLT_SUCCESS )
    {
        PRINTF("Error in initializing Wi-Fi utility : %ld \n", (long)result);
        return result;
    }

#ifdef WHD_USE_WCM
#ifdef ENABLE_SIGMA_TOOL
    /* Initialize SigmaDUT library */
    result = cywifi_init_sigmadut();
    if ( result != CY_RSLT_SUCCESS )
    {
       PRINTF("Error in initializing Sigma library : %ld \n", (long)result);
    }
#endif /* ENABLE_SIGMA_TOOL */

    /* Initialize IPERF utility and add IPERF commands */
	//iperf_utility_init(&wcm_config.interface);
#endif /* WHD_USE_WCM */

    return CY_RSLT_SUCCESS;
}

void console_task(void *param)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

#ifdef WHD_USE_WCM
	result = cy_wcm_init(&wcm_config);
    if (CY_RSLT_SUCCESS != result)
    {
        PRINTF("WCM initialization failed !! Result: %ld(0x%lx)\n", result, result);
		return;
    }
#else
	whd_network_init();
	if(whd_interface_type != CY_NET_INTERFACE_TYPE_AP_STA)
    {
	    if (CY_RSLT_SUCCESS != whd_custom_wifi_init(&whd_ifs[whd_interface_type]))
        {
            PRINTF("Failed to initialize Wi-Fi module !!\n");
            return;
        }
	}
    else
    {
		/* In concurrent mode primary interface would be STA and secondary AP*/
        if (CY_RSLT_SUCCESS != whd_custom_wifi_init(&whd_ifs[CY_NET_INTERFACE_TYPE_AP_STA]))
        {
            PRINTF("Error in initializing primary interface !!\n");
            return;
        }

        /* SoftAP is the secondary interface in Concurrent mode*/
        if (CY_RSLT_SUCCESS != whd_custom_wifi_init_secondary(&whd_ifs[CY_NET_INTERFACE_TYPE_AP]))
        {
            PRINTF("Error in initializing secondary interface !!\n");
            return;
        }
    }
#endif /* WHD_USE_WCM */
    command_console_add_command();

    while (1)
    {
        vTaskDelay(500);
    }
}

TaskHandle_t console_task_handler;
__attribute__((aligned(8))) uint8_t console_app_stack[CONSOLE_TASK_STACK_SIZE] = {0};
cy_thread_t app_thread;

int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitPinsM2();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    PRINTF("====================================================\n");
    PRINTF("\tIMXRT1170 H1A1 Wifi/Cert Test App\n");
    PRINTF("====================================================\n");

    xTaskCreate(console_task, "console", CONSOLE_TASK_STACK_SIZE, "console", CONSOLE_TASK_PRIORITY, NULL);
	/* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

	return 0;
}

#endif /* defined(IMXRT) */
