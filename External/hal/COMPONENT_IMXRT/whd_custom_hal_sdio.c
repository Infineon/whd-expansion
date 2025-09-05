/*
 * Copyright 2025, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 *
 * This file will come into picture for 3rd party platforms, for now this is limited to IMXRT.
 * Platform specfic HAL functions are defined/modified here
 *
 */

#if defined(WHD_CUSTOM_HAL) && defined(IMXRT)

#include "cyhal_sdio.h"
#include "fsl_sdio.h"
#include "whd.h"
#include "wifi_bt_config.h"
#include "whd_custom_hal_sdio.h"
#include "fsl_sdmmc_common.h"
#include "fsl_sdmmc_spec.h"

#define MINIMUM_WHD_STACK_SIZE        (1024 + 1200 + 2500)

#if !defined(CY_WIFI_OOB_INTR_PRIORITY)
    #define CY_WIFI_OOB_INTR_PRIORITY       (2)
#endif

#define CY_WIFI_HOST_WAKE_GPIO      0xFFFFFFFF
#define CY_WIFI_HOST_WAKE_IRQ_EVENT 0

static cyhal_sdio_t *cy_imx_sdio;

static sdio_block_size_t find_optimal_block_size(uint32_t);

void whd_custom_sdio_callback(void *);

static whd_driver_t whd_driv;
extern whd_resource_source_t resource_ops;
extern whd_result_t whd_get_host_buffer(whd_buffer_t* buffer, whd_buffer_dir_t direction,
                                uint16_t size, uint32_t timeout_ms);
extern void whd_host_buffer_release(whd_buffer_t buffer, whd_buffer_dir_t direction);
extern uint8_t* whd_host_buffer_get_current_piece_data_pointer(whd_buffer_t buffer);
extern uint16_t whd_host_buffer_get_current_piece_size(whd_buffer_t buffer);
extern whd_result_t whd_host_buffer_set_size(whd_buffer_t buffer, uint16_t size);
extern whd_result_t whd_host_buffer_add_remove_at_front(whd_buffer_t* buffer, int32_t add_remove_amount);
extern void whd_host_network_process_ethernet_data(whd_interface_t interface, whd_buffer_t buffer);

static whd_init_config_t init_config_default =
{
    .thread_stack_size  = MINIMUM_WHD_STACK_SIZE + 2048,
    .thread_stack_start = NULL,
    .thread_priority    = (uint32_t)3,
    .country            = WHD_COUNTRY_UNITED_STATES
};

static whd_buffer_funcs_t buffer_if_default =
{
    .whd_host_buffer_get                       = whd_get_host_buffer,
    .whd_buffer_release                        = whd_host_buffer_release,
    .whd_buffer_get_current_piece_data_pointer = whd_host_buffer_get_current_piece_data_pointer,
    .whd_buffer_get_current_piece_size         = whd_host_buffer_get_current_piece_size,
    .whd_buffer_set_size                       = whd_host_buffer_set_size,
    .whd_buffer_add_remove_at_front            = whd_host_buffer_add_remove_at_front,
};

static whd_netif_funcs_t netif_if_default =
{
    .whd_network_process_ethernet_data = whd_host_network_process_ethernet_data,
};

#if !defined(COMPONENT_WIFI_INTERFACE_M2M)
static const whd_oob_config_t OOB_CONFIG =
{
    .host_oob_pin      = CY_WIFI_HOST_WAKE_GPIO,
    .dev_gpio_sel      = 0,
    .is_falling_edge   = (CY_WIFI_HOST_WAKE_IRQ_EVENT == 2)
        ? WHD_TRUE
        : WHD_FALSE,
    .intr_priority     = CY_WIFI_OOB_INTR_PRIORITY
};
#endif

static cyhal_sdio_t sdio_obj;

whd_result_t whd_custom_get_wifi_interface(whd_interface_type_t interface_type, whd_interface_t *iface)
{
    CHECK_IFP_NULL(whd_driv->iflist[interface_type]);
    *iface = whd_driv->iflist[interface_type];
    return WHD_SUCCESS;
}

whd_driver_t whd_custom_get_wifi_driver(void)
{
    return whd_driv;
}

whd_result_t whd_custom_wifi_init(whd_interface_t *interface)

{
	whd_result_t result;
	whd_sdio_config_t whd_sdio_config =
    {
        .sdio_1bit_mode 	   = WHD_FALSE,
        .high_speed_sdio_clock = WHD_FALSE,
        .oob_config 		   = OOB_CONFIG
    };

	result = whd_init(&whd_driv, &init_config_default, &resource_ops, &buffer_if_default, &netif_if_default);

    if(result == WHD_SUCCESS)
    {
        result = whd_custom_hal_sdio_init(&sdio_obj);
        if(result == WHD_SUCCESS)
        {
            result = whd_bus_sdio_attach(whd_driv, &whd_sdio_config, &sdio_obj);
            if(result == WHD_SUCCESS)
            {
                result = whd_wifi_on(whd_driv, interface);
                if(result != WHD_SUCCESS)
                {
                    whd_bus_sdio_detach(whd_driv);
                }
            }
        }
        if (result != WHD_SUCCESS)
        {
            whd_deinit(*interface);
        }
    }

	return result;
}

whd_result_t whd_custom_hal_sdio_init(cyhal_sdio_t *obj)
{
    whd_result_t result = WHD_SUCCESS;

    (void)memset(&obj->sd_card, 0, sizeof(sdio_card_t));

    cy_imx_sdio = obj;

    BOARD_WIFI_BT_Config(&obj->sd_card, whd_custom_sdio_callback);

    result = SDIO_HostInit(&obj->sd_card);
    if (result != kStatus_Success)
    {
        WPRINT_WHD_ERROR(("SDIO_HostInit failed %ld\n", result));
        return result;
    }

	vTaskDelay(pdMS_TO_TICKS(100));

	BOARD_WIFI_BT_Enable(WHD_TRUE);

    obj->sd_card.currentTiming = kSD_TimingSDR50Mode;

    result = SDIO_CardInit(&obj->sd_card);
	if(result != kStatus_Success)
    {
        WPRINT_WHD_ERROR(("SDIO_CardInit failed %ld\n", result));
        return result;
    }

	result = SDIO_EnableAsyncInterrupt(&obj->sd_card, WHD_TRUE);
    if(result != kStatus_Success)
    {
        WPRINT_WHD_ERROR(("SDIO_EnableAsyncInterrupt failed %ld\n", result));
        return result;
    }

	return result;
}

whd_result_t whd_custom_hal_sdio_send_cmd(const cyhal_sdio_t *obj, cyhal_transfer_t direction,
                              cyhal_sdio_command_t command, uint32_t argument, uint32_t *response)
{

    whd_result_t result = WHD_SUCCESS;

    sdmmchost_transfer_t content;
    sdmmchost_cmd_t xcommand;
    sdmmchost_data_t xdata;

    memset(&content, 0, sizeof(content));
    memset(&xcommand, 0, sizeof(xcommand));
    memset(&xdata, 0, sizeof(xdata));

    xcommand.index = command;
    xcommand.argument = argument;
    xcommand.responseErrorFlags = 0;

    switch (command)
    {
    case CYHAL_SDIO_CMD_GO_IDLE_STATE:
        xcommand.responseType = kCARD_ResponseTypeNone;
        break;
    case CYHAL_SDIO_CMD_SEND_RELATIVE_ADDR:
        xcommand.responseType = kCARD_ResponseTypeR6;
        break;
    case CYHAL_SDIO_CMD_IO_SEND_OP_COND:
        xcommand.responseType = kCARD_ResponseTypeR4;
        break;
    case CYHAL_SDIO_CMD_SELECT_CARD:
        xcommand.responseType = kCARD_ResponseTypeR1;
        break;
    case CYHAL_SDIO_CMD_IO_RW_DIRECT:
    case CYHAL_SDIO_CMD_IO_RW_EXTENDED:
        xcommand.responseType = kCARD_ResponseTypeR5;
        break;
    default:
        return WHD_BADARG;
    }

    if (direction == CYHAL_WRITE)
    {
        xdata.txData = NULL;
    }
    else if (direction == CYHAL_READ)
    {
        xdata.rxData = NULL;
    }

    content.command = &xcommand;
    content.data = NULL;

    result = SDMMCHOST_TransferFunction(obj->sd_card.host, &content);
    if (result != kStatus_Success)
    {
        return result;
    }

    if (response != NULL)
    {
        *response = xcommand.response[0U];
    }

    return result;
}

whd_result_t whd_custom_hal_sdio_bulk_transfer(cyhal_sdio_t *obj, cyhal_transfer_t direction, uint32_t argument, uint8_t mode,
                                   const uint32_t *data, uint16_t length, uint32_t *response)
{
    whd_result_t result = WHD_SUCCESS;
    sdmmchost_transfer_t content;
    sdmmchost_cmd_t xcommand;
    sdmmchost_data_t xdata;

    memset(&content, 0, sizeof(content));
    memset(&xcommand, 0, sizeof(xcommand));
    memset(&xdata, 0, sizeof(xdata));

    uint32_t block_size;

    xcommand.index = CYHAL_SDIO_CMD_IO_RW_EXTENDED;
    xcommand.argument = argument;
    xcommand.responseErrorFlags = 0;
    xcommand.responseType = kCARD_ResponseTypeR5;

    if (direction == CYHAL_WRITE)
    {
        xdata.txData = (uint32_t *)data;
    }
    else if (direction == CYHAL_READ)
    {
        xdata.rxData = (uint32_t *)data;
    }

    if (mode == SDIO_BLOCK_MODE)
    {
        xdata.blockCount = (argument & (uint32_t)(0x1FF));
        xdata.blockSize = SDIO_64B_BLOCK;
    }
    else if (mode == SDIO_BYTE_MODE)
    {
        xdata.blockCount = 1U;
        block_size = find_optimal_block_size(length);
        if (block_size < SDIO_512B_BLOCK)
        {
            argument = (argument & (uint32_t)(~0x1FF)) | block_size;
        }
        else
        {
            argument = (argument & (uint32_t)(~0x1FF));
        }
        xdata.blockSize = (size_t)length;
    }

    content.command = &xcommand;
    content.data = (data == 0) ? NULL : &xdata;

    result = SDMMCHOST_TransferFunction(obj->sd_card.host, &content);

    if (result != kStatus_Success)
    {
        return result;
    }

    if (response != NULL)
    {
        *response = xcommand.response[0U];
    }
    return result;
}

void whd_custom_hal_sdio_irq_enable(cyhal_sdio_t *obj, cyhal_sdio_irq_event_t event, bool enable)
{
    SDMMCHOST_EnableCardInt(cy_imx_sdio->sd_card.host, enable);
}

void whd_custom_sdio_callback(void *pData)
{
    whd_thread_notify_irq(whd_custom_get_wifi_driver());
    SDMMCHOST_EnableCardInt(cy_imx_sdio->sd_card.host, WHD_FALSE);
}

void whd_custom_hal_sdio_unmask_interrupt(void)
{
    SDMMCHOST_EnableCardInt(cy_imx_sdio->sd_card.host, WHD_TRUE);
}

static sdio_block_size_t find_optimal_block_size(uint32_t data_size)
{
    if (data_size > (uint32_t)256)
        return SDIO_512B_BLOCK;
    if (data_size > (uint32_t)128)
        return SDIO_256B_BLOCK;
    if (data_size > (uint32_t)64)
        return SDIO_128B_BLOCK;
    if (data_size > (uint32_t)32)
        return SDIO_64B_BLOCK;
    if (data_size > (uint32_t)16)
        return SDIO_32B_BLOCK;
    if (data_size > (uint32_t)8)
        return SDIO_16B_BLOCK;
    if (data_size > (uint32_t)4)
        return SDIO_8B_BLOCK;
    if (data_size > (uint32_t)2)
        return SDIO_4B_BLOCK;

    return SDIO_4B_BLOCK;
}

/* Empty functions to avoid compilation and linking error */

void whd_custom_hal_sdio_register_irq(cyhal_sdio_t *obj, cyhal_sdio_irq_handler_t handler, void *handler_arg)
{
    /* To be implemented */
}

void whd_custom_hal_sdio_free(cyhal_sdio_t *obj)
{
    /* To be implemented */
}

#endif /* WHD_CUSTOM_HAL && IMXRT */
