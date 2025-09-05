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

#pragma once

#if defined(WHD_CUSTOM_HAL) && defined(IMXRT)

#include <stdint.h>
#include <stdbool.h>
#include "cy_result.h"
#include "cyhal_hw_types.h"
#include "cyhal_modules.h"
#include "whd_thread.h"
#include "whd_bus_common.h"
#include "whd_bus_protocol_interface.h"
#include "whd_bus_sdio_protocol.h"
#include "whd_debug.h"
#include "whd_events_int.h"
#include "whd_int.h"
#include "whd_chip.h"
#include "whd_utils.h"
#ifndef PROTO_MSGBUF
#include "whd_sdpcm.h"
#endif /* PROTO_MSGBUF */
#include "whd_wifi_api.h"
#include "whd_clm.h"
#include "whd_wlioctl.h"
#include "whd_types_int.h"
#include "whd_chip_constants.h"
#include "whd_proto.h"

#if defined(__cplusplus)
extern "C" {
#endif

typedef enum
{
    WHD_INTERFACE_TYPE_STA = 0,    /**< STA or client interface. */
    WHD_INTERFACE_TYPE_AP,         /**< SoftAP interface. */
    WHD_INTERFACE_TYPE_AP_STA      /**< Concurrent AP + STA mode. */
} whd_interface_type_t;

whd_result_t whd_custom_hal_sdio_init(cyhal_sdio_t *obj);

void whd_custom_hal_sdio_irq_enable(cyhal_sdio_t *obj, cyhal_sdio_irq_event_t event, bool enable);

whd_result_t whd_custom_hal_sdio_send_cmd(const cyhal_sdio_t *obj, cyhal_transfer_t direction,
                              cyhal_sdio_command_t command, uint32_t argument, uint32_t *response);

whd_result_t whd_custom_hal_sdio_bulk_transfer(cyhal_sdio_t *obj, cyhal_transfer_t direction, uint32_t argument, uint8_t mode,
                                   const uint32_t *data, uint16_t length, uint32_t *response);

void whd_custom_hal_sdio_unmask_interrupt(void);

void whd_custom_hal_sdio_register_irq(cyhal_sdio_t *obj, cyhal_sdio_irq_handler_t handler, void *handler_arg);

whd_result_t whd_custom_wifi_init(whd_interface_t *interface);

whd_driver_t whd_custom_get_wifi_driver(void);

whd_result_t whd_custom_get_wifi_interface(whd_interface_type_t interface_type, whd_interface_t *iface);

#ifdef __cplusplus
}
#endif

#endif /* WHD_CUSTOM_HAL && IMXRT */
