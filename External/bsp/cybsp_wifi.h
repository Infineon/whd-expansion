/***********************************************************************************************//**
 * \file cybsp_wifi.h
 *
 * \brief
 * Basic abstraction layer for dealing with boards containing a Cypress MCU. This
 * API provides convenience methods for initializing and manipulating different
 * hardware found on the board.
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2018-2022 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
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
 **************************************************************************************************/

/**
 * \addtogroup group_bsp_wifi WiFi Initialization
 * \{
 * Basic integration code for interfacing the WiFi Host Driver (WHD) with the Board
 * Support Packages (BSPs).
 */
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef WHD_USE_WCM
#include "whd_custom_hal_sdio.h"
#endif

#define cybsp_wifi_init_primary(interface) whd_custom_wifi_init(interface)
#define cybsp_wifi_init_secondary(interface, mac) whd_custom_wifi_init_secondary(interface, mac)
#define cybsp_wifi_deinit(interface) whd_custom_wifi_deinit(interface)

#ifdef __cplusplus
} /*extern "C" */
#endif