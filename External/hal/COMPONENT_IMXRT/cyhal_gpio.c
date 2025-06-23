/***************************************************************************//**
* \file cyhal_gpio.c
*
* \brief
* Provides a high level interface for interacting with the Cypress GPIO.
* This interface abstracts out the chip specific details. If any chip specific
* functionality is necessary, or performance is critical the low level functions
* can be used directly.
*
********************************************************************************
* \copyright
* Copyright 2018-2019 Cypress Semiconductor Corporation
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
*******************************************************************************/

/*
 *
 * The empty function definitions in this file are to avoid build errors for 3rd party platforms
 * The definitions will be updated on the basis of platform specific requirements wherever applicable
 * For now these changes are limited to the IMXRT platform
 *
 */

#if defined(WHD_CUSTOM_HAL) && defined(IMXRT)

#include "cyhal_gpio.h"

cy_rslt_t cyhal_gpio_init(cyhal_gpio_t pin, cyhal_gpio_direction_t direction, cyhal_gpio_drive_mode_t drvMode,
                          bool initVal)
{

}

/** Uninitialize the gpio peripheral and the cyhal_gpio_t object
 *
 * @param[in] pin Pin number
 */
void cyhal_gpio_free(cyhal_gpio_t pin)
{

}

/** Register/clear an interrupt handler for the pin toggle pin IRQ event
 *
 * @param[in] pin           The pin number
 * @param[in] intrPriority  The NVIC interrupt channel priority
 * @param[in] handler       The function to call when the specified event happens. Pass NULL to unregister the handler.
 * @param[in] handler_arg   Generic argument that will be provided to the handler when called, can be NULL
 */
void cyhal_gpio_register_irq(cyhal_gpio_t pin, uint8_t intrPriority, cyhal_gpio_irq_handler_t handler,
                             void *handler_arg)
{

}

/** Enable or Disable the GPIO IRQ
 *
 * @param[in] pin    The GPIO object
 * @param[in] event  The GPIO IRQ event
 * @param[in] enable True to turn on interrupts, False to turn off
 */
void cyhal_gpio_irq_enable(cyhal_gpio_t pin, cyhal_gpio_irq_event_t event, bool enable)
{

}

#endif /* WHD_CUSTOM_HAL && IMXRT */
