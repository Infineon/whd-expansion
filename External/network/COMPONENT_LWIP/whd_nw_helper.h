/*
 * Copyright 2019-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * This is a collection of network helper functions which would be used by various Cypress Middleware libraries.
 *
 */
#pragma once

#ifdef WHD_NETWORK_LWIP

#include <stdint.h>
#include <stdbool.h>


#if defined(__cplusplus)
extern "C" {
#endif

/** \addtogroup nwhelper_utils
 * This is a collection of network helper functions to fetch IPv4 address of the local device, notify IPv4 address
 * change via callback and conversion utilities.
 */

#define cy_assert( error_string, assertion )         do { (void)(assertion); } while(0)

typedef uintptr_t whd_nw_ip_interface_t;


/******************************************************
 *                   Enumerations
 ******************************************************/
/******************************************************************************/
/** \addtogroup group_nwhelper_enums
 * This provides the documentation of all the enums provided by this utility.
 *//** \{ */
/******************************************************************************/
/** IP version */
typedef enum nw_ip_version
{
    NW_IP_IPV4 = 4,          /**< IPv4 version */
    NW_IP_IPV6 = 6,          /**< IPv6 version */
    NW_IP_INVALID_IP = (-1), /**< Invalid IP version */
} whd_nw_ip_version_t;

/** Network interface type */
typedef enum
{
    CY_NW_INF_TYPE_WIFI = 0, /**< Wi-Fi network interface */
    CY_NW_INF_TYPE_ETH       /**< Ethernet network interface */
} whd_network_interface_type_t;

typedef enum
{
    CY_NET_IP_VER_V4 = 4,      /**< Denotes IPv4 version. */
    CY_NET_IP_VER_V6 = 6       /**< Denotes IPv6 version. */
} whd_net_ip_version_t;

typedef struct
{
	whd_nw_ip_version_t version;  /**< IP version. */
    union
    {
        uint32_t v4;     /**< IPv4 address in network byte order. */
        uint32_t v6[4];  /**< IPv6 address in network byte order. */
    } ip;                /**< IP address bytes. */
} whd_net_ip_address_t;


/** \} */

/******************************************************************************/
/** \addtogroup group_nwhelper_structures
 * Lists all the data structures and typedefs provided with the network helper utility along with the documentation.
 *//** \{ */
/******************************************************************************/
/** Network IP status change callback function
 *
 * @param[in] iface : Pointer to the network interface for which the callback is invoked.
 * @param[in] arg   : User data object provided during the status change callback registration.

 * @return none
 */
typedef void (whd_nw_ip_status_change_callback_func_t)(whd_nw_ip_interface_t iface, void *arg);

/** Network IP status change callback info */
typedef struct whd_nw_ip_status_change_callback
{
    whd_nw_ip_status_change_callback_func_t *cb_func; /**< IP address status change callback function */
    void *arg;                                    /**< User data */
    void *priv;                                   /**< NW interface */
} whd_nw_ip_status_change_callback_t;

/**
 * IP addr info
 */
typedef struct whd_nw_ip_address
{
    whd_nw_ip_version_t version; /**< IP version */

    union
    {
        uint32_t v4;         /**< IPv4 address info */
        uint32_t v6[4];      /**< IPv6 address info */
    } ip;                    /**< Union of IPv4 and IPv6 address info */
} whd_nw_ip_address_t;

/** Network interface object */
typedef void* whd_network_interface_object_t;

/** MAC Address info */
typedef struct whd_nw_ip_mac
{
    uint8_t    mac[6];              /**< MAC address                */
} whd_nw_ip_mac_t;

/** ARP Cache Entry info */
typedef struct whd_nw_arp_cache_entry
{
    whd_nw_ip_address_t    ip;         /**< IP address                 */
    whd_nw_ip_mac_t        mac;        /**< MAC address                */
} whd_nw_arp_cache_entry_t;

/**
 * Network interface info structure
 */
typedef struct
{
    whd_network_interface_type_t     type;     /**< Network interface type */
    whd_network_interface_object_t   object;   /**< Pointer to the network interface object */
} whd_network_interface_t;

/** \} */

/*****************************************************************************/
/**
 *
 *  @addtogroup group_nwhelper_func
 *
 * This is a collection of network helper functions which would be used by various Cypress Middleware libraries.
 *
 *  @{
 */
/*****************************************************************************/

/** GET IPv4 address in string format.
 *
 * @param[in]  addr     : Pointer to IPv4 address structure containing the IPv4 address.
 * @param[out] ip_str   : Pointer to the string containing IPv4 address in dotted-decimal notation.
 *                        ip_str must be 16 bytes long.
 * @return 0 : success
 *         1 : failed
 * */
#define INITIALISER_IPV4_ADDRESS(addr_var, addr_val)  addr_var = { CY_NET_IP_VER_V4, { .v4 = (uint32_t)(addr_val) } }
#define MAKE_IPV4_ADDRESS(a, b, c, d)                 ((((uint32_t) d) << 24) | (((uint32_t) c) << 16) | \
                                                       (((uint32_t) b) << 8) |((uint32_t) a))

#define IP_STR_LEN                       16

bool whd_nw_ntoa (whd_nw_ip_address_t *addr, char *ip_str);

/** @} */

#if defined(__cplusplus)
}
#endif

#endif
