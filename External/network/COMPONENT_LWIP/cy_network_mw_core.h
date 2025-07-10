/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

/**
* \addtogroup group_lwip_network_interface_integration lwip-network-interface-integration
* \{
* Functions for linking the lwIP TCP/IP stack with Wi-Fi Host Driver and Ethernet driver
*
* \defgroup group_lwip_network_interface_integration_enums Enumerated types
* \defgroup group_lwip_network_interface_integration_structures Port structures
* \defgroup group_lwip_network_interface_integration_functions Port functions
*/

#pragma once

#ifdef WHD_NETWORK_LWIP
#include <stddef.h>
#include "cy_result.h"
#include "cy_nw_helper.h"
#include "whd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CY_RSLT_NETWORK_INTERFACE_EXISTS                      (1)  /**< Denotes that the interface already exists   */
#define CY_RSLT_NETWORK_ERROR_ADDING_INTERFACE                (2)  /**< Denotes that adding the interface failed    */
#define CY_RSLT_NETWORK_ERROR_STARTING_DHCP                   (3)  /**< Denotes a failure to start DHCP      */
#define CY_RSLT_NETWORK_INTERFACE_DOES_NOT_EXIST              (4)  /**< Denotes that the interface does not exist   */
#define CY_RSLT_NETWORK_BAD_ARG                               (6)  /**< Denotes a BAD arg                    */
#define CY_RSLT_NETWORK_SOCKET_ERROR                          (7)  /**< Denotes the lwIP socket error          */
#define CY_RSLT_NETWORK_SOCKET_CREATE_FAIL                    (8)  /**< Denotes the lwIP socket create failure    */
#define CY_RSLT_NETWORK_INVALID_SOCKET                        (9)  /**< Denotes an invalid socket             */
#define CY_RSLT_NETWORK_CORRUPT_BUFFER                        (10) /**< Denotes a corrupt buffer             */
#define CY_RSLT_NETWORK_DHCP_TIMEOUT                          (11) /**< Denotes the DHCP timeout               */
#define CY_RSLT_NETWORK_DHCP_WAIT_TIMEOUT                     (12) /**< Denotes the DHCP wait timeout          */
#define CY_RSLT_NETWORK_DHCP_MUTEX_ERROR                      (13) /**< Denotes the DHCP Mutex error           */
#define CY_RSLT_NETWORK_ERROR_STARTING_INTERNAL_DHCP          (14) /**< Denotes the failure to start the internal DHCP server */
#define CY_RSLT_NETWORK_INTERFACE_NETWORK_NOT_UP              (15) /**< Denotes that the network is not up for the given interface */
#define CY_RSLT_NETWORK_ERROR_REMOVING_INTERFACE              (16) /**< Denotes an error while removing the interface */
#define CY_RSLT_NETWORK_IPV6_INTERFACE_NOT_READY              (17) /**< Denotes that the IPv6 interface is not ready */
#define CY_RSLT_NETWORK_ERROR_GET_MAC_ADDR                    (18) /**< Denotes the failure to read the MAC address */
#define CY_RSLT_NETWORK_ERROR_PING                            (19) /**< Denotes the failure to send a ping request*/
#define CY_RSLT_NETWORK_ERROR_NOMEM                           (20) /**< Denotes the failure to allocate memory*/
#define CY_RSLT_NETWORK_ERROR_RTOS                            (21) /**< Denotes the failure from RTOS calls*/
#define CY_RSLT_NETWORK_NOT_SUPPORTED                         (22) /**< Denotes that the feature is not supported*/
#define CY_RSLT_NETWORK_LINK_NOT_UP                           (23) /**< Denotes that the Link is not up*/
#define CY_RSLT_NETWORK_ERROR_TRNG                            (24) /**< Denotes that the random number generation failed*/


#define UNUSED_VARIABLE(x) ( (void)(x) )

#define EAPOL_PACKET_TYPE                        (0x888E)

#if !NO_SYS
#define PROTECTED_FUNC_CALL(func) \
LOCK_TCPIP_CORE(); \
func; \
UNLOCK_TCPIP_CORE();
#else
#define PROTECTED_FUNC_CALL(func) \
func;
#endif

#define MULTICAST_IP_TO_MAC(ip)                  {   (uint8_t) 0x01,             \
                                                     (uint8_t) 0x00,             \
                                                     (uint8_t) 0x5e,             \
                                                     (uint8_t) ((ip)[1] & 0x7F), \
                                                     (uint8_t) (ip)[2],          \
                                                     (uint8_t) (ip)[3]           \
                                                 }

#define IPV6_MULTICAST_TO_MAC_PREFIX             (0x33)

#define ARP_WAIT_TIME_IN_MSEC                    (30000)
#define ARP_CACHE_CHECK_INTERVAL_IN_MSEC         (5)

#define DHCP_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS (15000)
#ifndef AUTO_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS
#define AUTO_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS (60000 * 10)
#endif
#define DCHP_RENEWAL_DELAY_IN_MS                 (100)
#define DHCP_STOP_DELAY_IN_MS                    (400)
/**
 * Maximum number of interface instances supported
 */
#define CY_IFACE_MAX_HANDLE                      (4U)

#define MAX_AUTO_IP_RETRIES                      (5)

#ifdef COMPONENT_4390X
#define CY_PRNG_SEED_FEEDBACK_MAX_LOOPS          (1000)
#define CY_PRNG_CRC32_POLYNOMIAL                 (0xEDB88320)
#define CY_PRNG_ADD_CYCLECNT_ENTROPY_EACH_N_BYTE (1024)
#define CY_PRNG_WELL512_STATE_SIZE               (16)
#endif

/* Ping definitions*/
#define PING_DATA_SIZE                           (64)
#define PING_IF_NAME_LEN                         (6)
#define PING_RESPONSE_LEN                        (64)
#define PING_ID                                  (0xAFAF)  /** ICMP identifier for ping */

#define MAX_ETHERNET_PORT                        (2U)
#define ETH_INTERFACE_INDEX                      (2U)

#if defined(COMPONENT_CAT1)
#define MAX_TRNG_BIT_SIZE        (32UL)
#endif

typedef enum
{
    CY_NET_INTERFACE_TYPE_STA = 0,    /**< STA or client interface. */
    CY_NET_INTERFACE_TYPE_AP,         /**< SoftAP interface. */
    CY_NET_INTERFACE_TYPE_AP_STA      /**< Concurrent AP + STA mode. */
} whd_net_interface_t;

/**
 * \addtogroup group_lwip_network_interface_integration_macros
 * \{
 */

/******************************************************
 *                    Constants
 ******************************************************/
/**
 * Suppress unused variable warning
 */
#ifndef UNUSED_VARIABLE
#define UNUSED_VARIABLE(x) ( (void)(x) )
#endif

#define CY_MAC_ADDR_LEN                        (6U)         /**< MAC address length */

/** \} group_lwip_network_interface_integration_macros */

/**
 * \addtogroup group_lwip_network_interface_integration_enums
 * \{
 */

/**
 * Enumeration of network interface role
 */
typedef enum
{
    CY_NETWORK_WIFI_STA_INTERFACE    = 0,  /**< STA interface  */
    CY_NETWORK_WIFI_AP_INTERFACE     = 1,  /**< SoftAP interface */
    CY_NETWORK_ETH_INTERFACE         = 2   /**< Ethernet interface */
} whd_network_hw_interface_type_t;
/**
 * Enumeration of network activity type
 */
typedef enum
{
    CY_NETWORK_ACTIVITY_TX = 0,  /**< Tx network activity  */
    CY_NETWORK_ACTIVITY_RX = 1   /**< Rx network activity  */
} whd_network_activity_type_t;

/**
* IPv6 types
*/
typedef enum
{
    CY_NETWORK_IPV6_LINK_LOCAL = 0, /**< IPv6 link-local address type */
    CY_NETWORK_IPV6_GLOBAL          /**< IPv6 global address type */
} whd_network_ipv6_type_t;

/** \} group_lwip_network_interface_integration_enums */

/**
* \addtogroup group_lwip_network_interface_integration_structures
* \{
*/

/**
 * Interface context structure to store the hardware and
 * network interface structures
 */
typedef struct
{
    whd_network_hw_interface_type_t  iface_type;     /**< Interface type */
    uint8_t                         iface_idx;      /**< Ethernet interface index */
    void                           *hw_interface;  /**< HW interface handle. Caller should map it to the HW interface-specific structure */
    void                           *nw_interface;  /**< Network interface. Caller should map it to the network stack-specific interface structure */
    bool                            is_initialized; /**< Indicates whether the network interface is initialized */
    uint8_t                         mac_address[CY_MAC_ADDR_LEN]; /**< MAC address of the device to be configured to the lwIP stack */
} whd_network_interface_context;

/**
 * This structure represents a static IP address
 */
typedef struct whd_network_static_ip_addr
{
    whd_nw_ip_address_t addr;    /**< IP address for the network interface */
    whd_nw_ip_address_t netmask; /**< Netmask for the network interface */
    whd_nw_ip_address_t gateway; /**< Default gateway for network traffic */
} whd_network_static_ip_addr_t;

/** \} group_lwip_network_interface_integration_structures */

/**
* \addtogroup group_lwip_network_interface_integration_functions
* \{
*/


/**
 * Initializes the network stack, and allocates and initializes
 * the resources needed by this library
 *
 * @return CY_RSLT_SUCCESS if successful; failure code otherwise.
 */
cy_rslt_t whd_network_init();

/**
 * Deinitializes the network stack, and releases the resources
 * allocated in the \ref whd_network_init function
 *
 * @return CY_RSLT_SUCCESS if successful; failure code otherwise.
 */
cy_rslt_t whd_network_deinit();

/**
 * Adds a network interface. Binds the hardware interface to the IP layer of the network stack.
 *
 * @param[in]  iface_type      Network interface type \ref whd_network_hw_interface_type_t
 * @param[in]  iface_idx       Network interface index. If the whd_network_add_nw_interface function is called multiple times for same interface type,
 *                             the index passed should be the hardware port available on the device. This is used to handle multiple Ethernet interfaces.
 *                             For example, ethernet port0 will have index as 0 and ethernet port1 will have index as 1.
 * @param[in]  hw_interface    Handle to the already initialized HW interface.
 *                             For example, the whd_interface_t pointer that is initialized using the cybsp_wifi_init_primary() function. Similarly for Ethernet, it can be the Ethernet interface pointer.
 * @param[in]  mac_address     MAC address to be configured to the network interface
 * @param[in]  ipaddr          Static IP address provided by the user
 * @param[out] iface_context   Interface context. Network core library allocates memory for this structure and initializes it. Caller should not modify the contents of this structure, and should not free the memory. To free the memory allocated by this function, the caller should invoke the \ref whd_network_remove_nw_interface function.
 *
 * @return CY_RSLT_SUCCESS if successful; failure code otherwise.
 */
cy_rslt_t whd_network_add_nw_interface(whd_network_hw_interface_type_t iface_type, uint8_t iface_idx,
                                      void *hw_interface, uint8_t *mac_address,
                                      whd_network_static_ip_addr_t *ipaddr, whd_network_interface_context **iface_context);

/**
 * Removes a network interface. Unbinds the network interface from the IP layer of the network stack.
 *
 * @param[in] iface_context   Interface context that has been created with the \ref whd_network_add_nw_interface function.
 *
 * @return CY_RSLT_SUCCESS if successful; failure code otherwise.
 */
cy_rslt_t whd_network_remove_nw_interface(whd_network_interface_context *iface_context);

/**
 * Returns the network interface for the given network interface type
 *
 * @param[in] iface_type      Network interface type
 * @param[in] iface_idx       Network interface index
 *
 * @return Associated network stack's net interface structure for the HW interface
 */
void* whd_network_get_nw_interface(whd_network_hw_interface_type_t iface_type, uint8_t iface_idx);

/**
 * Brings up the network link layer and
 * starts DHCP if required
 *
 * @param[in] iface_context   Interface context that has been created with the \ref whd_network_add_nw_interface function
 *
 *  @return CY_RSLT_SUCCESS on successful network bring up; failure code otherwise.
 */
cy_rslt_t whd_network_ip_up(whd_network_interface_context *iface_context);

/**
 * Brings down the network interface and network link layer,
 * and stops DHCP
 *
 * @param[in] iface_context   Interface context that has been created with the \ref whd_network_add_nw_interface function
 *
 * @return CY_RSLT_SUCCESS on successful teardown of the network; failure code otherwise.
 */
cy_rslt_t whd_network_ip_down(whd_network_interface_context *iface_context);

/**
 * Invalidates all ARP entries and renews DHCP
 *
 * @param[in] iface_context   Interface context that has been created with the \ref whd_network_add_nw_interface function
 *
 * @return CY_RSLT_SUCCESS on successful network bring up; failure code otherwise.
 */
cy_rslt_t whd_network_dhcp_renew(whd_network_interface_context *iface_context);

/**
 * IP change callback function prototype
 * Callback function which can be registered to receive IP address changes
 * must be of this prototype.
 *
 * @param[in] iface_context   Interface context that has been created with the \ref whd_network_add_nw_interface function
 * @param[in] user_data       User data provided at the time of callback registration
 */
typedef void (*whd_network_ip_change_callback_t)(whd_network_interface_context *iface_context, void *user_data);

/**
 * Helps to register/unregister for any IP address changes from the network stack.
 * Passing "NULL" as callback will deregister the IP changes callback.
 *
 * @param[in] iface_context   Interface context that has been created with the \ref whd_network_add_nw_interface function
 * @param[in] cb              IP address change callback function
 * @param[in] user_data       User data that is passed to the callback function
 *
 */
void whd_network_register_ip_change_cb(whd_network_interface_context *iface_context, whd_network_ip_change_callback_t cb, void *user_data);

/**
 * Network activity callback function prototype.
 * The callback function which can be registered/unregistered for any network activity
 * must be of this prototype.
 */
typedef void (*whd_network_activity_event_callback_t)(bool event_type);

/**
 * Helps to register/unregister a callback function that will be invoked on any Tx/Rx activity on any network interface.
 * Passing "NULL" as cb will deregister the activity callback.
 *
 * @param[in] cb Network activity callback function
 *
 */
void whd_network_activity_register_cb(whd_network_activity_event_callback_t cb);

/**
 * Retrieves the IPv4 address of the given interface. See \ref whd_network_get_ipv6_address API to get IPv6 addresses.
 *
 * @param[in]   iface_context  : Interface context that has been created with the \ref whd_network_add_nw_interface function
 * @param[out]  ip_addr        : Pointer to an IP address structure of type whd_nw_ip_address_t
 *
 * @return CY_RSLT_SUCCESS if IP-address get is successful; failure code otherwise.

 */
cy_rslt_t whd_network_get_ip_address(whd_network_interface_context *iface_context, whd_nw_ip_address_t *ip_addr);

/**
 * Retrieves the IPv6 address of the given interface
 *
 * @param[in]   iface_context  : Interface context that has been created with the \ref whd_network_add_nw_interface function
 * @param[in]   type           : IPv6 address type. Only IPv6 link-local address type is supported.
 * @param[out]  ip_addr        : Pointer to an IP address structure of type whd_nw_ip_address_t
 *
 * @return CY_RSLT_SUCCESS if IP-address get is successful; failure code otherwise.

 */
cy_rslt_t whd_network_get_ipv6_address(whd_network_interface_context *iface_context, whd_network_ipv6_type_t type, whd_nw_ip_address_t *ip_addr);

/**
 * Retrieves the gateway IP address of the given interface
 *
 * @param[in]   iface_context  : Interface context that has been created with the \ref whd_network_add_nw_interface function
 * @param[out]  gateway_addr   : Pointer to an IP address structure of type whd_nw_ip_address_t to be filled with the gateway IP address
 *
 * @return CY_RSLT_SUCCESS if retrieval of the gateway IP address was successful; failure code otherwise.

 */
cy_rslt_t whd_network_get_gateway_ip_address(whd_network_interface_context *iface_context, whd_nw_ip_address_t *gateway_addr);

/**
 * Retrieves the MAC address of the gateway for the given interface. Uses Address Resolution Protocol (ARP) to retrieve the gateway MAC address.
 *
 * This function is a blocking call and uses an internal timeout while running ARP.
 *
 * @param[in]   iface_context  : Interface context that has been created with the \ref whd_network_add_nw_interface function
 * @param[out]  mac_addr       : Pointer to a MAC address structure which is filled with the gateway's MAC address on successful return
 *
 * @return CY_RSLT_SUCCESS if retrieval of the gateway MAC address was successful; failure code otherwise.
 */
cy_rslt_t whd_network_get_gateway_mac_address(whd_network_interface_context *iface_context, whd_nw_ip_mac_t *mac_addr);

/**
 * Retrieves the subnet mask address of the given interface
 *
 * @param[in]   iface_context  : Interface context that has been created with the \ref whd_network_add_nw_interface function
 * @param[out]  net_mask_addr  : Pointer to an IP address structure of type whd_nw_ip_address_t to be filled with the subnet mask address of the interface.
 *
 * @return CY_RSLT_SUCCESS if retrieval of the subnet mask address was successful; failure code otherwise.
 */
cy_rslt_t whd_network_get_netmask_address(whd_network_interface_context *iface_context, whd_nw_ip_address_t *net_mask_addr);

/**
 * Sends a ping request to the given IP address
 *
 * @param[in]  if_ctx           : Pointer to the network interface context (typecast to void *)
 * @param[in]  address          : Pointer to the destination IP address structure to which the ping request will be sent
 * @param[in]  timeout_ms       : Ping request timeout in milliseconds
 * @param[in]  elapsed_time_ms  : Pointer to store the roundtrip time (in milliseconds),
 *                                i.e., the time taken to receive the ping response from the destination
 * @return CY_RSLT_SUCCESS if successful; CY_RSLT_NETWORK_ERROR_PING if failure.
 * */
cy_rslt_t whd_network_ping(void *if_ctx, whd_nw_ip_address_t *address, uint32_t timeout_ms, uint32_t* elapsed_time_ms);

/**
 * Gets an IP address assigned from the connected AP's DHCP server
 *
 * @param[in] interface         : WHD interface
 * @param[out] nw_sta_ifx_ctx   : Pointer to the pointer to the network interface context than can be reused by the user
 *
 * @return CY_RSLT_SUCCESS if successful ; CY_RSLT_NETWORK_DHCP_TIMEOUT if failed
 */
cy_rslt_t whd_network_ip_assign(whd_interface_t interface, whd_network_interface_context **nw_sta_ifx_ctx);

/**
 * Initializes the network interface for WiFi Host Driver to interact with LWIP
 *
 * @param[in] interface         : WHD interface
 * @param[in] iface_type        : Network interface type \ref whd_network_hw_interface_type_t
 * @param[in] static_ip_ptr     : Pointer to the static IP address provided by the user
 * @param[out] nw_sta_ifx_ctx   : Pointer to the pointer to the network interface context than can be reused by the user
 *
 * @return CY_RSLT_SUCCESS if successful; error code from internal function calls, if failed
 */
cy_rslt_t whd_network_up(whd_interface_t interface, whd_network_hw_interface_type_t iface_type, whd_network_static_ip_addr_t *static_ip_ptr, whd_network_interface_context **nw_sta_ifx_ctx);

/**
 * De-initializes the network interface between LWIP and WiFi Host Driver
 *
 * @param[in] interface         : WHD interface
 * @param[in] iface_type        : Network interface type \ref whd_network_hw_interface_type_t
 *
 */
void whd_network_down(whd_interface_t interface, whd_network_hw_interface_type_t iface_type);

/** \} group_lwip_network_interface_integration_functions */


#ifdef __cplusplus
}
#endif
/** \} group_lwip_network_interface_integration */
#endif
