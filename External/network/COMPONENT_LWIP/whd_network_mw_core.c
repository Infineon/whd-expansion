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
#ifdef WHD_NETWORK_LWIP

#include <string.h>
#include <stdint.h>
#include "lwipopts.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/ethip6.h"
#include "lwip/igmp.h"
#include "lwip/nd6.h"
#include "netif/ethernet.h"
#include "lwip/prot/autoip.h"
#include "lwip/prot/dhcp.h"
#include "lwip/dns.h"
#include "lwip/inet_chksum.h"
#include "lwip/icmp.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

#include "whd_network_mw_core.h"
#include "whd_lwip_dhcp_server.h"
#include "cy_result.h"

#if defined(CYBSP_WIFI_CAPABLE)
#include "whd_network_buffer.h"
#include "whd.h"
#include "whd_wifi_api.h"
#include "whd_network_types.h"
#include "whd_buffer_api.h"

#ifdef COMPONENT_4390X
#include "whd_wlioctl.h"
#endif
#endif

/* While using lwIP/sockets errno is required. Since IAR and ARMC6 doesn't define errno variable, the following definition is required for building it successfully. */
#if !( (defined(__GNUC__) && !defined(__ARMCC_VERSION)) )
int errno;
#endif

/******************************************************
 *                      Macros
 ******************************************************/
/**
 * Suppress unused variable warning
 */

static bool is_sta_network_up          = false;
static bool is_ap_network_up           = false;
static bool is_sta_interface_created   = false;
static whd_network_interface_context *whd_nw_ap_if_ctx;
static whd_network_interface_context *whd_nw_sta_if_ctx;

static cy_lwip_dhcp_server_t internal_dhcp_server;

/******************************************************
 *               Variable Definitions
 ******************************************************/

typedef void (*cy_wifimwcore_eapol_packet_handler_t) (whd_interface_t whd_iface, whd_buffer_t buffer);

cy_rslt_t cy_wifimwcore_eapol_register_receive_handler(cy_wifimwcore_eapol_packet_handler_t eapol_packet_handler);



static whd_network_activity_event_callback_t activity_callback = NULL;
static bool is_dhcp_client_required = false;
#if defined(CYBSP_WIFI_CAPABLE)
static cy_wifimwcore_eapol_packet_handler_t internal_eapol_packet_handler = NULL;
#endif
static whd_network_ip_change_callback_t ip_change_callback = NULL;

/* Interface init status */
static bool ip_networking_inited[CY_IFACE_MAX_HANDLE];
#define SET_IP_NETWORK_INITED(interface_index, status)   (ip_networking_inited[(interface_index)&3] = status)

/* Interface UP status */
static bool ip_up[CY_IFACE_MAX_HANDLE];
#define SET_IP_UP(interface_index, status)               (ip_up[(interface_index)&3] = status)

struct  netif                                           *cy_lwip_ip_handle[CY_IFACE_MAX_HANDLE];
#define LWIP_IP_HANDLE(interface)                        (cy_lwip_ip_handle[(interface) & 3])

static struct netif       sta_ip_handle;
static struct netif       ap_ip_handle;
static struct netif       eth0_ip_handle;
static struct netif       eth1_ip_handle;

struct netif* cy_lwip_ip_handle[CY_IFACE_MAX_HANDLE] =
{
    [0] =  &sta_ip_handle,
    [1] =  &ap_ip_handle,
    [2] =  &eth0_ip_handle,
    [3] =  &eth1_ip_handle
};

struct icmp_packet
{
    struct   icmp_echo_hdr hdr;
    uint8_t  data[PING_DATA_SIZE];
};

whd_network_interface_context iface_context_database[CY_IFACE_MAX_HANDLE];
static uint8_t iface_count = 0;
static uint8_t connectivity_lib_init = 0;
static bool is_tcp_initialized       = false;

#ifdef COMPONENT_4390X
static uint32_t prng_well512_state[CY_PRNG_WELL512_STATE_SIZE];
static uint32_t prng_well512_index = 0;

static uint32_t prng_add_cyclecnt_entropy_bytes = CY_PRNG_ADD_CYCLECNT_ENTROPY_EACH_N_BYTE;

/** Mutex to protect the PRNG state array */
static cy_mutex_t cy_prng_mutex;
static cy_mutex_t *cy_prng_mutex_ptr;

#endif

/******************************************************
 *               Static Function Declarations
 ******************************************************/

#if LWIP_IPV4
static void invalidate_all_arp_entries(struct netif *netif);
#endif
static void internal_ip_change_callback (struct netif *netif);
static bool is_interface_added(uint8_t interface_index);
static cy_rslt_t is_interface_valid(whd_network_interface_context *iface);
static bool is_network_up(uint8_t interface_index);

#if LWIP_IPV4
static void ping_prepare_echo(struct icmp_packet *iecho, uint16_t len, uint16_t *ping_seq_num);
static err_t ping_send(int socket_hnd, const whd_nw_ip_address_t* address, struct icmp_packet *iecho, uint16_t *sequence_number);
static err_t ping_recv(int socket_hnd, whd_nw_ip_address_t* address, uint16_t *ping_seq_num);
#endif

#ifdef COMPONENT_4390X
static uint32_t prng_well512_get_random ( void );
static void     prng_well512_add_entropy( const void* buffer, uint16_t buffer_length );
cy_rslt_t cy_prng_get_random( void* buffer, uint32_t buffer_length );
cy_rslt_t cy_prng_add_entropy( const void* buffer, uint32_t buffer_length );
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

#if defined(CYBSP_WIFI_CAPABLE)

bool whd_nw_ntoa (whd_nw_ip_address_t *addr, char *ip_str)
{
    uint8_t index = 0;
    uint8_t arr[4] = {0};
    uint32_t ip_addr;
    if(ip_str == NULL || addr == NULL)
    {
        return 1;
    }
    ip_addr = addr->ip.v4;
    while(ip_addr != 0)
    {
        arr[index] = ip_addr & 0xff;
        ip_addr = ip_addr >> 8;
        index++;
    }
    memset(ip_str,0, (15*sizeof(char)));
    sprintf(ip_str, "%d.%d.%d.%d", arr[0],arr[1],arr[2],arr[3]);
    return 0;
}

/*
 * This function takes packets from the radio driver and passes them into the
 * lwIP stack. If the stack is not initialized, or if the lwIP stack does not
 * accept the packet, the packet is freed (dropped). If the packet is of type EAPOL
 * and if the Extensible Authentication Protocol over LAN (EAPOL) handler is registered, the packet will be redirected to the registered
 * handler and should be freed by the EAPOL handler.
 */
void whd_host_network_process_ethernet_data(whd_interface_t iface, whd_buffer_t buf)
{
    uint8_t *data = whd_buffer_get_current_piece_data_pointer(iface->whd_driver, buf);
    uint16_t ethertype;
    struct netif *net_interface = NULL;

    WPRINT_WHD_INFO(("%s(): START iface->role:[%d]\n", __FUNCTION__, iface->role ));

    if(iface->role == WHD_STA_ROLE)
    {
        net_interface = &sta_ip_handle;
        WPRINT_WHD_DEBUG(("STA net_interface:[%p] \n", net_interface));
    }
    else if(iface->role == WHD_AP_ROLE)
    {
        net_interface = &ap_ip_handle;
        WPRINT_WHD_DEBUG(("AP net_interface:[%p] \n", net_interface));
    }
    else
    {
        whd_host_buffer_release(buf, WHD_NETWORK_RX) ;
        return;
    }

    ethertype = (uint16_t)(data[12] << 8 | data[13]);
    if (ethertype == EAPOL_PACKET_TYPE)
    {
        if( internal_eapol_packet_handler != NULL )
        {
            internal_eapol_packet_handler(iface, buf);
        }
        else
        {
            whd_host_buffer_release(buf, WHD_NETWORK_RX) ;
        }
    }
    else
    {
        /* Call the registered activity handler with the argument as false
         * indicating that there is RX packet
         */
        if (activity_callback)
        {
            activity_callback(false);
        }

        WPRINT_WHD_DEBUG(("Send data up to LwIP \n"));
        /* If the interface is not yet set up, drop the packet */
        if (net_interface->input == NULL || net_interface->input(buf, net_interface) != ERR_OK)
        {
            WPRINT_WHD_ERROR(("Drop packet before lwip \n"));
            whd_host_buffer_release(buf, WHD_NETWORK_RX) ;
        }
    }
    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
}

/* Create a duplicate pbuf of the input pbuf */
static struct pbuf *pbuf_dup(const struct pbuf *orig)
{
    struct pbuf *p = pbuf_alloc(PBUF_LINK, orig->tot_len, PBUF_RAM);
    if (p != NULL)
    {
        pbuf_copy(p, orig);
        p->flags = orig->flags;
    }
    return p;
}

/*
 * This function takes the packets from the lwIP stack and sends them down to the radio.
 * If the radio is not ready, return and error; otherwise, add a reference to
 * the packet for the radio driver and send the packet to the radio driver. The radio
 * driver puts the packet into a send queue and sends it based on another thread. This
 * other thread will release the packet reference once the packet is actually sent.
 */
static err_t wifioutput(struct netif *iface, struct pbuf *p)
{
    whd_network_interface_context *if_ctx;
    if_ctx = (whd_network_interface_context *)iface->state;

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if (whd_wifi_is_ready_to_transceive((whd_interface_t)if_ctx->hw_interface) != WHD_SUCCESS)
    {
        WPRINT_WHD_ERROR(("Wi-Fi is not ready, packet not sent\n"));
        return ERR_INPROGRESS ;
    }

    struct pbuf *whd_buf = pbuf_dup(p);
    if (whd_buf == NULL)
    {
        WPRINT_WHD_ERROR(("failed to allocate buffer for outgoing packet\n"));
        return ERR_MEM;
    }
    /* Call the registered activity handler with the argument as true
     * indicating there is TX packet
     */
    if (activity_callback)
    {
        activity_callback(true);
    }
    whd_network_send_ethernet_data((whd_interface_t)if_ctx->hw_interface, whd_buf) ;

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));

    return ERR_OK ;
}


#if LWIP_IPV4 && LWIP_IGMP
/*
 * Respond to IGMP (group management) requests
 */
static err_t igmp_filter(struct netif *iface, const ip4_addr_t *group, enum netif_mac_filter_action action)
{
    whd_network_interface_context *if_ctx;
    if_ctx = (whd_network_interface_context *)iface->state;

    whd_mac_t mac = { MULTICAST_IP_TO_MAC((uint8_t*)group) };

    switch ( action )
    {
        case NETIF_ADD_MAC_FILTER:
            if ( whd_wifi_register_multicast_address( (whd_interface_t)if_ctx->hw_interface, &mac ) != CY_RSLT_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        case NETIF_DEL_MAC_FILTER:
            if ( whd_wifi_unregister_multicast_address( (whd_interface_t)if_ctx->hw_interface, &mac ) != CY_RSLT_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        default:
            return ERR_VAL;
    }

    return ERR_OK;
}
#endif

#if LWIP_IPV6 && LWIP_IPV6_MLD
/*
 * Function called by the lwIP stack to add or delete an entry in the IPv6 multicast filter table of the Ethernet MAC.
 */
static err_t mld_mac_filter(struct netif *iface, const ip6_addr_t *group, enum netif_mac_filter_action action)
{
    whd_mac_t macaddr;
    cy_rslt_t res;
    const uint8_t *ptr = (const uint8_t *)group->addr;
    whd_network_interface_context *if_ctx;
    if_ctx = (whd_network_interface_context *)iface->state;

    /* Convert the IPv6 multicast address to the MAC address.
     * The first two octets of the converted MAC address are fixed values (0x33 and 0x33).
     * The last four octets are the last four octets of the IPv6 multicast address.
     */
    macaddr.octet[0] = IPV6_MULTICAST_TO_MAC_PREFIX ;
    macaddr.octet[1] = IPV6_MULTICAST_TO_MAC_PREFIX ;
    macaddr.octet[2] = ptr[12] ;
    macaddr.octet[3] = ptr[13] ;
    macaddr.octet[4] = ptr[14] ;
    macaddr.octet[5] = ptr[15] ;

    switch ( action )
    {
        case NETIF_ADD_MAC_FILTER:
            res = whd_wifi_register_multicast_address( (whd_interface_t)if_ctx->hw_interface, &macaddr );
            if (  res != CY_RSLT_SUCCESS )
            {
                WPRINT_WHD_ERROR(("whd_wifi_register_multicast_address call failed, err = %lx\n", res));
                return ERR_VAL;
            }
            break;

        case NETIF_DEL_MAC_FILTER:
            res = whd_wifi_unregister_multicast_address( (whd_interface_t)if_ctx->hw_interface, &macaddr );
            if ( res != CY_RSLT_SUCCESS )
            {
                WPRINT_WHD_ERROR(("whd_wifi_unregister_multicast_address call failed, err = %lx\n", res));
                return ERR_VAL;
            }
            break;

        default:
            WPRINT_WHD_ERROR(("Invalid MAC Filter Action: %d\n", action));
            return ERR_VAL;
    }

    return ERR_OK;
}
#endif

/*
 * This function is called when adding the Wi-Fi network interface to lwIP,
 * it actually performs the initialization for the netif interface.
 */
static err_t wifiinit(struct netif *iface)
{
    cy_rslt_t res;
    whd_network_interface_context *if_ctx;
    if_ctx = (whd_network_interface_context *)iface->state;

    whd_mac_t macaddr;
    whd_interface_t whd_iface = (whd_interface_t)if_ctx->hw_interface;
#ifdef COMPONENT_4390X
    uint8_t buffer[WLC_GET_RANDOM_BYTES];
    uint32_t *wlan_rand = NULL;
#endif

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));
    /*
     * Set the MAC address of the interface
     */
    res = whd_wifi_get_mac_address(whd_iface, &macaddr);

    if (res != CY_RSLT_SUCCESS)
    {
        WPRINT_WHD_ERROR(("whd_wifi_get_mac_address call failed, err = %lx\n", res));
        return res ;
    }
    memcpy(&iface->hwaddr, &macaddr, sizeof(macaddr));
    iface->hwaddr_len = sizeof(macaddr);

#ifdef COMPONENT_4390X
    /*
     * CYW43907 kits do not have the TRNG module. Get a random number from the WLAN and feed
     * it as a seed to the PRNG function.
     * Before invoking whd_wifi_get_iovar_buffer, WHD interface should be initialized.
     * However, wcminit is called from cy_lwip_add_interface; the WHD interface is an
     * input to the cy_lwip_add_interface API. So it is safe to invoke
     * whd_wifi_get_iovar_buffer here.
     */
    if(WHD_SUCCESS != whd_wifi_get_iovar_buffer(whd_iface, IOVAR_STR_RAND, buffer, WLC_GET_RANDOM_BYTES))
    {
        return ERR_IF;
    }
    wlan_rand = (uint32_t *)buffer;

    /* Initialize the mutex to protect the PRNG WELL512 state */
    if (cy_prng_mutex_ptr == NULL)
    {
        cy_prng_mutex_ptr = &cy_prng_mutex;
        cy_rtos_init_mutex(cy_prng_mutex_ptr);
    }
    /* Feed the random number obtained from the WLAN to the WELL512
     * algorithm as the initial seed value.
     */
    cy_prng_add_entropy((const void *)wlan_rand, 4);
#endif
    /*
     * Set up the information associated with sending packets
     */
#if LWIP_IPV4
    iface->output = etharp_output;
#endif
    iface->linkoutput = wifioutput;
    iface->mtu = WHD_LINK_MTU;
    iface->flags |= (NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP) ;
#if LWIP_IPV6_MLD
    iface->flags |= NETIF_FLAG_MLD6;
#endif
    /*
     * Set the interface name for the interface
     */
    iface->name[0] = 'w' ;
    iface->name[1] = 'l' ;

#if LWIP_IPV4 && LWIP_IGMP
    netif_set_igmp_mac_filter(iface, igmp_filter) ;
#endif


#if LWIP_IPV6 == 1
    /*
     * Filter the output packets for IPv6 through the Ethernet output
     * function for IPv6
     */
    iface->output_ip6 = ethip6_output ;

    /*
     * Automatically generate a unicast IP address based on
     * neighbor discovery
     */
    iface->ip6_autoconfig_enabled = 1 ;

    /*
     * Create a link-local IPv6 address
     */
    netif_create_ip6_linklocal_address(iface, 1);

    /*
     * Tell the radio that you want to listen to solicited-node multicast
     * packets. These packets are part of the IPv6 neighbor discovery
     * process.
     */
    macaddr.octet[0] = 0x33 ;
    macaddr.octet[1] = 0x33 ;
    macaddr.octet[2] = 0xff ;
    whd_wifi_register_multicast_address((whd_interface_t)if_ctx->hw_interface, &macaddr) ;

    /*
     * Tell the radio that you want to listen to the multicast address
     * that targets all IPv6 devices. These packets are part of the IPv6
     * neighbor discovery process.
     */
    memset(&macaddr, 0, sizeof(macaddr)) ;
    macaddr.octet[0] = 0x33 ;
    macaddr.octet[1] = 0x33 ;
    macaddr.octet[5] = 0x01 ;
    whd_wifi_register_multicast_address((whd_interface_t)if_ctx->hw_interface, &macaddr) ;

#if LWIP_IPV6_MLD
    /*
     * Register the MLD MAC filter callback function that will be called by the lwIP stack to add or delete an
     * entry in the IPv6 multicast filter table of the Ethernet MAC
     */
    netif_set_mld_mac_filter(iface, mld_mac_filter);
#endif
#endif

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return 0 ;
}
#endif

cy_rslt_t whd_network_init( void )
{
    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if( connectivity_lib_init )
    {
        connectivity_lib_init++;
        WPRINT_WHD_DEBUG(("\n Connectivity library is already initialized.\n"));
        return CY_RSLT_SUCCESS;
    }

    /** Initialize TCP ip stack, LWIP init is called through tcpip_init **/
    if(!is_tcp_initialized)
    {
        /*Network stack initialization*/
        tcpip_init(NULL, NULL);
        is_tcp_initialized = true;
    }

    WPRINT_WHD_DEBUG(("\n tcpip_init success.\n"));

    /*Memory to store iface_context. Currently, 4 contexts*/
    for (int i = 0; i < CY_IFACE_MAX_HANDLE; i++)
    {
        memset(&(iface_context_database[i]), 0, sizeof(whd_network_interface_context) );
    }

    connectivity_lib_init++;

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
}

cy_rslt_t whd_network_deinit( void )
{

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if( connectivity_lib_init == 0 )
    {
        WPRINT_WHD_DEBUG(("\n Connectivity library is De-initialized.\n"));
        return CY_RSLT_SUCCESS;
    }
    else
    {
        connectivity_lib_init--;
        if(connectivity_lib_init == 0)
        {
        }
    }

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
}

cy_rslt_t whd_network_add_nw_interface(whd_network_hw_interface_type_t iface_type, uint8_t iface_idx,
                                         void *hw_interface, uint8_t *mac_address,
                                         whd_network_static_ip_addr_t *static_ipaddr,
                                         whd_network_interface_context **iface_context)
{
    bool iface_found = false;
    uint8_t index = 0;
#if LWIP_IPV4
    ip4_addr_t ipaddr, netmask, gateway ;
#endif

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if (iface_context == NULL || hw_interface == NULL || (iface_idx >= MAX_ETHERNET_PORT))
    {
        WPRINT_WHD_ERROR(("Bad arguments %p %p \n", iface_context, hw_interface));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    if (iface_count >= CY_IFACE_MAX_HANDLE)
    {
        WPRINT_WHD_ERROR(("Error adding interface \n"));
        return CY_RSLT_NETWORK_ERROR_ADDING_INTERFACE;
    }

    /* Static IP address is mandatory for AP */
    if((iface_type == CY_NETWORK_WIFI_AP_INTERFACE) && (static_ipaddr == NULL))
    {
        WPRINT_WHD_ERROR(("Error static IP addr cannot be NULL for AP interface \n"));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    /*Check if the interface is already added*/
    for (int i = 0; i < CY_IFACE_MAX_HANDLE; i++)
    {
        if( ( iface_context_database[i].is_initialized == true ) &&
            ( iface_context_database[i].iface_type == iface_type ) &&
            ( iface_context_database[i].iface_idx == iface_idx ) )
        {
           iface_found = true;
           *iface_context = &(iface_context_database[i]);

           break;
        }
    }

    if(iface_found == true)
    {
        iface_idx = ((iface_type == CY_NETWORK_ETH_INTERFACE)? (CY_NETWORK_ETH_INTERFACE + iface_idx) : iface_type);
        if (is_interface_added((uint8_t)iface_idx))
        {
            WPRINT_WHD_ERROR(("Interface already exists \n"));
            return CY_RSLT_NETWORK_INTERFACE_EXISTS;
        }
    }
    else
    {
        /*No duplicate; check for an available index and update the interface database*/
        for( index = 0; index < CY_IFACE_MAX_HANDLE; index++ )
        {
            if( iface_context_database[index].is_initialized == false )
            {
                iface_context_database[index].iface_type = iface_type;
                iface_context_database[index].iface_idx  = iface_idx;

                iface_context_database[index].hw_interface = hw_interface;
                WPRINT_WHD_DEBUG((" Adding interface \n"));

                *iface_context = &(iface_context_database[index]);
                break;
            }
        }

        /* Handling index out of bound exception */
        if( index == 4 )
        {
            return CY_RSLT_NETWORK_BAD_ARG;
        }
    }

#if LWIP_IPV4
    /* Assign the IP address if static; otherwise, zero the IP address */
    if (static_ipaddr != NULL)
    {
        WPRINT_WHD_INFO(("static_ipaddr is NOT NULL \n"));

#ifdef WPRINT_ENABLE_WHD_INFO
#define IPV4_MAX_STR_LEN 15
    char ip_str[IPV4_MAX_STR_LEN];
    whd_nw_ntoa(&(static_ipaddr->addr), ip_str);
    WPRINT_WHD_INFO(("static ipaddr assigned %s assigned \n", ip_str));
    whd_nw_ntoa(&(static_ipaddr->gateway), ip_str);
    WPRINT_WHD_INFO(("static gateway assigned %s assigned \n", ip_str));
    whd_nw_ntoa(&(static_ipaddr->netmask), ip_str);
    WPRINT_WHD_INFO(("static netmask assigned %s assigned \n", ip_str));
#endif

        memcpy(&gateway, &static_ipaddr->gateway.ip.v4, sizeof(gateway));
        memcpy(&ipaddr, &static_ipaddr->addr.ip.v4, sizeof(ipaddr));
        memcpy(&netmask, &static_ipaddr->netmask.ip.v4, sizeof(netmask));
    }
    else
    {
        WPRINT_WHD_INFO(("static_ipaddr from application API is NULL \n"));
        memset(&gateway, 0, sizeof(gateway));
        memset(&ipaddr, 0, sizeof(ipaddr));
        memset(&netmask, 0, sizeof(netmask));
    }

#endif

    /* update netif structure */
#if defined(CYBSP_WIFI_CAPABLE)
    if( CY_NETWORK_WIFI_STA_INTERFACE == iface_context_database[index].iface_type || CY_NETWORK_WIFI_AP_INTERFACE == iface_context_database[index].iface_type )
    {
        /* Assign network instance to the iface database */
        iface_context_database[index].nw_interface = LWIP_IP_HANDLE(iface_context_database[index].iface_type);
        WPRINT_WHD_DEBUG(("iface_context_database[index].nw_interface:[%p]\n", iface_context_database[index].nw_interface));

        WPRINT_WHD_DEBUG(("iface_context_database[index].hw_interface:[%p] hw_interface:[%p]\n", iface_context_database[index].hw_interface, hw_interface));
#if LWIP_IPV4
        /* Add the interface to the lwIP stack and make it the default */
        if(netifapi_netif_add(iface_context_database[index].nw_interface, &ipaddr, &netmask, &gateway, &iface_context_database[index], wifiinit, tcpip_input) != CY_RSLT_SUCCESS)
        {
            WPRINT_WHD_ERROR(("Error adding interface \n"));
            return CY_RSLT_NETWORK_ERROR_ADDING_INTERFACE;
        }
#else
        if(netifapi_netif_add(iface_context_database[index].nw_interface, &iface_context_database[index], wifiinit, tcpip_input) != CY_RSLT_SUCCESS)
        {
            WPRINT_WHD_ERROR(("Error adding interface \n"));
            return CY_RSLT_NETWORK_ERROR_ADDING_INTERFACE;
        }
#endif
    }

    if(iface_context_database[index].iface_type == CY_NETWORK_WIFI_STA_INTERFACE)
    {
        if(static_ipaddr == NULL)
        {
            is_dhcp_client_required = true;
        }
        netifapi_netif_set_default(iface_context_database[index].nw_interface);
    }

#if LWIP_NETIF_STATUS_CALLBACK == 1
    /*
     * Register a handler for any address changes.
     * Note: The "status" callback will also be called when the interface
     * goes up or down.
     */
    netif_set_status_callback(iface_context_database[index].nw_interface, internal_ip_change_callback);
#endif /* LWIP_NETIF_STATUS_CALLBACK */

    SET_IP_NETWORK_INITED(iface_context_database[index].iface_type, true);

#endif

    WPRINT_WHD_DEBUG(("added network net_interface:[%p] \n", iface_context_database[index].nw_interface));

    /* Update the interface initialized flag only after netif is added. This flag will be referred in Rx data handler */
    iface_context_database[index].is_initialized = true;

    /*Update the valid interface count: Used as max cap for interface addition*/

    iface_count++;

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
}

void* whd_network_get_nw_interface(whd_network_hw_interface_type_t iface_type, uint8_t iface_idx)
{
    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if( ((iface_type != CY_NETWORK_WIFI_STA_INTERFACE) && (iface_type != CY_NETWORK_WIFI_AP_INTERFACE) && (iface_type != CY_NETWORK_ETH_INTERFACE))
        || (iface_idx >= MAX_ETHERNET_PORT) )
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return NULL;
    }

    for (int i = 0; i < CY_IFACE_MAX_HANDLE; i++)
    {
        if( ( iface_context_database[i].is_initialized == true ) &&
            ( iface_context_database[i].iface_type == iface_type ) &&
            ( iface_context_database[i].iface_idx == iface_idx ) )
        {
            return (void *)(iface_context_database[i].nw_interface);
        }
    }

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return NULL;
}

cy_rslt_t whd_network_remove_nw_interface(whd_network_interface_context *iface_context)
{
    uint8_t interface_index;

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));
    if(is_interface_valid(iface_context) != CY_RSLT_SUCCESS)
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    interface_index = (uint8_t)((iface_context->iface_type == CY_NETWORK_ETH_INTERFACE)? (CY_NETWORK_ETH_INTERFACE + iface_context->iface_idx) : iface_context->iface_type);

    /* Check any one interface is added */
    if(!iface_count)
    {
        WPRINT_WHD_ERROR(("Error Not added \n"));
        return CY_RSLT_NETWORK_INTERFACE_DOES_NOT_EXIST;
    }

    /* Interface can be removed only if the interface was previously added and the network is down */
    if(!is_interface_added(interface_index))
    {
        WPRINT_WHD_ERROR(("Error Interface doesn't exist \n"));
        return CY_RSLT_NETWORK_INTERFACE_DOES_NOT_EXIST;
    }

    if(is_network_up(interface_index))
    {
        WPRINT_WHD_ERROR(("Error removing interface, bring down the network before removing the interface \n"));
        return CY_RSLT_NETWORK_ERROR_REMOVING_INTERFACE;
    }

    /* Remove the status callback */
    netif_set_remove_callback(LWIP_IP_HANDLE(interface_index), internal_ip_change_callback);
    /* Remove the interface */
    netifapi_netif_remove(LWIP_IP_HANDLE(interface_index));
    if(iface_context->iface_type == CY_NETWORK_WIFI_STA_INTERFACE || iface_context->iface_type == CY_NETWORK_ETH_INTERFACE)
    {
        is_dhcp_client_required = false;
    }

    SET_IP_NETWORK_INITED(interface_index, false);

#ifdef COMPONENT_4390X
    /* cy_prng_mutex_ptr is initialized when the first network interface is initialized.
     * Deinitialize the mutex only after all the interfaces are deinitialized.
     */
    if (!ip_networking_inited[CY_NETWORK_WIFI_STA_INTERFACE] &&
        !ip_networking_inited[CY_NETWORK_WIFI_AP_INTERFACE])
    {
        if (cy_prng_mutex_ptr != NULL)
        {
            cy_rtos_deinit_mutex(cy_prng_mutex_ptr);
            cy_prng_mutex_ptr = NULL;
        }
    }
#endif

    /* Clear interface details from database */
    for (int i = 0; i < CY_IFACE_MAX_HANDLE; i++)
    {
        if( ( iface_context_database[i].iface_type == iface_context->iface_type ) &&
            ( iface_context_database[i].iface_idx == iface_context->iface_idx ) )
        {
            iface_context_database[i].is_initialized = false;
            iface_context_database[i].nw_interface = NULL;
            iface_context_database[i].hw_interface = NULL;
        }
    }

    /*Update the valid interface count: Used as max cap for interface addition*/
    iface_count--;

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
}

cy_rslt_t whd_network_ip_up(whd_network_interface_context *iface)
{
    cy_rslt_t result                     = CY_RSLT_SUCCESS;
    uint8_t   interface_index;
#if LWIP_IPV4
    uint32_t    address_resolution_timeout = 0;
    bool        timeout_occurred = false;
    ip4_addr_t  ip_addr;
#endif

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));
    if(is_interface_valid(iface) != CY_RSLT_SUCCESS)
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    interface_index = (uint8_t)((iface->iface_type == CY_NETWORK_ETH_INTERFACE)? (CY_NETWORK_ETH_INTERFACE + iface->iface_idx) : iface->iface_type);
    WPRINT_WHD_DEBUG(("interface_index:[%d] \n", interface_index));

    if(is_network_up(interface_index))
    {
        WPRINT_WHD_DEBUG(("Network is already up \n"));
        return CY_RSLT_SUCCESS;
    }

    /*
     * If LPA is enabled, invoke the activity callback to resume the network stack,
     * before invoking the lwIP APIs that requires TCP core lock
     */
    if (activity_callback)
    {
        activity_callback(true);
    }

    /*
    * Bring up the network interface
    */
    netifapi_netif_set_up((LWIP_IP_HANDLE(interface_index)));

    /*
    * Bring up the network link layer
    */
    netifapi_netif_set_link_up((LWIP_IP_HANDLE(interface_index)));

#if LWIP_IPV6
    /* Wait for the IPv6 address to change from tentative to valid or invalid */
    while(ip6_addr_istentative(netif_ip6_addr_state(LWIP_IP_HANDLE(interface_index), 0)))
    {
        /* Give the lwIP stack time to change the state */
        cy_rtos_delay_milliseconds(ND6_TMR_INTERVAL);
    }

    /* lwIP changes state to either INVALID or VALID. Check if the state is VALID */
    if(ip6_addr_isvalid(netif_ip6_addr_state(LWIP_IP_HANDLE(interface_index), 0)))
    {
        WPRINT_WHD_INFO(("IPv6 Network ready IP: %s \r\n", ip6addr_ntoa(netif_ip6_addr(LWIP_IP_HANDLE(interface_index), 0))));
    }
    else
    {
        WPRINT_WHD_INFO(("IPv6 network not ready \r\n"));
    }
#endif

#if LWIP_IPV4
    if(iface->iface_type == CY_NETWORK_WIFI_STA_INTERFACE || iface->iface_type == CY_NETWORK_ETH_INTERFACE)
    {
        if(is_dhcp_client_required)
        {
            /* TO DO :  Save the current power save state */
            /* TO DO :  Disable power save for the DHCP exchange */

            /*
             * For DHCP only, reset netif IP address
             *to avoid reusing the previous netif IP address
             * given from the previous DHCP session.
             */
            ip4_addr_set_zero(&ip_addr);

            /*
             * If LPA is enabled, invoke the activity callback to resume the network stack
             * before invoking the lwIP APIs that require the TCP core lock.
             */
            if (activity_callback)
            {
                activity_callback(true);
            }

            netif_set_ipaddr(LWIP_IP_HANDLE(interface_index), &ip_addr);

            /*
             * If LPA is enabled, invoke the activity callback to resume the network stack
             * before invoking the lwIP APIs that require the TCP core lock.
             */
            if (activity_callback)
            {
                activity_callback(true);
            }

            /* TO DO : DHCPv6 need to be handled when we support IPV6 addresses other than the link local address */
            /* Start DHCP */
            WPRINT_WHD_DEBUG(("Start DHCP client netif:[%p]\n", LWIP_IP_HANDLE(interface_index) ));
            if(netifapi_dhcp_start(LWIP_IP_HANDLE(interface_index)) != CY_RSLT_SUCCESS)
            {
                WPRINT_WHD_ERROR(("CY_RSLT_NETWORK_ERROR_STARTING_DHCP error\n"));
                return CY_RSLT_NETWORK_ERROR_STARTING_DHCP;
            }
            /* Wait a little to allow DHCP to complete */

            while((netif_dhcp_data(LWIP_IP_HANDLE(interface_index))->state != DHCP_STATE_BOUND) && (timeout_occurred == false))
            {
                cy_rtos_delay_milliseconds(10);
                address_resolution_timeout += 10;
                if(address_resolution_timeout >= DHCP_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS)
                {
                    /* Timeout has occurred */
                    timeout_occurred = true;
                }
            }

            WPRINT_WHD_DEBUG(("netif_dhcp_data(LWIP_IP_HANDLE(interface_index))->state:[%d]\n",netif_dhcp_data(LWIP_IP_HANDLE(interface_index))->state));
            if (timeout_occurred)
            {
                /*
                 * If LPA is enabled, invoke the activity callback to resume the network stack
                 * before invoking the lwIP APIs that require the TCP core lock.
                 */
                if (activity_callback)
                {
                    activity_callback(true);
                }
                netifapi_dhcp_release_and_stop(LWIP_IP_HANDLE(interface_index));
                cy_rtos_delay_milliseconds(DHCP_STOP_DELAY_IN_MS);

                /*
                * If LPA is enabled, invoke the activity callback to resume the network stack
                * before invoking the lwIP APIs that require the TCP core lock.
                */
                if (activity_callback)
                {
                    activity_callback(true);
                }

                dhcp_cleanup(LWIP_IP_HANDLE(interface_index));
#if LWIP_AUTOIP
                int   tries = 0;
                WPRINT_WHD_INFO(("Unable to obtain IP address via DHCP. Perform Auto IP\n"));
                address_resolution_timeout = 0;
                timeout_occurred            = false;

                /*
                 * If LPA is enabled, invoke the activity callback to resume the network stack
                 * before invoking the lwIP APIs that require the TCP core lock.
                 */
                if (activity_callback)
                {
                    activity_callback(true);
                }

                if (autoip_start(LWIP_IP_HANDLE(interface_index) ) != ERR_OK )
                {
                    /* trick: Skip the while-loop, do the cleaning up stuff */
                    timeout_occurred = true;
                }

                while ((timeout_occurred == false) && ((netif_autoip_data(LWIP_IP_HANDLE(interface_index))->state != AUTOIP_STATE_BOUND)))
                {
                    cy_rtos_delay_milliseconds(10);
                    address_resolution_timeout += 10;
                    if(address_resolution_timeout >= AUTO_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS)
                    {
                        if(tries++ < MAX_AUTO_IP_RETRIES)
                        {
                            address_resolution_timeout = 0;
                        }
                        else
                        {
                            timeout_occurred = true;
                        }
                     }
                }

                if (timeout_occurred)
                {
                    WPRINT_WHD_ERROR(("Unable to obtain IP address via DCHP and AutoIP\n"));

                    /*
                     * If LPA is enabled, invoke the activity callback to resume the network stack
                     * before invoking the lwIP APIs that require the TCP core lock.
                     */
                    if (activity_callback)
                    {
                        activity_callback(true);
                    }

                    autoip_stop(LWIP_IP_HANDLE(interface_index));
                    return CY_RSLT_NETWORK_DHCP_WAIT_TIMEOUT;
                }
                else
                {
                    WPRINT_WHD_DEBUG(("IP address obtained through AutoIP \n"));
                }
#else
                WPRINT_WHD_ERROR(("CY_RSLT_NETWORK_DHCP_WAIT_TIMEOUT error\n"));
                return CY_RSLT_NETWORK_DHCP_WAIT_TIMEOUT;
#endif
            }
        }
    }
    else
    {
        /* DHCP Server iniliasiation */
        if((result = whd_lwip_dhcp_server_start(&internal_dhcp_server, iface))!= CY_RSLT_SUCCESS)
        {
            WPRINT_WHD_ERROR(("Unable to obtain IP address via DHCP\n"));
            return CY_RSLT_NETWORK_ERROR_STARTING_DHCP;
        }
    }
#endif

    SET_IP_UP(interface_index, true);

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return result;
}

cy_rslt_t whd_network_ip_down(whd_network_interface_context *iface)
{
    uint8_t interface_index;

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if(is_interface_valid(iface) != CY_RSLT_SUCCESS)
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    interface_index = (uint8_t)((iface->iface_type == CY_NETWORK_ETH_INTERFACE)? (CY_NETWORK_ETH_INTERFACE + iface->iface_idx) : iface->iface_type);

    if(!is_network_up((uint8_t)interface_index))
    {
        WPRINT_WHD_INFO(("Network is not UP \r\n"));
        return CY_RSLT_NETWORK_INTERFACE_NETWORK_NOT_UP;
    }

#if LWIP_IPV4
    if(is_dhcp_client_required)
    {
#if LWIP_AUTOIP
        if(netif_autoip_data(LWIP_IP_HANDLE(interface_index))->state == AUTOIP_STATE_BOUND)
        {
            /*
             * If LPA is enabled, invoke the activity callback to resume the network stack
             * before invoking the lwIP APIs that require the TCP core lock.
             */
            if (activity_callback)
            {
                activity_callback(true);
            }

            autoip_stop(LWIP_IP_HANDLE(interface_index));
        }
        else
#endif
        {
            /*
             * If LPA is enabled, invoke the activity callback to resume the network stack
             * before invoking the lwIP APIs that require the TCP core lock.
             */
            if (activity_callback)
            {
                activity_callback(true);
            }

            netifapi_dhcp_release_and_stop(LWIP_IP_HANDLE(interface_index));
            cy_rtos_delay_milliseconds(DHCP_STOP_DELAY_IN_MS);

            /*
             * If LPA is enabled, invoke the activity callback to resume the network stack
             * before invoking the lwIP APIs that require the TCP core lock.
             */
            if (activity_callback)
            {
                activity_callback(true);
            }

            dhcp_cleanup(LWIP_IP_HANDLE(interface_index));
        }
    }
    if(iface->iface_type == CY_NETWORK_WIFI_AP_INTERFACE)
    {

        WPRINT_WHD_DEBUG(("Stop DHCP for SoftAP interface \n" ));
        /* Stop the internal DHCP server for the SoftAP interface */
        /* DHCP Server stop to be implemented */
    }
#endif

    /*
     * If LPA is enabled, invoke the activity callback to resume the network stack
     * before invoking the lwIP APIs that require the TCP core lock.
     */
    if (activity_callback)
    {
        activity_callback(true);
    }

    /*
    * Bring down the network link layer
    */
    netifapi_netif_set_link_down(LWIP_IP_HANDLE(interface_index));

    /*
    * Bring down the network interface
    */
    netifapi_netif_set_down(LWIP_IP_HANDLE(interface_index));

    /* TO DO : clear all ARP cache */

    /** TO DO:
     *  Kick the radio chip if it is in power save mode if the link down event is due to missing beacons.
     *  Setting the chip to the same power save mode is sufficient.
     */
    SET_IP_UP(interface_index, false);

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
}

void whd_network_register_ip_change_cb(whd_network_interface_context *iface, whd_network_ip_change_callback_t cb, void *user_data)
{
    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));
    /* Assign the callback arguments */
    ip_change_callback = cb;
    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
}

/*
 * This function helps to register/deregister the callback for network activity
 */
void whd_network_activity_register_cb(whd_network_activity_event_callback_t cb)
{
    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));
    /* Update the activity callback with the argument passed */
    activity_callback = cb;
    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
}

/*
 * This function requests for an IP address from DHCP server of the AP after connection
 */
cy_rslt_t whd_network_get_ip_address(whd_network_interface_context *iface_context, whd_nw_ip_address_t *ip_addr)
{
#if LWIP_IPV4
    struct netif *net_interface  = NULL;
    uint32_t ipv4_addr;
    ip4_addr_t* addr = NULL;

    UNUSED_VARIABLE(addr);

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if((ip_addr == NULL) || (is_interface_valid(iface_context) != CY_RSLT_SUCCESS))
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    net_interface = (struct netif *)iface_context->nw_interface;

#if LWIP_IPV6
        ipv4_addr = net_interface->ip_addr.u_addr.ip4.addr;
        addr = &net_interface->ip_addr.u_addr.ip4;
#else
        ipv4_addr = net_interface->ip_addr.addr;
        addr = &net_interface->ip_addr;
#endif

    ip_addr->version = NW_IP_IPV4;
    ip_addr->ip.v4 = ipv4_addr;
    WPRINT_WHD_INFO(("IP Address %s assigned \n", ip4addr_ntoa(addr)));

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
#else
    WPRINT_WHD_DEBUG(("%s() LWIP_IPV4 flag is not enabled \n", __FUNCTION__ ));
    return CY_RSLT_NETWORK_NOT_SUPPORTED;
#endif
}

cy_rslt_t whd_network_get_ipv6_address(whd_network_interface_context *iface_context, whd_network_ipv6_type_t type, whd_nw_ip_address_t *ip_addr)
{
#if LWIP_IPV6
    struct netif *net_interface  = NULL;
    const ip6_addr_t* ipv6_addr  = NULL;

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if((ip_addr == NULL) || (is_interface_valid(iface_context) != CY_RSLT_SUCCESS))
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    net_interface = (struct netif *)iface_context->nw_interface;

    ipv6_addr = netif_ip6_addr(net_interface, 0);
    if(ipv6_addr != NULL)
    {
        ip_addr->version = NW_IP_IPV6;
        ip_addr->ip.v6[0] = ipv6_addr->addr[0];
        ip_addr->ip.v6[1] = ipv6_addr->addr[1];
        ip_addr->ip.v6[2] = ipv6_addr->addr[2];
        ip_addr->ip.v6[3] = ipv6_addr->addr[3];

        WPRINT_WHD_INFO(("IPV6 Address %s assigned \n", ip6addr_ntoa(netif_ip6_addr(net_interface, 0))));
    }
    else
    {
        memset(ip_addr, 0, sizeof(whd_nw_ip_address_t));
        WPRINT_WHD_ERROR(("IPV6 network not ready \n"));
        return CY_RSLT_NETWORK_IPV6_INTERFACE_NOT_READY;
    }

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
#else
    WPRINT_WHD_DEBUG(("%s() LWIP_IPV6 flag is not enabled \n", __FUNCTION__ ));
    return CY_RSLT_NETWORK_NOT_SUPPORTED;
#endif
}

cy_rslt_t whd_network_get_gateway_ip_address(whd_network_interface_context *iface_context, whd_nw_ip_address_t *gateway_addr)
{
#if LWIP_IPV4
    struct netif *net_interface  = NULL;
    uint32_t ipv4_addr;
    ip4_addr_t* addr = NULL;

    UNUSED_VARIABLE(addr);

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if((gateway_addr == NULL) || (is_interface_valid(iface_context) != CY_RSLT_SUCCESS))
    {
        WPRINT_WHD_ERROR(("Invalid arguments %p %p\n", gateway_addr, is_interface_valid(iface_context)));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    net_interface = (struct netif *)(iface_context->nw_interface);

#if LWIP_IPV6
    ipv4_addr = net_interface->gw.u_addr.ip4.addr;
    addr = &net_interface->gw.u_addr.ip4;
#else
    ipv4_addr = net_interface->gw.addr;
    addr = &net_interface->gw;
#endif

    gateway_addr->version = NW_IP_IPV4;
    gateway_addr->ip.v4 = ipv4_addr;
    WPRINT_WHD_INFO(("Gateway IP Address %s assigned \n", ip4addr_ntoa(addr)));

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
#else
    WPRINT_WHD_DEBUG(("%s() LWIP_IPV4 flag is not enabled \n", __FUNCTION__ ));
    return CY_RSLT_NETWORK_NOT_SUPPORTED;
#endif
}

cy_rslt_t whd_network_get_gateway_mac_address(whd_network_interface_context *iface_context, whd_nw_ip_mac_t *mac_addr)
{
#if LWIP_IPV4
    err_t err;
    whd_nw_ip_address_t gateway_ip_addr;
    struct eth_addr *eth_ret = NULL;
    const ip4_addr_t *ip_ret = NULL;
    int32_t arp_waittime = ARP_WAIT_TIME_IN_MSEC;
    ssize_t arp_index = -1;
    ip4_addr_t ipv4addr;
    struct netif *net_interface  = NULL;

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if((mac_addr == NULL) || (is_interface_valid(iface_context) != CY_RSLT_SUCCESS))
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    net_interface = (struct netif *)(iface_context->nw_interface);

    if (whd_network_get_gateway_ip_address(iface_context, &gateway_ip_addr) != CY_RSLT_SUCCESS)
    {
        WPRINT_WHD_ERROR(("whd_network_get_gateway_ip_address failed\n" ));
        return  CY_RSLT_NETWORK_ERROR_GET_MAC_ADDR;
    }

    ipv4addr.addr = gateway_ip_addr.ip.v4;

    /* Check if the gateway address entry is already present in the ARP cache */
    arp_index = etharp_find_addr(net_interface, (const ip4_addr_t *) &ipv4addr, &eth_ret, (const ip4_addr_t **) &ip_ret);
    if(arp_index == -1)
    {
        /* Address entry is not present in the ARP cache. Send the ARP request.*/
        err = etharp_request(net_interface, (const ip4_addr_t *) &ipv4addr);
        if(err != ERR_OK)
        {
            WPRINT_WHD_ERROR(("etharp_request failed with error %d\n", err));
            return  CY_RSLT_NETWORK_ERROR_GET_MAC_ADDR;
        }

        do
        {
            arp_index = etharp_find_addr(net_interface, (const ip4_addr_t *) &ipv4addr, &eth_ret, (const ip4_addr_t **) &ip_ret);
            if(arp_index != -1)
            {
                 WPRINT_WHD_INFO(("arp entry found \r\n"));
                 break;
            }
            cy_rtos_delay_milliseconds(ARP_CACHE_CHECK_INTERVAL_IN_MSEC);
            arp_waittime -= ARP_CACHE_CHECK_INTERVAL_IN_MSEC;
            if(arp_waittime <= 0)
            {
                WPRINT_WHD_INFO(("Could not resolve MAC address for the given destination address \r\n"));
                return CY_RSLT_NETWORK_ERROR_GET_MAC_ADDR;
            }
        } while(1);

    }

    memcpy(mac_addr, eth_ret->addr, CY_MAC_ADDR_LEN);

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
#else
    WPRINT_WHD_DEBUG(("%s() LWIP_IPV4 flag is not enabled \n", __FUNCTION__ ));
    return CY_RSLT_NETWORK_NOT_SUPPORTED;
#endif
}

cy_rslt_t whd_network_get_netmask_address(whd_network_interface_context *iface_context, whd_nw_ip_address_t *net_mask_addr)
{
#if LWIP_IPV4
    struct netif *net_interface  = NULL;
    uint32_t ipv4_addr;
    ip4_addr_t* addr = NULL;

    UNUSED_VARIABLE(addr);

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));
    if((net_mask_addr == NULL) || (is_interface_valid(iface_context) != CY_RSLT_SUCCESS))
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    net_interface = (struct netif *)(iface_context->nw_interface);

#if LWIP_IPV6
    ipv4_addr = net_interface->netmask.u_addr.ip4.addr;
    addr = &net_interface->netmask.u_addr.ip4;
#else
    ipv4_addr = net_interface->netmask.addr;
    addr = &net_interface->netmask;
#endif

    net_mask_addr->version = NW_IP_IPV4;
    net_mask_addr->ip.v4 = ipv4_addr;
    WPRINT_WHD_INFO(("net mask IP Address %s assigned \n", ip4addr_ntoa(addr)));

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
#else
    WPRINT_WHD_DEBUG(("%s() LWIP_IPV4 flag is not enabled \n", __FUNCTION__ ));
    return CY_RSLT_NETWORK_NOT_SUPPORTED;
#endif
}

cy_rslt_t whd_network_dhcp_renew(whd_network_interface_context *iface)
{
#if LWIP_IPV4
    uint8_t interface_index;

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));
    if(is_interface_valid(iface) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    interface_index = (uint8_t)((iface->iface_type == CY_NETWORK_ETH_INTERFACE)? (CY_NETWORK_ETH_INTERFACE + iface->iface_idx) : iface->iface_type);

    /* Invalidate ARP entries */
    netifapi_netif_common(LWIP_IP_HANDLE(interface_index), (netifapi_void_fn) invalidate_all_arp_entries, NULL );


    /*
     * If LPA is enabled, invoke the activity callback to resume the network stack
     * before invoking the lwIP APIs that require the TCP core lock.
     */
    if (activity_callback)
    {
        activity_callback(true);
    }

    /* Renew DHCP */
    netifapi_netif_common(LWIP_IP_HANDLE(interface_index), NULL, dhcp_renew);

    cy_rtos_delay_milliseconds(DCHP_RENEWAL_DELAY_IN_MS);

    WPRINT_WHD_DEBUG(("%s(): END \n", __FUNCTION__ ));
    return CY_RSLT_SUCCESS;
#else
    WPRINT_WHD_DEBUG(("%s() LWIP_IPV4 flag is not enabled \n", __FUNCTION__ ));
    return CY_RSLT_NETWORK_NOT_SUPPORTED;
#endif
}

/**
 * Remove all ARP table entries of the specified netif.
 * @param netif Points to a network interface
 */
#if LWIP_IPV4
void invalidate_all_arp_entries(struct netif *netif)
{
     /*Free all entries in ARP list */
    etharp_cleanup_netif(netif);
}
#endif

cy_rslt_t whd_network_ping(void *iface_context, whd_nw_ip_address_t *address, uint32_t timeout_ms, uint32_t* elapsed_time_ms)
{
#if LWIP_IPV4
    cy_time_t send_time;
    cy_time_t recvd_time;
    err_t err;
    struct timeval timeout_val;
    struct icmp_packet ping_packet;
    uint16_t ping_seq_num = 0;
    int socket_for_ping = -1;
    struct netif *net_interface;
    char if_name[PING_IF_NAME_LEN];
    struct ifreq iface;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    WPRINT_WHD_DEBUG(("%s(): START \n", __FUNCTION__ ));

    if(iface_context == NULL || address == NULL || elapsed_time_ms == NULL)
    {
        WPRINT_WHD_ERROR(("%s: Invalid arguments \n", __func__));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    whd_network_interface_context *if_ctx;
    if_ctx = (whd_network_interface_context *)iface_context;

    /*
     * If LPA is enabled, invoke the activity callback to resume the network stack,
     * before invoking the LwIP APIs
    */
    if (activity_callback)
    {
        activity_callback(true);
    }

    /* Open a local socket for pinging */
    socket_for_ping = lwip_socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP);
    if (socket_for_ping < 0)
    {
        result = CY_RSLT_NETWORK_ERROR_PING;
        WPRINT_WHD_ERROR(("lwiP socket open error \n"));
        goto exit;
    }

    /* Convert the timeout into struct timeval */
    timeout_val.tv_sec  = (long)(timeout_ms / 1000);
    timeout_val.tv_usec = (long)((timeout_ms % 1000) * 1000);

    /*
     * If LPA is enabled, invoke the activity callback to resume the network stack,
     * before invoking the LwIP APIs
    */
    if (activity_callback)
    {
        activity_callback(true);
    }

    /* Set the receive timeout on the local socket, so ping will time out */
    if(lwip_setsockopt(socket_for_ping, SOL_SOCKET, SO_RCVTIMEO, &timeout_val, sizeof(struct timeval)) != ERR_OK)
    {
        result = CY_RSLT_NETWORK_ERROR_PING;
        WPRINT_WHD_ERROR(("lwip socket setting receive timeout error \n"));
        goto exit;
    }

    /* Bind the interface to the device */
    net_interface = (struct netif *)if_ctx->nw_interface;
    memset(&iface, 0, sizeof(iface));
    memcpy(if_name, net_interface->name, sizeof(net_interface->name));
    snprintf(&if_name[2], (PING_IF_NAME_LEN - 2), "%u", (uint8_t)(net_interface->num));
    memcpy(iface.ifr_name, if_name, PING_IF_NAME_LEN);

    if(lwip_setsockopt(socket_for_ping, SOL_SOCKET, SO_BINDTODEVICE, &iface, sizeof(iface)) != ERR_OK)
    {
        result = CY_RSLT_NETWORK_ERROR_PING;
        WPRINT_WHD_ERROR(("lwip socket setting socket bind error \n"));
        goto exit;
    }

    /* Send a ping request */
    err = ping_send(socket_for_ping, address, &ping_packet, &ping_seq_num);
    if (err != ERR_OK)
    {
        result = CY_RSLT_NETWORK_ERROR_PING;
        WPRINT_WHD_ERROR(("sending ping failed\n"));
        goto exit;
    }
    /* Record the time the ping request was sent */
    cy_rtos_get_time(&send_time);

    /* Wait for the ping reply */
    err = ping_recv(socket_for_ping, address, &ping_seq_num);
    if (err != ERR_OK)
    {
        result = CY_RSLT_NETWORK_ERROR_PING;
        WPRINT_WHD_ERROR(("receiving ping failed\n"));
        goto exit;
    }

    /* Compute the elapsed time since a ping request was initiated */
    cy_rtos_get_time(&recvd_time);
    *elapsed_time_ms = (uint32_t)(recvd_time - send_time);

exit:
    /* Close the socket */
    if(socket_for_ping >= 0)
    {
        /*
         * If LPA is enabled, invoke the activity callback to resume the network stack,
         * before invoking the LwIP APIs
        */
        if (activity_callback)
        {
            activity_callback(true);
        }
        lwip_close(socket_for_ping);
    }
    return result;
#else
    WPRINT_WHD_DEBUG(("%s() LWIP_IPV4 flag is not enabled \n", __FUNCTION__ ));
    return CY_RSLT_NETWORK_NOT_SUPPORTED;
#endif
}

#ifdef CYBSP_WIFI_CAPABLE
/* Used to register callback for EAPOL packets */
cy_rslt_t cy_wifimwcore_eapol_register_receive_handler( cy_wifimwcore_eapol_packet_handler_t eapol_packet_handler )
{
    internal_eapol_packet_handler = eapol_packet_handler;
    return CY_RSLT_SUCCESS;
}
#endif

#if LWIP_IPV4
void ping_prepare_echo(struct icmp_packet *iecho, uint16_t len, uint16_t *ping_seq_num)
{
    int i;
    ICMPH_TYPE_SET(&iecho->hdr, ICMP_ECHO);
    ICMPH_CODE_SET(&iecho->hdr, 0);
    iecho->hdr.chksum = 0;
    iecho->hdr.id = PING_ID;
    iecho->hdr.seqno = htons(++(*ping_seq_num));

    /* Fill the additional data buffer with some data */
    for ( i = 0; i < (int)sizeof(iecho->data); i++ )
    {
        iecho->data[i] = (uint8_t)i;
    }

#ifndef COMPONENT_CAT3
    iecho->hdr.chksum = inet_chksum(iecho, len);
#endif
}

err_t ping_send(int socket_hnd, const whd_nw_ip_address_t* address, struct icmp_packet *iecho, uint16_t *sequence_number)
{
    int                err;
    struct sockaddr_in to;

    /* Construct ping request */
    ping_prepare_echo(iecho, sizeof(struct icmp_packet), sequence_number);

    /* Send the ping request */
    to.sin_len         = sizeof( to );
    to.sin_family      = AF_INET;
    to.sin_addr.s_addr = address->ip.v4;

    /*
     * If LPA is enabled, invoke the activity callback to resume the network stack,
     * before invoking the LwIP APIs
    */
    if (activity_callback)
    {
        activity_callback(true);
    }
    err = lwip_sendto(socket_hnd, iecho, sizeof(struct icmp_packet), 0, (struct sockaddr*) &to, sizeof(to));

    return (err ? ERR_OK : ERR_VAL);
}

err_t ping_recv(int socket_hnd, whd_nw_ip_address_t* address, uint16_t *ping_seq_num)
{
    char                  buf[PING_RESPONSE_LEN];
    int                   fromlen;
    int                   len;
    struct sockaddr_in    from;
    struct ip_hdr*        iphdr;
    struct icmp_echo_hdr* iecho;
    do
    {
        len = lwip_recvfrom(socket_hnd, buf, sizeof(buf), 0, (struct sockaddr*) &from, (socklen_t*) &fromlen);
        if (len >= (int) (sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr)))
        {
            iphdr = (struct ip_hdr *) buf;
            iecho = (struct icmp_echo_hdr *) (buf + (IPH_HL(iphdr) * 4));

            if ((iecho->id == PING_ID) &&
                 (iecho->seqno == htons(*ping_seq_num)) &&
                 (ICMPH_TYPE(iecho) == ICMP_ER))
            {
                return ERR_OK; /* Echo reply received - return success */
            }
        }
    } while (len > 0);

    return ERR_TIMEOUT; /* No valid echo reply received before timeout */
}
#endif

void internal_ip_change_callback (struct netif *netif)
{
    WPRINT_WHD_INFO(("IP change callback triggered\n"));
    /* Notify ECM about IP address change */
    if(ip_change_callback != NULL)
    {
        for (int i = 0; i < CY_IFACE_MAX_HANDLE; i++)
        {
            if(netif == (struct netif *)iface_context_database[i].nw_interface)
            {
                WPRINT_WHD_INFO(("netif:[%p] iface_context_database[i].nw_interface:[%p]\n", netif, iface_context_database[i].nw_interface));
                ip_change_callback(&(iface_context_database[i]), NULL);
                return;
            }
        }
    }
}

bool is_interface_added(uint8_t interface_index)
{
    return (ip_networking_inited[interface_index & 3]);
}

bool is_network_up(uint8_t interface_index)
{
    return (ip_up[interface_index & 3]);
}

cy_rslt_t is_interface_valid(whd_network_interface_context *iface)
{
    if( (iface == NULL) ||
        (iface->nw_interface == NULL) ||
        (iface->hw_interface == NULL) ||
        (((iface->iface_type != CY_NETWORK_WIFI_STA_INTERFACE) && (iface->iface_type != CY_NETWORK_WIFI_AP_INTERFACE)) &&
        (iface->iface_type != CY_NETWORK_ETH_INTERFACE) && (iface->iface_idx >= MAX_ETHERNET_PORT))
      )
    {
        WPRINT_WHD_ERROR(("iface->iface_type:[%d] iface->iface_idx:[%d]\n", iface->iface_type, iface->iface_idx));
        return CY_RSLT_NETWORK_BAD_ARG;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t whd_network_up(whd_interface_t interface, whd_network_hw_interface_type_t iface_type, whd_network_static_ip_addr_t *static_ip_ptr, whd_network_interface_context **nw_if_ctx)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    if(iface_type == CY_NETWORK_WIFI_STA_INTERFACE)
    {
        if(is_sta_interface_created == false)
        {
            res = whd_network_add_nw_interface(CY_NETWORK_WIFI_STA_INTERFACE, 0, interface, NULL, static_ip_ptr, &whd_nw_sta_if_ctx);
            if (res != CY_RSLT_SUCCESS)
            {
                WPRINT_WHD_ERROR(("failed to add the network interface \n"));
                return res;
            }
            is_sta_interface_created = true;
        }
        if((res = whd_network_ip_up(whd_nw_sta_if_ctx)) != CY_RSLT_SUCCESS)
        {
            WPRINT_WHD_ERROR(("failed to bring up the network stack \n"));
            if(whd_network_remove_nw_interface(whd_nw_sta_if_ctx) != CY_RSLT_SUCCESS)
            {
                WPRINT_WHD_ERROR(("failed to remove the network interface \n"));
            }
            is_sta_interface_created = false;
            return res;
        }
        is_sta_network_up = true;
        *nw_if_ctx = whd_nw_sta_if_ctx;
    }
    else if(iface_type == CY_NETWORK_WIFI_AP_INTERFACE)
    {
        res = whd_network_add_nw_interface(CY_NETWORK_WIFI_AP_INTERFACE, 0, interface, NULL, static_ip_ptr, &whd_nw_ap_if_ctx);
        if (res != CY_RSLT_SUCCESS)
        {
            WPRINT_WHD_ERROR(("failed to add the network interface \n"));
            return res;
        }
        if((res = whd_network_ip_up(whd_nw_ap_if_ctx)) != CY_RSLT_SUCCESS)
        {
            WPRINT_WHD_ERROR(("failed to bring up the network stack \n"));
            if(whd_network_remove_nw_interface(whd_nw_ap_if_ctx) != CY_RSLT_SUCCESS)
            {
                WPRINT_WHD_ERROR(("failed to remove the network interface \n"));
            }
            return res;
        }
        is_ap_network_up = true;
        *nw_if_ctx = whd_nw_ap_if_ctx;
    }
    return res;
}

cy_rslt_t whd_network_ip_assign(whd_interface_t interface, whd_network_interface_context **nw_sta_ifx_ctx)
{
	cy_rslt_t res = CY_RSLT_SUCCESS;
    whd_network_static_ip_addr_t  *static_ip_ptr;
    static_ip_ptr = NULL;
    uint32_t retry_count = 0;
    whd_nw_ip_address_t ipv4_addr;
    whd_net_ip_address_t ip_addr;
    char ip_str[15];
    memset(ip_str, 0, sizeof(ip_str));
    memset(&ip_addr, 0, sizeof(whd_net_ip_address_t));

    if((res = whd_network_up(interface, CY_NETWORK_WIFI_STA_INTERFACE, static_ip_ptr, nw_sta_ifx_ctx)) != CY_RSLT_SUCCESS)
    {
        WPRINT_WHD_ERROR(("Failed to bring up the network stack\n"));
        return res;
    }

    /** wait in busy loop till dhcp starts and ip address gets assigned **/
    while (true)
    {
        res = whd_network_get_ip_address(*nw_sta_ifx_ctx, &ipv4_addr);
        if (res == CY_RSLT_SUCCESS)
        {
#ifdef WHD_ENABLE_INFO
            whd_nw_ntoa(&ipv4_addr, ip_str);
            WPRINT_WHD_INFO(("IPV4 Address %s assigned \n", ip_str));
#endif
            ip_addr.version = CY_NET_IP_VER_V4;
            ip_addr.ip.v4 = ipv4_addr.ip.v4;
            break;
        }
        // TO DO : get ipv6 address
        /* Delay of 10 ms */
        cy_rtos_delay_milliseconds(10);
        /* Increment count for every 10 ms */
        retry_count++;
        /* Return DHCP Timeout Error when it exceeds 6000 * 10 ms = 60 seconds */
        if (retry_count > 6000)
        {
            WPRINT_WHD_ERROR(("DHCP Timeout \n"));
            return CY_RSLT_NETWORK_DHCP_TIMEOUT;
            /* do disconnect to bring network down as DHCP failed */
        }
    }

    return res;
}

void whd_network_down(whd_interface_t interface, whd_network_hw_interface_type_t iface_type)
{
    if(iface_type == CY_NETWORK_WIFI_STA_INTERFACE)
    {
        whd_network_ip_down(whd_nw_sta_if_ctx);
        is_sta_network_up = false;
        whd_network_remove_nw_interface(whd_nw_sta_if_ctx);
        is_sta_interface_created = false;
    }
    else
    {
        whd_network_ip_down(whd_nw_ap_if_ctx);
        whd_network_remove_nw_interface(whd_nw_ap_if_ctx);
        is_ap_network_up = false;
    }
}


#ifdef COMPONENT_4390X
/* CYW43907 kits do not have a TRNG module.
 * The following are the functions to generate pseudorandon numbers.
 * These functions are internal to the AnyCloud library; currently used
 * by Secure Sockets and WCM.
 */
static uint32_t crc32_calc( const uint8_t* buffer, uint16_t buffer_length, uint32_t prev_crc32 )
{
    uint32_t crc32 = ~prev_crc32;
    int i;

    for ( i = 0; i < buffer_length; i++ )
    {
        int j;

        crc32 ^= buffer[ i ];

        for ( j = 0; j < 8; j++ )
        {
            if ( crc32 & 0x1 )
            {
                crc32 = ( crc32 >> 1 ) ^ CY_PRNG_CRC32_POLYNOMIAL;
            }
            else
            {
                crc32 = ( crc32 >> 1 );
            }
        }
    }

    return ~crc32;
}

static uint32_t prng_well512_get_random( void )
{
    /*
     * Implementation of WELL (Well Equidistributed Long-period Linear) pseudorandom number generator.
     * Use the WELL512 source code placed by the inventor to public domain.
     */

    uint32_t a, b, c, d;
    uint32_t result;

    cy_rtos_get_mutex( cy_prng_mutex_ptr , CY_RTOS_NEVER_TIMEOUT);

    a = prng_well512_state[ prng_well512_index ];
    c = prng_well512_state[ ( prng_well512_index + 13 ) & 15 ];
    b = a ^ c ^ ( a << 16 ) ^ ( c << 15 );
    c = prng_well512_state[ ( prng_well512_index + 9 ) & 15 ];
    c ^= ( c >> 11 );
    a = prng_well512_state[ prng_well512_index ] = b ^ c;
    d = a ^ ( ( a << 5 ) & (uint32_t)0xDA442D24UL );
    prng_well512_index = ( prng_well512_index + 15 ) & 15;
    a = prng_well512_state[ prng_well512_index ];
    prng_well512_state[ prng_well512_index ] = a ^ b ^ d ^ ( a << 2 ) ^ ( b << 18 ) ^ ( c << 28 );

    result = prng_well512_state[ prng_well512_index ];

    cy_rtos_set_mutex( cy_prng_mutex_ptr );

    return result;
}

static void prng_well512_add_entropy( const void* buffer, uint16_t buffer_length )
{
    uint32_t crc32[ CY_PRNG_WELL512_STATE_SIZE ];
    uint32_t curr_crc32 = 0;
    unsigned i;

    for ( i = 0; i < CY_PRNG_WELL512_STATE_SIZE; i++ )
    {
        curr_crc32 = crc32_calc( buffer, buffer_length, curr_crc32 );
        crc32[ i ] = curr_crc32;
    }

    cy_rtos_get_mutex( cy_prng_mutex_ptr , CY_RTOS_NEVER_TIMEOUT);

    for ( i = 0; i < CY_PRNG_WELL512_STATE_SIZE; i++ )
    {
        prng_well512_state[ i ] ^= crc32[ i ];
    }

    cy_rtos_set_mutex( cy_prng_mutex_ptr );
}

static bool prng_is_add_cyclecnt_entropy( uint32_t buffer_length )
{
    bool add_entropy = false;

    cy_rtos_get_mutex( cy_prng_mutex_ptr , CY_RTOS_NEVER_TIMEOUT);

    if ( prng_add_cyclecnt_entropy_bytes >= CY_PRNG_ADD_CYCLECNT_ENTROPY_EACH_N_BYTE )
    {
        prng_add_cyclecnt_entropy_bytes %= CY_PRNG_ADD_CYCLECNT_ENTROPY_EACH_N_BYTE;
        add_entropy = true;
    }

    prng_add_cyclecnt_entropy_bytes += buffer_length;

    cy_rtos_set_mutex( cy_prng_mutex_ptr );

    return add_entropy;
}

static void prng_add_cyclecnt_entropy( uint32_t buffer_length )
{
    cy_rslt_t result;
    if ( prng_is_add_cyclecnt_entropy( buffer_length ) )
    {
        cy_time_t cycle_count;
        result = cy_rtos_get_time( &cycle_count );
        if (result == CY_RSLT_SUCCESS)
        {
            prng_well512_add_entropy( &cycle_count, sizeof( cycle_count ) );
        }
    }
    return;
}

cy_rslt_t cy_prng_get_random( void* buffer, uint32_t buffer_length )
{
    uint8_t* p = buffer;

    prng_add_cyclecnt_entropy( buffer_length );

    while ( buffer_length != 0 )
    {
        uint32_t rnd_val = prng_well512_get_random( );
        int      i;

        for ( i = 0; i < 4; i++ )
        {
            *p++ = (uint8_t)( rnd_val & 0xFF );
            if ( --buffer_length == 0 )
            {
                break;
            }
            rnd_val = ( rnd_val >> 8 );
        }
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_prng_add_entropy( const void* buffer, uint32_t buffer_length )
{
    prng_well512_add_entropy( buffer, buffer_length );
    return CY_RSLT_SUCCESS;
}
#endif

#endif /* WHD_NETWORK */
