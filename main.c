/*
 * Copyright (C) 2009 - 2018 Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

#include <stdio.h>

#include "xparameters.h"
#include "xil_exception.h"
#include "netif/xadapter.h"
#include <xgpio.h>
#include "platform.h"
#include "platform_config.h"
#include "xtime_l.h"
#include "sleep.h"
#if defined (__arm__) || defined(__aarch64__)
#include "xil_printf.h"
#endif
#include "xuartns550.h"
#include "lwip/tcp.h"
#include "xil_cache.h"

#if LWIP_IPV6==1
#include "lwip/ip.h"
#else
#if LWIP_DHCP==1
#include "lwip/dhcp.h"
#endif
#endif

#define RECV_BUF_SIZE				10					// RW UART receive buffer
#define RW_TCP_DEBUG 1
#define UART_TIMEOUT_INTERVAL ((XTime)10000 * (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / (2U*1000000U)))
#define IMU_UPDATE_INTERVAL 500
#define GPS_GPIO_ID				XPAR_GPS_PPS_DEVICE_ID // AXI GPIO 2 IP ID for GPIO PPS line
#define PPS_PULSE_WIDTH 	900
extern XGpio GPS_GPIO;
/***************** RW Timeout ****************/
extern int RW1_timeout_flag, RW2_timeout_flag, RW3_timeout_flag, RW4_timeout_flag;
extern int RW1_Start_byte_flag, RW2_Start_byte_flag, RW3_Start_byte_flag, RW4_Start_byte_flag;
extern XTime t_start_RW1, t_start_RW2, t_start_RW3, t_start_RW4;
extern unsigned int server_connected;
extern err_t tcp_receive_message(void);


#if LWIP_IPV6==0
#if LWIP_DHCP==1
extern volatile int dhcp_timoutcntr;
err_t dhcp_start(struct netif *netif);
#endif
#endif


extern u8 RW1_Buffer[];
extern u8 RW2_Buffer[];
extern u8 RW3_Buffer[];
extern u8 RW4_Buffer[];
extern u8 RW1_RecieveBuffer[];
extern u8 RW2_RecieveBuffer[];
extern u8 RW3_RecieveBuffer[];
extern u8 RW4_RecieveBuffer[];
extern u8 GPS_Data_Buffer[RECV_BUF_SIZE];
extern XUartNs550 RW1, RW2, RW3, RW4, GPS_Data;  			/* The instance of the Uart16550 Device */

extern volatile int TcpFastTmrFlag;
extern volatile int TcpSlowTmrFlag;
static struct netif server_netif;
struct netif *echo_netif;

u32 client_connected = 0;
u32 GPS_flag = 0;

/* defined by each RAW mode application */
void print_app_header();
void print_client_header();
int start_application_server();
int start_application_client();
void power_init(void);
int start_application_server();
void IMU_Simulation(void);
int tcp_monitor_application();

extern XGpio RW1_DEBUG;
// DEBUGGING GPIOs
#define RW1_DEBUG_ID			XPAR_RW1_DEBUG_DEVICE_ID	// RW1 DEBUG
err_t gps(void);
void RW_DATA(void);

void tcp_fasttmr(void);
void tcp_slowtmr(void);
u32 wait(u32 useconds);
int uart_init(void);
void RW_Simulation(void);
/* missing declaration in lwIP */
void lwip_init();

int tcp_7001_application();
void data_counter();






#if LWIP_IPV6==1
void print_ip6(char *msg, ip_addr_t *ip)
{
	print(msg);
	xil_printf(" %x:%x:%x:%x:%x:%x:%x:%x\n\r",
			IP6_ADDR_BLOCK1(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK2(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK3(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK4(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK5(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK6(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK7(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK8(&ip->u_addr.ip6));

}
#else
void
print_ip(char *msg, ip_addr_t *ip)
{
	print(msg);
	xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),
			ip4_addr3(ip), ip4_addr4(ip));
}

void
print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{

	print_ip("Board IP: ", ip);
	print_ip("Netmask : ", mask);
	print_ip("Gateway : ", gw);
}
#endif

#if defined (__arm__) && !defined (ARMR5)
#if XPAR_GIGE_PCS_PMA_SGMII_CORE_PRESENT == 1 || XPAR_GIGE_PCS_PMA_1000BASEX_CORE_PRESENT == 1
int ProgramSi5324(void);
int ProgramSfpPhy(void);
#endif
#endif

#ifdef XPS_BOARD_ZCU102
#ifdef XPAR_XIICPS_0_DEVICE_ID
int IicPhyReset(void);
#endif
#endif

int main()
{
	xil_printf("---------Starting RSAT Test Console---------\n");

	XTime tcurrent;
	u32 flag, monitor_count = 0;
	u32 GPS_timer_count = 0;
#if LWIP_IPV6==0
	ip_addr_t ipaddr, netmask, gw;
#endif

	/* the mac address of the board. this should be unique per board */
	unsigned char mac_ethernet_address[] =
	{ 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

	echo_netif = &server_netif;
#if defined (__arm__) && !defined (ARMR5)
#if XPAR_GIGE_PCS_PMA_SGMII_CORE_PRESENT == 1 || XPAR_GIGE_PCS_PMA_1000BASEX_CORE_PRESENT == 1
	ProgramSi5324();
	ProgramSfpPhy();
#endif
#endif

/* Define this board specific macro in order perform PHY reset on ZCU102 */
#ifdef XPS_BOARD_ZCU102
	if(IicPhyReset()) {
		xil_printf("Error performing PHY reset \n\r");
		return -1;
	}
#endif

	init_platform();


	// Power on initialization for IMU and Analog sensors
	power_init();

	#if LWIP_IPV6==0
#if LWIP_DHCP==1
    ipaddr.addr = 0;
	gw.addr = 0;
	netmask.addr = 0;
#else
	/* initliaze IP addresses to be used */
	IP4_ADDR(&ipaddr,  192, 168,   1, 100);
	IP4_ADDR(&netmask, 255, 255, 255,  0);
	IP4_ADDR(&gw,      192, 168,   1,  1);

#endif
#endif

	lwip_init();

#if (LWIP_IPV6 == 0)
	/* Add network interface to the netif_list, and set it as default */
	if (!xemac_add(echo_netif, &ipaddr, &netmask,
						&gw, mac_ethernet_address,
						PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\n\r");
		return -1;
	}

#else
	/* Add network interface to the netif_list, and set it as default */
	if (!xemac_add(echo_netif, NULL, NULL, NULL, mac_ethernet_address,
						PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\n\r");
		return -1;
	}
	echo_netif->ip6_autoconfig_enabled = 1;

	netif_create_ip6_linklocal_address(echo_netif, 1);
	netif_ip6_addr_set_state(echo_netif, 0, IP6_ADDR_VALID);

	print_ip6("\n\rBoard IPv6 address ", &echo_netif->ip6_addr[0].u_addr.ip6);

#endif
	netif_set_default(echo_netif);

	/* now enable interrupts */
	platform_enable_interrupts();

	/* specify that the network if is up */
	netif_set_up(echo_netif);

#if (LWIP_IPV6 == 0)
#if (LWIP_DHCP==1)
	/* Create a new DHCP client for this interface.
	 * Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
	 * the predefined regular intervals after starting the client.
	 */
	dhcp_start(echo_netif);
	dhcp_timoutcntr = 0;

	while(((echo_netif->ip_addr.addr) == 0) && (dhcp_timoutcntr > 0))
		xemacif_input(echo_netif);

	if (dhcp_timoutcntr <= 0) {
		if ((echo_netif->ip_addr.addr) == 0) {
			xil_printf("DHCP Timeout\r\n");
			xil_printf("Configuring default IP of 192.168.1.100\r\n");
			IP4_ADDR(&(echo_netif->ip_addr),  192, 168,   1, 100);
			IP4_ADDR(&(echo_netif->netmask), 255, 255, 255,  0);
			IP4_ADDR(&(echo_netif->gw),      192, 168,   1,  1);
		}
	}

	ipaddr.addr = echo_netif->ip_addr.addr;
	gw.addr = echo_netif->gw.addr;
	netmask.addr = echo_netif->netmask.addr;
#endif

	print_ip_settings(&ipaddr, &netmask, &gw);

#endif

	// Uart Config

	XUartNs550_Recv(&RW1, RW1_RecieveBuffer, 1);


	RW1_Buffer[0] = 'a';

	XUartNs550_Send(&RW1, RW1_Buffer,1);


	XUartNs550_Recv(&RW2, RW2_RecieveBuffer, 1);


	RW2_Buffer[0] = 'b';

	XUartNs550_Send(&RW2, RW2_Buffer,1);


	XUartNs550_Recv(&RW3, RW3_RecieveBuffer, 1);

	RW3_Buffer[0] = 'c';

	XUartNs550_Send(&RW3, RW3_Buffer,1);


	XUartNs550_Recv(&RW4, RW4_RecieveBuffer, 1);


	RW4_Buffer[0] = 'd';

	XUartNs550_Send(&RW4, RW4_Buffer,1);

	XUartNs550_Recv(&GPS_Data, GPS_Data_Buffer, 1);


	GPS_Data_Buffer[0] = 'g';

	XUartNs550_Send(&GPS_Data, GPS_Data_Buffer,1);

/*****************************************/

	start_application_server();


	/* receive and process packets */
	while (1) {

		if(server_connected == 1)
		{
			sleep(1);
			start_application_client();
			sleep(1);
			tcp_monitor_application();
			server_connected = 0;
			client_connected = 1;
		}
/******************* TCP slow timer *******************/
		if (TcpFastTmrFlag) {
			tcp_fasttmr();
			TcpFastTmrFlag = 0;
		}
/******************************************************/
/******************* TCP fast timer *******************/
		if (TcpSlowTmrFlag) {
			monitor_count++;
			GPS_timer_count++;
			monitor_count = monitor_count % 4;			// 2 second timer
			GPS_timer_count = GPS_timer_count % 2;		// 1 second timer

			if(client_connected == 1)
			{
				/****************** for GPS **********************/
				if ((GPS_flag == 0) && (GPS_timer_count == 0)){
					GPS_flag = 1;
					}

				if (GPS_flag) {
					gps();
					XGpio_DiscreteWrite(&GPS_GPIO,1,1);
					usleep(5);
					XGpio_DiscreteWrite(&GPS_GPIO,1,0);
					GPS_flag = 0;
					}
				/****************** for monitoring ****************/
				if ((flag == 0) && (monitor_count == 1)){
					flag = 1;
					}

				if (flag) {
					//data_counter();
					flag = 0;
					}

			}
			tcp_slowtmr();
			TcpSlowTmrFlag = 0;

		}

		xemacif_input(echo_netif);
		/******************* IMU Simulation *******************/
		if (wait(IMU_UPDATE_INTERVAL))
		{
			IMU_Simulation();
		}
		/******************************************************/


		if(client_connected == 1)
		{
			tcp_receive_message();
			RW_Simulation();
			/****************** Reaction wheel timeout ****************/

			// Check if Timeout has occured for the UART handlers
			XTime_GetTime(&tcurrent);

			//if(RW1_Start_byte_flag && ((tcurrent - t_start_RW1) > UART_TIMEOUT_INTERVAL))
			if (RW1_Start_byte_flag && (tcurrent > (t_start_RW1 + UART_TIMEOUT_INTERVAL)))
			{
				RW1_timeout_flag = 1;
			}

			if (RW2_Start_byte_flag && (tcurrent > (t_start_RW2 + UART_TIMEOUT_INTERVAL)))
			{
				RW2_timeout_flag = 1;
			}

			if (RW3_Start_byte_flag && (tcurrent > (t_start_RW3 + UART_TIMEOUT_INTERVAL)))
			{
				RW3_timeout_flag = 1;
			}

			if (RW4_Start_byte_flag && (tcurrent > (t_start_RW4 + UART_TIMEOUT_INTERVAL)))
			{
				RW4_timeout_flag = 1;
			}

		}

	}
	/* never reached */
	cleanup_platform();

	return 0;
}
