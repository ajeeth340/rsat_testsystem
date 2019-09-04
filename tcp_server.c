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
#include <string.h>

#include "lwip/err.h"
#include "lwip/tcp.h"
#if defined (__arm__) || defined (__aarch64__)
#include "xil_printf.h"
#endif
unsigned int server_connected = 0;
// variable for capturing data from UI
u8 UI_Buff[1000];

// Acknowledge and Negative acknowledge variable
u8 S_ack = 'A';		// Sun sensor acknowledgment
u8 T_ack = 'B';		// Thermistor acknowledgment
u8 I_ack = 'C';		// IMU acknowledgment
u8 nack  = 'e';		// Negative acknowledgment
u8 UI_ack;

// variables required for parsing data from UI
u8 Sun_Buff[500];
u8 therm_Buff[500];
u8 IMU_Buff[500];
u32 UI_connected = 0;

// Function to parse data
u8 Parse_data(u8 *Buff);
void parse_process_Sun(u8 *S_Buff, int len);
void parse_process_Ther(u8 *T_Buff, int len);
void parse_process_IMU(u8 *I_Buff, int len);


void Analog_sensor(u8 channel,u8 data1,u8 data2);	 // Analog sensors data processing and data transmission to DAC
void imu_data_processing(u8 Channel_No, u16 data);
void start_application_client();
extern err_t tcp_print(char *,unsigned int );
err_t recv_callback(void *arg, struct tcp_pcb *tpcb,
                               struct pbuf *p, err_t err)
{
	//char msg[] = "End of Server recv_callback\n";
	/* do not read the packet if we are not in ESTABLISHED state */
	if (!p) {
		tcp_close(tpcb);
		tcp_recv(tpcb, NULL);
		return ERR_OK;
	}

	/* indicate that the packet has been received */
	tcp_recved(tpcb, p->len);

	/***** Copy all the data from UI and store it in the UI_Buff ******/
	memcpy(UI_Buff, p->payload, p->len);

	// Copy all the data to respective buffers
	UI_ack = Parse_data(UI_Buff);

	/* reply back the acknowledgment */
	tcp_write(tpcb, &UI_ack, 1, 1);

	//tcp_print(msg, strlen(msg));
	/* free the received pbuf */
	pbuf_free(p);

	return ERR_OK;
}

err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	static int connection = 1;
	server_connected = 1;
	/* set the receive callback for this connection */
	tcp_recv(newpcb, recv_callback);

	/* just use an integer number indicating the connection id as the
	   callback argument */
	tcp_arg(newpcb, (void*)(UINTPTR)connection);

	/* increment for subsequent accepted connections */
	connection++;
	return ERR_OK;
}


int start_application_server()
{
	struct tcp_pcb *pcb;
	err_t err;
	unsigned port = 4001;

	/* create new TCP PCB structure */
	pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
	if (!pcb) {
		xil_printf("Error creating PCB. Out of Memory\n\r");
		return -1;
	}

	/* bind to specified @port */
	err = tcp_bind(pcb, IP_ANY_TYPE, port);
	if (err != ERR_OK) {
		xil_printf("Unable to bind to port %d: err = %d\n\r", port, err);
		return -2;
	}

	/* we do not need any arguments to callback functions */
	tcp_arg(pcb, NULL);

	/* listen for connections */
	pcb = tcp_listen(pcb);
	if (!pcb) {
		xil_printf("Out of memory while tcp_listen\n\r");
		return -3;
	}

	/* specify callback to use for incoming connections */
	tcp_accept(pcb, accept_callback);

	xil_printf("TCP server started @ port %d\n\r", port);

	return 0;
}

/**********************************************************************/
u8 Parse_data(u8 *Buff){
	char msg[] = "End of Server Parse_data\n";
	int i,len;
	u8 a;
	if (Buff[0] == 0x61){
		len = Buff[1];
		for (i = 0; i < len; i++){
			Sun_Buff[i] = Buff[i+2];
		}
		parse_process_Sun(Sun_Buff, len);
		a = S_ack;
	}

	else if (Buff[0] == 0x62){
		len = Buff[1];
		for (i = 0; i < len; i++){
			therm_Buff[i] = Buff[i+2];
		}
		parse_process_Ther(therm_Buff, len);
		a = T_ack;
	}


	else if (Buff[0] == 0x63){
		len = Buff[1];
		for (i = 0; i < len; i++){
			IMU_Buff[i] = Buff[i+2];
//			xil_printf("IMU_Buffer = %x\n", IMU_Buff[i]);
		}
		parse_process_IMU(IMU_Buff, len);
		a = I_ack;
	}

	else
		a = nack;

	//tcp_print(msg, strlen(msg));
	return a;
}

/**********************************************************************/
void parse_process_Sun(u8 *S_Buff, int len){

	char msg[] = "End of Server parse_process_Sun\n";
	u8 Analog_Sensor_LSB, Analog_Sensor_MSB;
	u8 ChannelNum;
	int i, j, num_data;
	num_data = len/3;
	j = 0;
	for (i = 0; i < num_data; i++){

		ChannelNum = S_Buff[j];
		Analog_Sensor_MSB = S_Buff[j+1];
		Analog_Sensor_LSB = S_Buff[j+2];
		Analog_sensor(ChannelNum, Analog_Sensor_MSB, Analog_Sensor_LSB);

		j += 3;
	}
	//tcp_print(msg, strlen(msg));
}

/**********************************************************************/
void parse_process_Ther(u8 *T_Buff, int len){

	char msg[] = "End of Server parse_process_Ther\n";
	u8 Analog_Sensor_LSB,Analog_Sensor_MSB;
	u8 ChannelNum;
	int i, j, num_data;
	num_data = len/3;
	j = 0;
	for (i = 0; i < num_data; i++){

		ChannelNum = T_Buff[j];
		Analog_Sensor_MSB = T_Buff[j+1];
		Analog_Sensor_LSB = T_Buff[j+2];

		Analog_sensor(ChannelNum, Analog_Sensor_MSB, Analog_Sensor_LSB);
		j += 3;
	}
	//tcp_print(msg, strlen(msg));
}

/**********************************************************************/
void parse_process_IMU(u8 *I_Buff, int len){

	char msg[] = "End of Server parse_process_IMU\n";
	u8 ChannelNum;
	u16 SensorData;
	int i, j, num_data;
	num_data = len/3;
	j = 0;
	for (i = 0; i < num_data; i++){

		ChannelNum = I_Buff[j];
		SensorData = (I_Buff[j+1]<<8)|(I_Buff[j+2]);
		imu_data_processing(ChannelNum,SensorData);
		j += 3;
	}
	//tcp_print(msg, strlen(msg));
}

