#include <stdio.h>
#include <string.h>
#include "lwip/err.h"
#include "lwip/tcp.h"
#include "lwip/inet.h"
#include "xscutimer.h"
#include "xuartns550.h"
#include "xil_exception.h"
#include "processing_var.h"

#if defined (__arm__) || defined (__aarch64__)
#include "xil_printf.h"
#endif

#define TCP_SERVER_IP_ADDRESS "192.168.1.9"
#define PORT 5001

#define TCP_SEND_BUFSIZE 			500
#define TEST_BUFFER_SIZE			64                  // Parser frame includes 64 bytes data


u32 client_status = 0;
int start_application_client();
//extern err_t tcp_print(char *,unsigned int );

void transfer_data(void);
static err_t tcp_send_message(void);

//static char send_buf[TCP_SEND_BUFSIZE];
static struct tcp_pcb *c_pcb;
extern struct tcp_pcb *m_pcb;
static void tcp_client_close(struct tcp_pcb *pcb);

void tcp_receive_message(void);
err_t UI_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
extern unsigned int UI_RW1_Buffer_Count, UI_RW2_Buffer_Count, UI_RW3_Buffer_Count, UI_RW4_Buffer_Count;
extern unsigned char server_connected;
extern u32 client_connected;
extern XUartNs550 RW1, RW2, RW3, RW4, GPS_Data;
/*****************SEND BUFFER TO UI ************************/
extern u8 RW1_SendBuffer[TEST_BUFFER_SIZE];
extern u8 RW2_SendBuffer[TEST_BUFFER_SIZE];
extern u8 RW3_SendBuffer[TEST_BUFFER_SIZE];
extern u8 RW4_SendBuffer[TEST_BUFFER_SIZE];
extern unsigned int UI_RW1_Send_Flag;
u8 GPS_sendBuffer[5];
unsigned int GPS_sendCount = 0;
u8 RW1_UART_BUF[TEST_BUFFER_SIZE];
u8 RW2_UART_BUF[TEST_BUFFER_SIZE];
u8 RW3_UART_BUF[TEST_BUFFER_SIZE];
u8 RW4_UART_BUF[TEST_BUFFER_SIZE];

u8 GPS_Data_Buf[TCP_SEND_BUFSIZE];
/************************** Counter for RW data transfer***********************/
unsigned int RW1_count_zybo = 0;
unsigned int RW2_count_zybo = 0;
unsigned int RW3_count_zybo = 0;
unsigned int RW4_count_zybo = 0;

unsigned int RW1_count_UI = 0;
unsigned int RW2_count_UI = 0;
unsigned int RW3_count_UI = 0;
unsigned int RW4_count_UI = 0;

extern u8 RW_UI_Buf[500];
extern u32 GPS_flag;
/**
 * Find maximum between two numbers.
 */
int max(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

/**
 * Find minimum between two numbers.
 */
int min(int num1, int num2)
{
    return (num1 > num2 ) ? num2 : num1;
}


/** TCP connected callback (active connection), send data now */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	if (err != ERR_OK) {
		tcp_client_close(tpcb);
		xil_printf("Connection error\n\r");
		return err;
	}
    c_pcb= tpcb;

    /* initiate data transfer */
	return ERR_OK;
}

/***************************************************************************************************
                        void transfer_data(void)
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : void
 * Description   : This function transfers the packet from client to the connected server.
 ****************************************************************************************************/

void transfer_data(void){

	tcp_send_message();
}

/***************************************************************************************************
                        err_t UI_recv_callback()
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : returns ERR_CONN or ERR_CONN
 * Description   : Closes the client port if the connection is not established.
 ****************************************************************************************************/

err_t UI_recv_callback(void *arg, struct tcp_pcb *tpcb,
                               struct pbuf *p, err_t err)
{
	//char msg[] = "End of client UI_recv_callback\n";
	unsigned int i,j;
	unsigned int msg_len, msg_fin;
	unsigned char ch;

	u8 UI_RX_BUF[TCP_SEND_BUFSIZE];

	/* do not read the packet if we are not in ESTABLISHED state */
	if (!p) {
		tcp_close(tpcb);
		tcp_recv(tpcb, NULL);
		return ERR_OK;
	}

	/* indicate that the packet has been received */
	tcp_recved(tpcb, p->len);
	memcpy(UI_RX_BUF, p->payload, p->len);

#if RW_UI_DEBUG == 1
	xil_printf("Data from UI: ");
	for(i = 0;i < p->len; i++)
			{	xil_printf("0x%x, ",UI_RX_BUF[i]);
			}
	xil_printf("\n");
#endif

/************************ Data received from UI ***************/
	i = 0;
	j = 0;
	while (i < p->len)
	{
		ch = UI_RX_BUF[i++];

		/***************** GPS ******************************/
		if(ch == 'G')
		{
			//xil_printf("GPS Data = ");
			msg_len = UI_RX_BUF[i++];
			msg_fin = min(i + msg_len, p->len);
			j = i;
			while(i < msg_fin)
			{
				GPS_Data_Buf[i-j] = UI_RX_BUF[i];
				//xil_printf("%x ", GPS_Data[i-j]);
				i++;
			}
			//xil_printf("\n");
			XUartNs550_Send (&GPS_Data, GPS_Data_Buf, msg_len);
		}

		else if (ch == 'P')
		{
			msg_len = UI_RX_BUF[i++];
			msg_fin = min(i + msg_len, p->len);
			j = i;
			//xil_printf("RW1 = ");
			while (i < msg_fin)
			{
				RW1_UART_BUF[i-j] = UI_RX_BUF[i];
				//xil_printf("%x ", RW1_UART_BUF[i-j]);
				i++;
			}
			//xil_printf("\n");
			RW1_count_UI++;							//counter for data received by zybo
			// Send the response to OBC through Uart16550 1
			XUartNs550_Send (&RW1, RW1_UART_BUF, msg_len);
		}
		else if(ch == 'Q')
		{
			msg_len = UI_RX_BUF[i++];
			msg_fin = min(i + msg_len, p->len);
			j = i;
			//xil_printf("RW2 = ");
			while(i < msg_fin)
			{
				RW2_UART_BUF[i-j] = UI_RX_BUF[i];
				//xil_printf("%x ", RW2_UART_BUF[i-j]);
				i++;
			}
			//xil_printf("\n");
			RW2_count_UI++;							//counter for data received by zybo
			// Send the response to OBC through Uart16550 2
			XUartNs550_Send (&RW2, RW2_UART_BUF, msg_len);
		}
		else if(ch == 'R')
		{
			msg_len = UI_RX_BUF[i++];
			msg_fin = min(i + msg_len, p->len);
			j = i;
			//xil_printf("RW3 = ");
			while(i < msg_fin)
			{
				RW3_UART_BUF[i-j] = UI_RX_BUF[i];
				//xil_printf("%x ", RW3_UART_BUF[i-j]);
				i++;
			}
			//xil_printf("\n");
			RW3_count_UI++;							//counter for data received by zybo
			// Send the response to OBC through Uart16550 3
			XUartNs550_Send (&RW3, RW3_UART_BUF, msg_len);

		}

		else if(ch == 'S')
		{
			msg_len = UI_RX_BUF[i++];
			msg_fin = min(i + msg_len, p->len);
			j = i;
			//xil_printf("RW4 = ");
			while(i < msg_fin)
			{
				RW4_UART_BUF[i-j] = UI_RX_BUF[i];
				//xil_printf("%x ", RW4_UART_BUF[i-j]);
				i++;
			}
			//xil_printf("\n");
			RW4_count_UI++;							//counter for data received by zybo

			// Send the response to OBC through Uart16550 4
			XUartNs550_Send (&RW4, RW4_UART_BUF, msg_len);
		}
	}
	//tcp_print(msg, strlen(msg));
	pbuf_free(p);

	return ERR_OK;
}


/***************************************************************************************************
                           static err_t tcp_receive_message(void)
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : returns ERR_CONN or ERR_CONN
 * Description   : Closes the client port if the connection is not established.
 ****************************************************************************************************/
void tcp_receive_message(void)
{
	tcp_recv(c_pcb, UI_recv_callback);
}


/***************************************************************************************************
                           static err_t tcp_send_message(void)
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : returns ERR_CONN or ERR_CONN
 * Description   : Closes the client port if the connection is not established.
 ****************************************************************************************************/
static err_t tcp_send_message(void)
{
	//char msg[] = "End of client tcp_send_message\n";
	err_t err;
	unsigned int rw_wrt_ready = 0;
	u8_t apiflags = TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE;

	if (c_pcb == NULL) {
			return ERR_CONN;
		}

	/*********************** RW1_client_send ******************/
		if (UI_RW1_Buffer_Count > 0 )
			{
				err = tcp_write(c_pcb, RW1_SendBuffer, UI_RW1_Buffer_Count, apiflags);

				rw_wrt_ready = 1;
				if (err != ERR_OK) {
					xil_printf("TCP client: Error on tcp_write: %d\r\n",err);
					return err;
					}
				//xil_printf("RW1_count_zybo = %d\r\n", RW1_count_zybo);
				RW1_count_zybo++;		//counter for data sent from zybo

				UI_RW1_Buffer_Count = 0;
			}

	/*********************** RW2_client_send ******************/
		if(UI_RW2_Buffer_Count > 0 )
				{
					err = tcp_write(c_pcb, RW2_SendBuffer, UI_RW2_Buffer_Count, apiflags);

					rw_wrt_ready = 1;

					if (err != ERR_OK) {
						xil_printf("TCP client: Error on tcp_write: %d\r\n",err);
						return err;
						}
					RW2_count_zybo++;		//counter for data sent from zybo

					UI_RW2_Buffer_Count = 0;
				}

	/*********************** RW3_client_send ******************/
		if(UI_RW3_Buffer_Count > 0)
				{
					err = tcp_write(c_pcb, RW3_SendBuffer, UI_RW3_Buffer_Count, apiflags);

					rw_wrt_ready = 1;
					if (err != ERR_OK) {
						xil_printf("TCP client: Error on tcp_write: %d\r\n",err);
						return err;
						}
					RW3_count_zybo++;		//counter for data sent from zybo

					UI_RW3_Buffer_Count = 0;
				}

	/*********************** RW4_client_send ******************/
		if(UI_RW4_Buffer_Count > 0 )
				{
					err = tcp_write(c_pcb, RW4_SendBuffer, UI_RW4_Buffer_Count, apiflags);

					rw_wrt_ready = 1;
					if (err != ERR_OK) {
						xil_printf("TCP client: Error on tcp_write: %d\r\n",err);
						return err;
						}
					RW4_count_zybo++;		//counter for data sent from zybo

					UI_RW4_Buffer_Count = 0;
				}
	/************************************************************************************************************/

		if (rw_wrt_ready)
		{
			err = tcp_output(c_pcb);
			rw_wrt_ready = 0;
			if (err != ERR_OK) {
				xil_printf("TCP client: Error on tcp_output: %d\r\n",err);
				return err;
				}
		}
		//tcp_print(msg, strlen(msg));
	return 0;
}


/***************************************************************************************************
                           static err_t gps(void)
 ***************************************************************************************************
 * I/P Arguments : none
 * Return value  : returns ERR_CONN or ERR_CONN
 * Description   : This function sends character 'g' to UI for GPS pulse generation. It also expects
 * 					acknowledgment (ACK = 'G') from UI.
 ***************************************************************************************************/
err_t gps(void)
{

	err_t err;
	GPS_sendBuffer[0] = 'g';
	GPS_sendCount = 1;
	u8_t apiflags = TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE;

	if (c_pcb == NULL) {
		return ERR_CONN;
		}

	err = tcp_write(c_pcb, GPS_sendBuffer, GPS_sendCount, apiflags);

	if (err != ERR_OK) {
		xil_printf("TCP client: Error on tcp_write: %d\r\n",err);
		client_connected = 0;
		return err;
		}

	err = tcp_output(c_pcb);
	if (err != ERR_OK) {
		xil_printf("TCP client: Error on tcp_output: %d\r\n",err);
		client_connected = 0;
		return err;
		}

	return 0;
}


/***************************************************************************************************
                           void tcp_client_close()
 ***************************************************************************************************
 * I/P Arguments : none
 * Return value  : none
 * Description   : Closes the client port if the connection is not established.
 ***************************************************************************************************/
/** Close a tcp session */
static void tcp_client_close(struct tcp_pcb *pcb)
{
	err_t err;

	if (pcb != NULL) {
		tcp_sent(pcb, NULL);
		tcp_err(pcb, NULL);
		err = tcp_close(pcb);
		if (err != ERR_OK) {
			/* Free memory with abort */
			tcp_abort(pcb);
		}
	}
}



/***************************************************************************************************
                           int start_application_client()
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : none
 * Description   : This function is responsible to connect client to server
 ****************************************************************************************************/

int start_application_client()
{
	struct tcp_pcb *pcb_c;
	err_t err;
	ip_addr_t remote_addr;

	inet_aton(TCP_SERVER_IP_ADDRESS, &remote_addr);

	/* create new TCP PCB structure */
	pcb_c = tcp_new_ip_type(IPADDR_TYPE_ANY);
	if (!pcb_c) {
		xil_printf("Error creating PCB. Out of Memory\n\r");
		return -1;
	}

	err= tcp_connect(pcb_c, &remote_addr, PORT, tcp_client_connected);

	if (err) {
			xil_printf("Error on tcp_connect: %d\r\n", err);
			tcp_client_close(pcb_c);
			return 0;
		}
	/* we do not need any arguments to callback functions */
	tcp_arg(pcb_c, NULL);
	xil_printf("Client connected to Server at port 5001\n");

	return 0;
}

