#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lwip/err.h"
#include "lwip/tcp.h"
#include "lwip/inet.h"
#include "xscutimer.h"

/* Macro for RW counter debugging....
 *  Setting '1' will enable and '0' will disable the macro execution
*/
#define RW_COUNT_DEBUG 1
#define RW_NACK_COUNT 0


#define TCP_SERVER_IP_ADDRESS "192.168.1.9"
#define PORT_M 8001
//#define TCP_SEND_BUFSIZE 100
#define TEST_BUFFER_SIZE			64                  // Parser frame includes 64 bytes data
int tcp_monitor_application();
err_t tcp_send_m_data(void);
void tcp_client_close(struct tcp_pcb *pcb);
void data_monitoring(void);
err_t tcp_print(char *send_buffer,unsigned int buffer_count);

struct tcp_pcb *m_pcb;
err_t UI_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
extern u8 RW1_SendBuffer[TEST_BUFFER_SIZE];
extern u8 RW2_SendBuffer[TEST_BUFFER_SIZE];
extern u8 RW3_SendBuffer[TEST_BUFFER_SIZE];
extern u8 RW4_SendBuffer[TEST_BUFFER_SIZE];

extern unsigned int UI_RW1_Buffer_Count;
extern unsigned int UI_RW2_Buffer_Count;
extern unsigned int UI_RW3_Buffer_Count;
extern unsigned int UI_RW4_Buffer_Count;
extern u8 RW1_UART_BUF[TEST_BUFFER_SIZE];
extern u8 RW2_UART_BUF[TEST_BUFFER_SIZE];
extern u8 RW3_UART_BUF[TEST_BUFFER_SIZE];
extern u8 RW4_UART_BUF[TEST_BUFFER_SIZE];

extern unsigned int RW1_count_OBC, RW2_count_OBC, RW3_count_OBC, RW4_count_OBC;

extern unsigned int RW1_count_zybo,RW2_count_zybo ,RW3_count_zybo ,RW4_count_zybo ;

extern unsigned int RW1_count_UI, RW2_count_UI, RW3_count_UI, RW4_count_UI;

extern unsigned int Rw1RecvIntrpt, Rw2RecvIntrpt, Rw3RecvIntrpt, Rw4RecvIntrpt;

extern unsigned int RW1_NACK_count, RW1_NACK_timeout_count;
extern unsigned int RW2_NACK_count, RW2_NACK_timeout_count;
extern unsigned int RW3_NACK_count, RW3_NACK_timeout_count;
extern unsigned int RW4_NACK_count, RW4_NACK_timeout_count;


void RW_DATA(void);
void print_monitor_header(){

	xil_printf("\r\n--------------Data monitoring Program---------------------\r\n ");

}
/***************************************************************************************************
                           void tcp_client_close()
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : none
 * Description   : Closes the client port if the connection is not established.
 ****************************************************************************************************/
/** Close a tcp session */
void tcp_client_close(struct tcp_pcb *pcb)
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

/** TCP connected callback (active connection), send data now */
err_t tcp_client_m_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	char message[] = "Hi thanks for accepting the connection\n";
	if (err != ERR_OK) {
		//tcp_client_close(tpcb);
		xil_printf("Connection error\n\r");
		return err;
	}
    m_pcb= tpcb;
    tcp_print(message,sizeof(message));
	/* initiate data transfer */
	return ERR_OK;
}


/***************************************************************************************************
                           static tcp_print(u8 *send_buffer,u8 buffer_count)
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : returns ERR_CONN or ERR_CONN
 * Description   : This function is used instead of xil_printf function for debug purpose.
 ****************************************************************************************************/
err_t tcp_print(char *send_buffer,unsigned int buffer_count)
{
	err_t err;
	u8_t apiflags = TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE;

	if (m_pcb == NULL) {
		return ERR_CONN;
	}

	if(buffer_count > 0)
	{
		//send_buffer[buffer_count] = '\n';
		err = tcp_write(m_pcb, send_buffer, buffer_count, apiflags);
		if (err != ERR_OK) {
			xil_printf("TCP monitor: Error on tcp_write: %d\r\n",err);
			return err;
		}

		err = tcp_output(m_pcb);
		if (err != ERR_OK) {
			xil_printf("TCP monitor: Error on tcp_output: %d\r\n",err);
			return err;
			}
	}
	return 0;
}


/***************************************************************************************************
                           int start_application_client()
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : none
 * Description   : This function is responsible to connect client to server
 ****************************************************************************************************/


int tcp_monitor_application()
{
	struct tcp_pcb *pcb_m;
		err_t err;
		ip_addr_t remote_addr;

		inet_aton(TCP_SERVER_IP_ADDRESS, &remote_addr);

		/* create new TCP PCB structure */
		pcb_m = tcp_new_ip_type(IPADDR_TYPE_ANY);
		if (!pcb_m) {
			xil_printf("Error creating PCB. Out of Memory\n\r");
			return -1;
		}

		err= tcp_connect(pcb_m, &remote_addr, PORT_M, tcp_client_m_connected);

		if (err) {
				xil_printf("Error on tcp_connect: %d\r\n", err);
				tcp_client_close(pcb_m);
				return 0;
			}

		/* we do not need any arguments to callback functions */
		tcp_arg(pcb_m, NULL);
		//xil_printf("Client connected to Server at port 5001\n");
		return 0;
}


/***************************************************************************************************
                           void data_counter(void)
 ****************************************************************************************************
 * I/P Arguments : none
 * Return value  : none
 * Description   : This function will increment the count of data transmission
 ****************************************************************************************************/

void data_counter()
{
#if RW_COUNT_DEBUG == 1

	char send_buf_OBC[300], send_buf_ZYBO[300], send_buf_UI[300], send_buf_INT[300];
	int length_obc, length_zybo, length_UI, length_INT;
	char RW1_NACK_BUF[100], RW2_NACK_BUF[100], RW3_NACK_BUF[100], RW4_NACK_BUF[100];
	unsigned int RW1_len, RW2_len, RW3_len, RW4_len;

	sprintf(send_buf_OBC, "OBC: %010d, %010d, %010d, %010d\n",
	    		RW1_count_OBC, RW2_count_OBC, RW3_count_OBC, RW4_count_OBC);
	sprintf(send_buf_ZYBO, "ZYB: %010d, %010d, %010d, %010d\n",
	    		RW1_count_zybo, RW2_count_zybo, RW3_count_zybo, RW4_count_zybo);
	sprintf(send_buf_UI, "UI_: %010d, %010d, %010d, %010d\n",
	    		RW1_count_UI, RW2_count_UI, RW3_count_UI, RW4_count_UI);
	sprintf(send_buf_INT, "INT: %010d, %010d, %010d, %010d\n\n",
	    		Rw1RecvIntrpt, Rw2RecvIntrpt, Rw3RecvIntrpt, Rw4RecvIntrpt);

	sprintf(RW1_NACK_BUF,"RW1: NACK_COUNT = %010d  NACK_TIMEOUT_COUNT = %010d\r\n", RW1_NACK_count, RW1_NACK_timeout_count);
	sprintf(RW2_NACK_BUF,"RW2: NACK_COUNT = %010d  NACK_TIMEOUT_COUNT = %010d\r\n", RW2_NACK_count, RW2_NACK_timeout_count);
	sprintf(RW3_NACK_BUF,"RW3: NACK_COUNT = %010d  NACK_TIMEOUT_COUNT = %010d\r\n", RW3_NACK_count, RW3_NACK_timeout_count);
	sprintf(RW4_NACK_BUF,"RW4: NACK_COUNT = %010d  NACK_TIMEOUT_COUNT = %010d\r\n", RW4_NACK_count, RW4_NACK_timeout_count);

	length_obc = strlen(send_buf_OBC);
	length_zybo = strlen(send_buf_ZYBO);
	length_UI = strlen(send_buf_UI);
	length_INT = strlen(send_buf_INT);
	RW1_len = strlen(RW1_NACK_BUF);
	RW2_len = strlen(RW2_NACK_BUF);
	RW3_len = strlen(RW3_NACK_BUF);
	RW4_len = strlen(RW4_NACK_BUF);

	tcp_print(send_buf_OBC, length_obc);
	tcp_print(send_buf_ZYBO, length_zybo);
	tcp_print(send_buf_UI, length_UI);
	tcp_print(send_buf_INT, length_INT);
	tcp_print(RW1_NACK_BUF, RW1_len);
	tcp_print(RW2_NACK_BUF, RW2_len);
	tcp_print(RW3_NACK_BUF, RW3_len);
	tcp_print(RW4_NACK_BUF, RW4_len);

#endif

#if RW_NACK_COUNT == 1

	char RW1_NACK_BUF[50], RW2_NACK_BUF[50], RW3_NACK_BUF[50], RW4_NACK_BUF[50];
	unsigned int RW1_len, RW2_len, RW3_len, RW4_len;
	sprintf(RW1_NACK_BUF,"RW1: NACK_COUNT = %d  NACK_TIMEOUT_COUNT = %d\r\n", RW1_NACK_count, RW1_NACK_timeout_count);
	sprintf(RW2_NACK_BUF,"RW2: NACK_COUNT = %d  NACK_TIMEOUT_COUNT = %d\r\n", RW2_NACK_count, RW2_NACK_timeout_count);
	sprintf(RW3_NACK_BUF,"RW3: NACK_COUNT = %d  NACK_TIMEOUT_COUNT = %d\r\n", RW3_NACK_count, RW3_NACK_timeout_count);
	sprintf(RW4_NACK_BUF,"RW4: NACK_COUNT = %d  NACK_TIMEOUT_COUNT = %d\r\n", RW4_NACK_count, RW4_NACK_timeout_count);

	RW1_len = strlen(RW1_NACK_BUF);
	RW2_len = strlen(RW2_NACK_BUF);
	RW3_len = strlen(RW3_NACK_BUF);
	RW4_len = strlen(RW4_NACK_BUF);

	tcp_print(RW1_NACK_BUF, RW1_len);
	tcp_print(RW2_NACK_BUF, RW2_len);
	tcp_print(RW3_NACK_BUF, RW3_len);
	tcp_print(RW4_NACK_BUF, RW4_len);

#endif

}
