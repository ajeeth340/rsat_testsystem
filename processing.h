/*
 * processing.h
 *
 *  Created on: Jun 28, 2019
 *      Author: Ajeeth
 */


#include "xscugic.h"
#include "xscutimer.h"
#include "xuartns550.h"
#include <xgpio.h>
#include "xspips.h"
#include "xspi.h"
#include "xtime_l.h"
#include "sleep.h"

#ifndef SRC_PROCESSING_H_
#define SRC_PROCESSING_H_

/**************************** Definitions of Interface  *******************************/

#define RW1_ID					XPAR_RW1_DEVICE_ID // Uart16550 IP 0 ID for RW1
#define RW2_ID					XPAR_RW2_DEVICE_ID // Uart16550 IP 1 ID for RW2
#define RW3_ID					XPAR_RW3_DEVICE_ID // Uart16550 IP 2 ID for RW3
#define RW4_ID					XPAR_RW4_DEVICE_ID // Uart16550 IP 3 ID for RW4
#define RW1_IRQ				  	XPAR_FABRIC_RW1_IP2INTC_IRPT_INTR // Interrupt generated ID for Reaction wheel 1
#define RW2_IRQ 				XPAR_FABRIC_RW2_IP2INTC_IRPT_INTR // Interrupt generated ID for Reaction wheel 2
#define RW3_IRQ 				XPAR_FABRIC_RW3_IP2INTC_IRPT_INTR // Interrupt generated ID for Reaction wheel 3
#define RW4_IRQ 				XPAR_FABRIC_RW4_IP2INTC_IRPT_INTR // Interrupt generated ID for Reaction wheel 4

#define RW_POLY				 	0x8408

#define GPS_Data_ID				XPAR_GPS_DATA_DEVICE_ID // Uart16550 IP 4 ID for GPS_Data
#define GPS_Data_IRQ			XPAR_FABRIC_GPS_DATA_IP2INTC_IRPT_INTR
/************************** Variable Definitions *****************************/
#define BUFFER_SIZE					17
#define BUFFER_COUNT				13
#define RECV_BUF_SIZE				10					// RW UART receive buffer
#define TEST_BUFFER_SIZE			64                  // Parser frame includes 64 bytes data

#define GPS_GPIO_ID				XPAR_GPS_PPS_DEVICE_ID 		// AXI GPIO 2 IP ID for GPIO PPS line
#define DAC_GPIO_ID				XPAR_DAC_GPIO_DEVICE_ID 	// AXI GPIO 0 IP ID for DAC lines
#define IMU_DR_GPIO_ID			XPAR_IMU_DR_DEVICE_ID		// AXI GPIO 1 IP ID for SPI data ready lines
#define IMU_ID					XPAR_SPI_0_DEVICE_ID // Quad SPI IP ID

#define GPS_BUFFER_SIZE			100

/* Global Timer is always clocked at half of the CPU frequency */
#define COUNTS_PER_USECOND  (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / (2U*1000000U))
#define INTC		XScuGic
#define INTC_HANDLER	XScuGic_InterruptHandler
#define INTC_DEVICE_ID XPAR_PS7_SCUGIC_0_DEVICE_ID

#define TIMER_DEVICE_ID		XPAR_XSCUTIMER_0_DEVICE_ID
#define INTC_BASE_ADDR		XPAR_SCUGIC_0_CPU_BASEADDR
#define INTC_DIST_BASE_ADDR	XPAR_SCUGIC_0_DIST_BASEADDR
#define TIMER_IRPT_INTR		XPAR_SCUTIMER_INTR
/***************** Variable Declaration and initialization *****************/

int RW1_Start_byte_flag=0;
int	RW2_Start_byte_flag=0;
int RW3_Start_byte_flag=0;
int RW4_Start_byte_flag=0;

int RW1_timeout_flag = 0;
int RW2_timeout_flag = 0;
int RW3_timeout_flag = 0;
int RW4_timeout_flag = 0;

// DEBUGGING GPIOs
#define RW1_DEBUG_ID			XPAR_RW1_DEBUG_DEVICE_ID	// RW1 DEBUG
#define RW2_DEBUG_ID			XPAR_RW2_DEBUG_DEVICE_ID	// RW2 DEBUG
#define RW3_DEBUG_ID			XPAR_RW3_DEBUG_DEVICE_ID	// RW3 DEBUG
#define RW4_DEBUG_ID			XPAR_RW4_DEBUG_DEVICE_ID	// RW4 DEBUG

XGpio RW1_DEBUG, RW2_DEBUG, RW3_DEBUG, RW4_DEBUG; // Instance for RW DEBUGGING


unsigned int RW1_Frame_complete_flag=0;
unsigned int RW2_Frame_complete_flag=0;
unsigned int RW3_Frame_complete_flag=0;
unsigned int RW4_Frame_complete_flag=0;

u8 RW_UI_Buf[500];
int RW1_ReceivedCount=0;
int RW2_ReceivedCount=0;
int RW3_ReceivedCount=0;
int RW4_ReceivedCount=0;

int Status = 0;

XTime t_start_RW1, t_start_RW2, t_start_RW3, t_start_RW4;

u16 IMU_Data[BUFFER_SIZE];

/*****************RECEIVE BUFFER FROM UART******************/
u8 RW1_RecieveBuffer[RECV_BUF_SIZE];
u8 RW2_RecieveBuffer[RECV_BUF_SIZE];
u8 RW3_RecieveBuffer[RECV_BUF_SIZE];
u8 RW4_RecieveBuffer[RECV_BUF_SIZE];

u8 RW1_Buffer[TEST_BUFFER_SIZE];
u8 RW2_Buffer[TEST_BUFFER_SIZE];
u8 RW3_Buffer[TEST_BUFFER_SIZE];
u8 RW4_Buffer[TEST_BUFFER_SIZE];

u8 GPS_Data_Buffer[RECV_BUF_SIZE];
u8 GPS_Data_Buffer[];

/*****************SEND BUFFER TO UI ************************/
u8 RW1_SendBuffer[TEST_BUFFER_SIZE];
u8 RW2_SendBuffer[TEST_BUFFER_SIZE];
u8 RW3_SendBuffer[TEST_BUFFER_SIZE];
u8 RW4_SendBuffer[TEST_BUFFER_SIZE];

/******************* Counter Declaration for RW data transfer **************************/

unsigned int RW1_NACK_count, RW1_NACK_timeout_count;
unsigned int RW2_NACK_count, RW2_NACK_timeout_count;
unsigned int RW3_NACK_count, RW3_NACK_timeout_count;
unsigned int RW4_NACK_count, RW4_NACK_timeout_count;

unsigned int RW1_count_OBC, RW2_count_OBC, RW3_count_OBC, RW4_count_OBC;

unsigned int Rw1RecvIntrpt, Rw2RecvIntrpt, Rw3RecvIntrpt, Rw4RecvIntrpt;


static volatile int TotalSentCount; // Uart PS Sentcount variable
static volatile int TotalReceivedCount; // Uart PS Recvcount vaiable


unsigned int UI_RW1_Buffer_Count = 0;
unsigned int UI_RW2_Buffer_Count = 0;
unsigned int UI_RW3_Buffer_Count = 0;
unsigned int UI_RW4_Buffer_Count = 0;
/*
 * The following counters are used to determine when the entire buffer has
 * been sent and received.
 */

volatile int RW1_SentCount;
volatile int RW2_SentCount;
volatile int RW3_SentCount;
volatile int RW4_SentCount;

int TotalErrorCount;
typedef signed char err_t;

/**************** UART16550 handler variable declaration *******************************/

u8 Errors_1, Errors_2, Errors_3, Errors_4;

/***************************************** Drivers instance Variables ********************************************************************/

XScuGic InterruptController;				// Instance of the Interrupt Controller
XScuGic INTCInst;							// The instance of the Timer Driver
INTC IntcInstance;							/* The instance of the Interrupt Controller */
XScuTimer Timer;							// Cortex A9 SCU Private Timer Instance
XUartNs550 RW1, RW2, RW3, RW4, GPS_Data;  	/* The instance of the Uart16550 Device */
XSpiPs DAC_SPI_Instance;					// DAC_SPI_MASTER instance of the SPI Driver
XSpi  IMU_SPI_Instance;  					// Instance of the Quad SPI Driver */
XGpio DAC_GPIO,IMU_DR,GPS_GPIO;				// The instance of the GPIO Drivers
XGpio RW1_DEBUG; // Instance for RW DEBUGGING



/****************************************** Function Declaration**************************************************************************/

int Timer_Init(u16 DeviceId);


/*********************************** Handler declaration for Reaction Wheel****************************/
void UartNs550IntrHandler_RW1(void *CallBackRef, u32 Event, unsigned int EventData);
void UartNs550IntrHandler_RW2(void *CallBackRef, u32 Event, unsigned int EventData);
void UartNs550IntrHandler_RW3(void *CallBackRef, u32 Event, unsigned int EventData);
void UartNs550IntrHandler_RW4(void *CallBackRef, u32 Event, unsigned int EventData);
void UartNs550IntrHandler_GPS_Data(void *CallBackRef, u32 Event, unsigned int EventData);

static void RW1SendHandler(void *CallBackRef, unsigned int EventData);
static void RW1RecvHandler(void *CallBackRef, unsigned int EventData);

static void RW2SendHandler(void *CallBackRef, unsigned int EventData);
static void RW2RecvHandler(void *CallBackRef, unsigned int EventData);

static void RW3SendHandler(void *CallBackRef, unsigned int EventData);
static void RW3RecvHandler(void *CallBackRef, unsigned int EventData);

static void RW4SendHandler(void *CallBackRef, unsigned int EventData);
static void RW4RecvHandler(void *CallBackRef, unsigned int EventData);

static void GPS_DataSendHandler(void *CallBackRef, unsigned int EventData);
static void GPS_DataRecvHandler(void *CallBackRef, unsigned int EventData);

void Analog_SPI_intialization(void);
void Spi_Load_FIFO(XSpi *InstancePtr, u16 SendBufPtr[],  int ByteCount);
void IMU_Simulation(void);
void imu_data_processing(u8 Channel_No, u16 data);
void Analog_sensor(u8 channel,u8 data1,u8 data2);
/*********************************** Reaction wheel simulation function declaration ********************/
void RW_Simulation(void);
void RW1_Simulation(void);
void RW2_Simulation(void);
void RW3_Simulation(void);
void RW4_Simulation(void);

int platform_setup_interrupts(void);

void RW_UI_SND(void);
void RW_process_data(u8 uart_send_buf[TEST_BUFFER_SIZE], u32 numbytes);
void transfer_data(void);
extern err_t tcp_print(char *send_buffer,unsigned int buffer_count);


#endif /* SRC_PROCESSING_H_ */
