#include "xscugic.h"
#include "xparameters.h"
#include <string.h>
#include "xscutimer.h"
#include "xuartns550.h"
#include "xil_exception.h"
#include "processing.h"
#include "processing_var.h"
#include "platform.h"
#include <xgpio.h>
#include "xspi.h"		/* SPI device driver */
#include "xspips.h"		/* SPI device driver */
#include "xtime_l.h"
#include "xil_types.h"
#include "xpseudo_asm.h"
#include "xreg_cortexa9.h"
#include "xil_io.h"
#include "xtime_l.h"
#include "sleep.h"
/* Global Timer is always clocked at half of the CPU frequency */
#define COUNTS_PER_USECOND  (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / (2U*1000000U))

#define XPAR_GPS_DEFAULT_BAUD_RATE 9600
#define XUN_FORMAT_8_BITS		3 /**< 8 data bits */
#define XUN_FORMAT_ODD_PARITY		1 /**< Odd Parity */
#define XUN_FORMAT_1_STOP_BIT		0 /**< 1 stop bit */
static XScuTimer TimerInstance;
extern XScuGic_Config XScuGic_ConfigTable[XPAR_XSCUGIC_NUM_INSTANCES];



int platform_setup_interrupts(void)
{
	u16 Options;
	//char msg[] = "End of platform_setup_interrupts\n";
	XUartNs550 *Uart16550Ptr1, *Uart16550Ptr2, *Uart16550Ptr3, *Uart16550Ptr4, *Uart16550Ptr5;

	XUartNs550Format gps_format =
	{
		XPAR_GPS_DEFAULT_BAUD_RATE,
		XUN_FORMAT_8_BITS,
		XUN_FORMAT_ODD_PARITY,
		XUN_FORMAT_1_STOP_BIT

	};
	//*Uart16550Ptr3

	XScuGic_Config *IntcConfig;
	INTC *IntcInstancePtr;


	Uart16550Ptr1 = &RW1;
	Uart16550Ptr2 = &RW2;
	Uart16550Ptr3 = &RW3;
	Uart16550Ptr4 = &RW4;
	Uart16550Ptr5 = &GPS_Data;


	IntcInstancePtr = &IntcInstance;

	XScuGic_DeviceInitialize(INTC_DEVICE_ID);

	// Initialize the interrupt controller driver so that it is ready to use.

	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
		}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,	IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS)
		{
			return XST_FAILURE;
		}
//  XUN_OPTION_FIFOS_ENABLE |
	 Options = XUN_OPTION_DATA_INTR | XUN_OPTION_RESET_TX_FIFO | XUN_OPTION_RESET_RX_FIFO;
/********************************************* RW1 (Uart16550 1) initialization ***************************************/

	// Initialize the Uart16550 driver so that it's ready to use.

	Status = XUartNs550_Initialize(Uart16550Ptr1, RW1_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Set 1 Byte threshold level for FIFO.

	XUartNs550_SetFifoThreshold(Uart16550Ptr1, XUN_FIFO_TRIGGER_01);

	// Perform a self-test to ensure that the hardware was built correctly.

	Status = XUartNs550_SelfTest(Uart16550Ptr1);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, RW1_IRQ, 0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, RW1_IRQ,
			(Xil_ExceptionHandler)XUartNs550_InterruptHandler, Uart16550Ptr1);
	if (Status != XST_SUCCESS) {
		return Status;
		}
	/*
	 * Enable the interrupt for the Uart16550 device 1.
	 */
	XScuGic_Enable(IntcInstancePtr, RW1_IRQ);

	/*
	 * Setup the handlers for the UART that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the UART driver instance as the callback reference so
	 * the handlers are able to access the instance data.
	 */
	 XUartNs550_SetHandler(Uart16550Ptr1, UartNs550IntrHandler_RW1 , Uart16550Ptr1);

	 /*
	  * Enable the interrupt of the UART so interrupts will occur,keep the
	  * FIFOs enabled, clear RX_FIFO AND TX_FIFO.
	  */
	 XUartNs550_SetOptions(Uart16550Ptr1, Options);


/********************************************* RW2 (Uart16550 2) initialization ***************************************/

	// Initialize the Uart16550 driver so that it's ready to use.

	Status = XUartNs550_Initialize(Uart16550Ptr2, RW2_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	// Set 1 Byte threshold level for FIFO.

	XUartNs550_SetFifoThreshold(Uart16550Ptr2, XUN_FIFO_TRIGGER_01);
	// Perform a self-test to ensure that the hardware was built correctly.

	Status = XUartNs550_SelfTest(Uart16550Ptr2);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, RW2_IRQ, 0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, RW2_IRQ,
			(Xil_ExceptionHandler)XUartNs550_InterruptHandler, Uart16550Ptr2);
	if (Status != XST_SUCCESS) {
		return Status;
		}
	/*
	 * Enable the interrupt for the Uart16550 device 2.
	 */
	XScuGic_Enable(IntcInstancePtr, RW2_IRQ);

	/*
	 * Setup the handlers for the UART that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the UART driver instance as the callback reference so
	 * the handlers are able to access the instance data.
	 */
	XUartNs550_SetHandler(Uart16550Ptr2, UartNs550IntrHandler_RW2 , Uart16550Ptr2);

	 /*
	  * Enable the interrupt of the UART so interrupts will occur,keep the
	  * FIFOs enabled, clear RX_FIFO AND TX_FIFO.
	  */
	XUartNs550_SetOptions(Uart16550Ptr2, Options);



/********************************************* RW3 (Uart16550 3) initialization ***************************************/


	// Initialize the Uart16550 driver so that it's ready to use.

		Status = XUartNs550_Initialize(Uart16550Ptr3, RW3_ID);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		// Set 1 Byte threshold level for FIFO.

		XUartNs550_SetFifoThreshold(Uart16550Ptr3, XUN_FIFO_TRIGGER_01);

		// Perform a self-test to ensure that the hardware was built correctly.

		Status = XUartNs550_SelfTest(Uart16550Ptr3);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}


		XScuGic_SetPriorityTriggerType(IntcInstancePtr, RW3_IRQ, 0xA0, 0x3);


		 // Connect the interrupt handler that will be called when an
		 // interrupt occurs for the device.

		Status = XScuGic_Connect(IntcInstancePtr, RW3_IRQ,
				(Xil_ExceptionHandler)XUartNs550_InterruptHandler, Uart16550Ptr3);
		if (Status != XST_SUCCESS) {
			return Status;
			}

		 // Enable the interrupt for the Uart16550 device 3.

		XScuGic_Enable(IntcInstancePtr, RW3_IRQ);



		 // Setup the handlers for the UART that will be called from the
		 // interrupt context when data has been sent and received, specify a
		 // pointer to the UART driver instance as the callback reference so
		 // the handlers are able to access the instance data.
		 //
		XUartNs550_SetHandler(Uart16550Ptr3, UartNs550IntrHandler_RW3 , Uart16550Ptr3);

	/*
	 * Enable the interrupt of the UART so interrupts will occur,keep the
	 * FIFOs enabled, clear RX_FIFO AND TX_FIFO.
	*/
		XUartNs550_SetOptions(Uart16550Ptr3, Options);


/********************************************* RW4 (Uart16550 4) initialization ***************************************/

	// Initialize the Uart16550 driver so that it's ready to use.

	Status = XUartNs550_Initialize(Uart16550Ptr4, RW4_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Set 1 Byte threshold level for FIFO.

	XUartNs550_SetFifoThreshold(Uart16550Ptr4, XUN_FIFO_TRIGGER_01);

	// Perform a self-test to ensure that the hardware was built correctly.

	Status = XUartNs550_SelfTest(Uart16550Ptr4);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, RW4_IRQ, 0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, RW4_IRQ,
			(Xil_ExceptionHandler)XUartNs550_InterruptHandler, Uart16550Ptr4);
	if (Status != XST_SUCCESS) {
		return Status;
		}
	/*
	 * Enable the interrupt for the Uart16550 device 4.
	 */
	XScuGic_Enable(IntcInstancePtr, RW4_IRQ);

	/*
	 * Setup the handlers for the UART that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the UART driver instance as the callback reference so
	 * the handlers are able to access the instance data.
	 */
	XUartNs550_SetHandler(Uart16550Ptr4, UartNs550IntrHandler_RW4 , Uart16550Ptr4);


	 /*
	  * Enable the interrupt of the UART so interrupts will occur,keep the
	  * FIFOs enabled, clear RX_FIFO AND TX_FIFO.
	  */
	XUartNs550_SetOptions(Uart16550Ptr4, Options);




/********************************************* GPS_Data (Uart16550 5) initialization ***************************************/

		// Initialize the Uart16550 driver so that it's ready to use.

		Status = XUartNs550_Initialize(Uart16550Ptr5, GPS_Data_ID);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		// Set 1 Byte threshold level for FIFO.

		XUartNs550_SetFifoThreshold(Uart16550Ptr5, XUN_FIFO_TRIGGER_01);

		// Perform a self-test to ensure that the hardware was built correctly.

		Status = XUartNs550_SelfTest(Uart16550Ptr5);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		XScuGic_SetPriorityTriggerType(IntcInstancePtr, GPS_Data_IRQ, 0xA0, 0x3);

		XUartNs550_SetDataFormat(Uart16550Ptr5, &gps_format);

		/*
		 * Connect the interrupt handler that will be called when an
		 * interrupt occurs for the device.
		 */
		Status = XScuGic_Connect(IntcInstancePtr, GPS_Data_IRQ,
				(Xil_ExceptionHandler)XUartNs550_InterruptHandler, Uart16550Ptr5);
		if (Status != XST_SUCCESS) {
			return Status;
			}
		/*
		 * Enable the interrupt for the Uart16550 device 5.
		 */
		XScuGic_Enable(IntcInstancePtr, GPS_Data_IRQ);

		/*
		 * Setup the handlers for the UART that will be called from the
		 * interrupt context when data has been sent and received, specify a
		 * pointer to the UART driver instance as the callback reference so
		 * the handlers are able to access the instance data.
		 */
		 XUartNs550_SetHandler(Uart16550Ptr5, UartNs550IntrHandler_GPS_Data , Uart16550Ptr5);

		 /*
			 * Enable the interrupt of the UART so interrupts will occur,keep the
			 * FIFOs enabled, clear RX_FIFO AND TX_FIFO.
			 */
		 XUartNs550_SetOptions(Uart16550Ptr5, Options);

/******************************************************************************/

	Xil_ExceptionInit();
	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
					(Xil_ExceptionHandler)XScuGic_DeviceInterruptHandler,
					(void *)INTC_DEVICE_ID);
	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	XScuGic_RegisterHandler(INTC_BASE_ADDR, TIMER_IRPT_INTR,
					(Xil_ExceptionHandler)timer_callback,
					(void *)&TimerInstance);
	/*
	 * Enable the interrupt for scu timer.
	 */
	XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, TIMER_IRPT_INTR);
	xil_printf("Interrupt Enable successful\n");

	//tcp_print(msg, strlen(msg));
	return 0;

}


/*****************************************************************************/
/**
*
* This function is the handler which performs processing to handle data events
* from the UartNs550.  It is called from an interrupt context such that the
* amount of processing performed should be minimized.
*
* This handler provides an example of how to handle data for the UART and
* is application specific.
*
* @param	CallBackRef contains a callback reference from the driver,
*			in this case it is the instance pointer for the UART driver.
* @param	Event contains the specific kind of event that has occurred.
* @param	EventData contains the number of bytes sent or received for sent
*		and receive events.
*
* @return	None.
*
* @note		None.
*
*******************************************************************************/
void UartNs550IntrHandler_RW1(void *CallBackRef, u32 Event, unsigned int EventData)
{
	XUartNs550 *UartNs550Ptr = (XUartNs550 *)CallBackRef;

	/*
	 * All of the data has been sent.
	 */
	if (Event == XUN_EVENT_SENT_DATA) {
		TotalSentCount = EventData;
		RW1SendHandler(CallBackRef, EventData);
	}

	/*
	 * All of the data has been received.
	 */
	if (Event == XUN_EVENT_RECV_DATA) {
		TotalReceivedCount = EventData;
		RW1RecvHandler(CallBackRef, EventData);
	}

	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 4 character times.
	 */
	if (Event == XUN_EVENT_RECV_TIMEOUT) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred.
	 */
	if (Event == XUN_EVENT_RECV_ERROR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
		Errors_1 = XUartNs550_GetLastErrors(UartNs550Ptr);
	}
}

void UartNs550IntrHandler_RW2(void *CallBackRef, u32 Event, unsigned int EventData)
{
	XUartNs550 *UartNs550Ptr = (XUartNs550 *)CallBackRef;

	/*
	 * All of the data has been sent.
	 */
	if (Event == XUN_EVENT_SENT_DATA) {
		TotalSentCount = EventData;
		RW2SendHandler(CallBackRef, EventData);
	}

	/*
	 * All of the data has been received.
	 */
	if (Event == XUN_EVENT_RECV_DATA) {
		TotalReceivedCount = EventData;
		RW2RecvHandler(CallBackRef, EventData);
	}

	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 4 character times.
	 */
	if (Event == XUN_EVENT_RECV_TIMEOUT) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred.
	 */
	if (Event == XUN_EVENT_RECV_ERROR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
		Errors_2 = XUartNs550_GetLastErrors(UartNs550Ptr);
	}
}

void UartNs550IntrHandler_RW3(void *CallBackRef, u32 Event, unsigned int EventData)
{
	XUartNs550 *UartNs550Ptr = (XUartNs550 *)CallBackRef;

	/*
	 * All of the data has been sent.
	 */
	if (Event == XUN_EVENT_SENT_DATA) {
		TotalSentCount = EventData;
		RW3SendHandler(CallBackRef, EventData);
	}

	/*
	 * All of the data has been received.
	 */
	if (Event == XUN_EVENT_RECV_DATA) {
		TotalReceivedCount = EventData;
		RW3RecvHandler(CallBackRef, EventData);
	}

	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 4 character times.
	 */
	if (Event == XUN_EVENT_RECV_TIMEOUT) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred.
	 */
	if (Event == XUN_EVENT_RECV_ERROR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
		Errors_3 = XUartNs550_GetLastErrors(UartNs550Ptr);
	}
}

void UartNs550IntrHandler_RW4(void *CallBackRef, u32 Event, unsigned int EventData)
{
	XUartNs550 *UartNs550Ptr = (XUartNs550 *)CallBackRef;

	/*
	 * All of the data has been sent.
	 */
	if (Event == XUN_EVENT_SENT_DATA) {
		TotalSentCount = EventData;
		RW4SendHandler(CallBackRef, EventData);
	}

	/*
	 * All of the data has been received.
	 */
	if (Event == XUN_EVENT_RECV_DATA) {
		TotalReceivedCount = EventData;
		RW4RecvHandler(CallBackRef, EventData);
	}

	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 4 character times.
	 */
	if (Event == XUN_EVENT_RECV_TIMEOUT) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred.
	 */
	if (Event == XUN_EVENT_RECV_ERROR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
		Errors_4 = XUartNs550_GetLastErrors(UartNs550Ptr);
	}
}

void UartNs550IntrHandler_GPS_Data(void *CallBackRef, u32 Event, unsigned int EventData)
{
	XUartNs550 *UartNs550Ptr = (XUartNs550 *)CallBackRef;

	/*
	 * All of the data has been sent.
	 */
	if (Event == XUN_EVENT_SENT_DATA) {
		TotalSentCount = EventData;
		GPS_DataSendHandler(CallBackRef, EventData);
	}

	/*
	 * All of the data has been received.
	 */
	if (Event == XUN_EVENT_RECV_DATA) {
		TotalReceivedCount = EventData;
		GPS_DataRecvHandler(CallBackRef, EventData);
	}

	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 4 character times.
	 */
	if (Event == XUN_EVENT_RECV_TIMEOUT) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred.
	 */
	if (Event == XUN_EVENT_RECV_ERROR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
		Errors_4 = XUartNs550_GetLastErrors(UartNs550Ptr);
	}
}

/***************************************************************************************************
 * Function name : RW1SendHandler(), RW2SendHandler(),
 * 				   RW3SendHandler(), RW4SendHandler()
 ****************************************************************************************************
 * I/P Arguments :  @param	CallBackRef contains a callback reference from the driver.
*							In this case it is the instance pointer for the UartLite driver.
* 					@param	EventData contains the number of bytes sent or received for sent
*							and receive events.
*
* Return value  :  None
* Description   :  This function is the handler which performs processing to send data to the
* 				   UartLite. It is called from an interrupt context such that the amount of
*				   processing performed should be minimized. It is called when the transmit
* 				   FIFO of the UartLite is empty and more data can be sent through the UartLite.
*
* 				   This handler provides an example of how to handle data for the UartLite, but
* 				   is application specific.
*
*
 ****************************************************************************************************/

/*********************************************** RW1SendHandler ************************************************/
static void RW1SendHandler(void *CallBackRef, unsigned int EventData)
{
	RW1_SentCount = EventData;
}
/*********************************************** RW2SendHandler ************************************************/
static void RW2SendHandler(void *CallBackRef, unsigned int EventData)
{
	RW2_SentCount = EventData;
}

/*********************************************** RW3SendHandler ************************************************/
static void RW3SendHandler(void *CallBackRef, unsigned int EventData)
{
	RW3_SentCount = EventData;
}

/*********************************************** RW4SendHandler ************************************************/
static void RW4SendHandler(void *CallBackRef, unsigned int EventData)
{
	RW4_SentCount = EventData;
}

/*********************************************** RW1SendHandler ************************************************/
static void GPS_DataSendHandler(void *CallBackRef, unsigned int EventData)
{
	RW1_SentCount = EventData;
}

/*************************************************GPIO_PINS_Intialization************************************************************************/
void GPIO_PINS_Intialization(void)
{
	//char msg[] = "End of GPIO Pins Init\n";
	XGpio_Initialize(&GPS_GPIO, GPS_GPIO_ID); 		// GPS_GPIO Pin Configuration
	XGpio_SetDataDirection(&GPS_GPIO, 1, 0x0); 		// Make GPS_GPIO Pin Output

	XGpio_Initialize(&DAC_GPIO, DAC_GPIO_ID); 		// DAC_GPIO Pin Configuration
	XGpio_SetDataDirection(&DAC_GPIO, 1, 0x0); 		// Make DAC_GPIO Pin Output

	XGpio_Initialize(&IMU_DR, IMU_DR_GPIO_ID); 		// IMU_Data_Ready Gpio Pin Configuration
	XGpio_SetDataDirection(&IMU_DR, 1, 0x0); 		// Make IMU_Data_Ready Pin Output

	/********************************************************************************/
	/********************************************************************************/

		XGpio_Initialize(&RW1_DEBUG, RW1_DEBUG_ID); 	// RW1_DEBUG Pin Configuration
		XGpio_SetDataDirection(&RW1_DEBUG, 1, 0x0); 	// Make RW1_DEBUG Pin Output

		XGpio_Initialize(&RW2_DEBUG, RW2_DEBUG_ID); 	// RW1_DEBUG Pin Configuration
		XGpio_SetDataDirection(&RW2_DEBUG, 1, 0x0);		// Make RW1_DEBUG Pin Output

		XGpio_Initialize(&RW3_DEBUG, RW3_DEBUG_ID); 	// RW1_DEBUG Pin Configuration
		XGpio_SetDataDirection(&RW3_DEBUG, 1, 0x0); 	// Make RW1_DEBUG Pin Output

		XGpio_Initialize(&RW4_DEBUG, RW4_DEBUG_ID); 	// RW1_DEBUG Pin Configuration
		XGpio_SetDataDirection(&RW4_DEBUG, 1, 0x0); 	// Make RW1_DEBUG Pin Output

		/********************************************************************************/
	//tcp_print(msg, strlen(msg));

}

/************************************************Analog_SPI_intialization********************************************************************************/
void Analog_SPI_intialization(void)
{
	//char msg[] = "End of Analog SPI Init\n";
	XSpiPs_Config *SpiConfig;
	u8 ref_in[] = {0x0c,0x14,0x00};

	//Initialize SPI
	SpiConfig = XSpiPs_LookupConfig(XPAR_XSPIPS_0_DEVICE_ID);
	XSpiPs_CfgInitialize(&DAC_SPI_Instance, SpiConfig,SpiConfig->BaseAddress);
	XSpiPs_SelfTest(&DAC_SPI_Instance);

	//Set SPI mode
	XSpiPs_SetOptions(&DAC_SPI_Instance, XSPIPS_MANUAL_START_OPTION |XSPIPS_CLK_PHASE_1_OPTION |XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION);

	//Set Clk to 600khz
	XSpiPs_SetClkPrescaler(&DAC_SPI_Instance, XSPIPS_CLK_PRESCALE_256);

	/* To set the DAC(AD5383) internal voltage 2.5V
	 * Start the sequence for DAC (SPI sequence)
	 *
	 */
	// Start sequence

	//Initiate reset of DAC
	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b111);
	usleep(1000);
	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b110);
	usleep(1000);

	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b110); // Chip select Pin
	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b010); // LDAC Pin

	/* Reference voltage to be set 2.5 voltage
	 *
	 */
	XSpiPs_PolledTransfer(&DAC_SPI_Instance,&ref_in[0],NULL,sizeof(ref_in[0]));
	XSpiPs_PolledTransfer(&DAC_SPI_Instance,&ref_in[1],NULL,sizeof(ref_in[1]));
	XSpiPs_PolledTransfer(&DAC_SPI_Instance,&ref_in[2],NULL,sizeof(ref_in[2]));

	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b100);
	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b111);

	//tcp_print(msg, strlen(msg));
}

/**************************************************IMU_SPI_Intialization***********************************************************/

void IMU_SPI_Intialization(XSpi *SpiInstancePtr, u16 SpiDeviceId)
{
	//char msg[] = "End of IMU SPI Init\n";

	XSpi_Config *ConfigPtr;
	ConfigPtr = XSpi_LookupConfig(SpiDeviceId); //  Device ID Configartion
	XSpi_CfgInitialize(SpiInstancePtr, ConfigPtr,ConfigPtr->BaseAddress);
	XSpi_SetOptions(SpiInstancePtr, XSP_CLK_PHASE_1_OPTION | XSP_CLK_ACTIVE_LOW_OPTION); // Clock Phase =1; Clock Polrity=1;

	//tcp_print(msg, strlen(msg));
}

/********************************************************IMU_Simulation***************************************************************/
void IMU_Simulation(void)
{
	//char msg[] = "End of IMU Simulation\n";
	u32 ControlReg=0;

	//Data Ready Low
	XGpio_DiscreteWrite(&IMU_DR,1,0); // Make Data ready Pin LOW

    //Empty TX & RX FIFO
	ControlReg = XSpi_GetControlReg(&IMU_SPI_Instance);
	XSpi_SetControlReg(&IMU_SPI_Instance, ControlReg |XSP_CR_RXFIFO_RESET_MASK |XSP_CR_TXFIFO_RESET_MASK);
	ControlReg = XSpi_GetControlReg(&IMU_SPI_Instance);

	//Load FIFO
	Spi_Load_FIFO(&IMU_SPI_Instance, IMU_Data, BUFFER_COUNT); // Load the value to FIFO

	//SPI Enable
	ControlReg = XSpi_GetControlReg(&IMU_SPI_Instance);
	XSpi_SetControlReg(&IMU_SPI_Instance, ControlReg |XSP_CR_ENABLE_MASK);
	ControlReg = XSpi_GetControlReg(&IMU_SPI_Instance);
	//Data Ready High
	XGpio_DiscreteWrite(&IMU_DR,1,1);

	//tcp_print(msg, strlen(msg));
}

/**************************************Spi_Load_FIFO*************************************************************************/
void Spi_Load_FIFO(XSpi *InstancePtr, u16 SendBufPtr[],  int ByteCount)
{
	//char msg[] = "End of Spi_Load_FIFO\n";
	u32 StatusReg;
	u32 Data = 0;


	StatusReg = XSpi_GetStatusReg(InstancePtr); // Status Register Reading

	while (!(StatusReg & XSP_SR_TX_FULL_MASK)) // Monitoring the Status of TX_ Full
		{

			Data = (u32)*SendBufPtr++ ;									 	// Load the Buffer pointer to data variable
			XSpi_WriteReg(InstancePtr->BaseAddr, XSP_DTR_OFFSET,Data); 		// Send to the Register
			StatusReg = XSpi_GetStatusReg(InstancePtr);						// Read the Status Register
			ByteCount--;													// Decrement the Byte count
			if (ByteCount<=0) break;

		}
	//tcp_print(msg, strlen(msg));
}

/************************** imu_data_processing function *****************************/
void imu_data_processing(u8 Channel_No, u16 data)
{
	//char msg[] = "End of imu_data_processing\n";
	/* IMU data processing functions to store the array
	 * Channel Number becomes the index of array and sensor data is stored on the corresponding index.
	 */
	IMU_Data[Channel_No-1]= data;
//	xil_printf("IMU channel = %x\n IMU Data = %x", Channel_No, data);
	//tcp_print(msg, strlen(msg));
}

/********************************************************Analog_sensor****************************************************************************************************/
void Analog_sensor(u8 channel,u8 data1,u8 data2)
{
	//char msg[] = "End of Analog_sensor\n";
/***************************  data to be processed for SPI transfer  ****************************/
	u8 x,y,z;
	u8 sun[3];
	sun[0] = channel;
	x = data1;
	y = data2;
	z = (0xc0 & y);
	y = (y << 2);
	z = (z >> 6);
	x = (x << 2);
	x |= 0xc0;
	x |= z;
	sun[1] = x;
	sun[2] = y;


	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b111);
	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b011);

	XSpiPs_PolledTransfer(&DAC_SPI_Instance,&sun[0],NULL,sizeof(sun[0]));
	XSpiPs_PolledTransfer(&DAC_SPI_Instance,&sun[1],NULL,sizeof(sun[1]));
	XSpiPs_PolledTransfer(&DAC_SPI_Instance,&sun[2],NULL,sizeof(sun[2]));

	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b101);
	XGpio_DiscreteWrite(&DAC_GPIO, 1, 0b111);

	//tcp_print(msg, strlen(msg));
}
/*********************************************************************************************************/
u32 wait(u32 useconds)
{
	static XTime tEnd;
	XTime tCur;
	static int timer_start = 0;

	if(timer_start == 0)
	{
		XTime_GetTime(&tCur);
		tEnd = tCur + (((XTime) useconds) * COUNTS_PER_USECOND);
		timer_start = 1;
	}
	else
	{
		XTime_GetTime(&tCur);
		if(tCur < tEnd)
			return 0;
		else
		{
			timer_start = 0;
			return 1;
		}
	}
	return 0;
}
/********************************************************************************************************/

/**********************************************************************************************************
 * Function name : power_init()
 **********************************************************************************************************
 * I/P Arguments : void
 *
 * Return value  :  None
 * Description   :  This function will initializes all the peripherals.
 *
 ***************************************************************************************************************/
void power_init(void)
{
	GPIO_PINS_Intialization();
	Analog_SPI_intialization();
	IMU_SPI_Intialization(&IMU_SPI_Instance,IMU_ID);
}


/**********************************************************************************************************
 * Function name : RW1RecvHandler(), RW2RecvHandler(),
 * 				   RW3RecvHandler(), RW4RecvHandler()
 **********************************************************************************************************
 * I/P Arguments :  @param	CallBackRef contains a callback reference from the driver.
 *							In this case it is the instance pointer for the UartLite driver.
 * 					@param	EventData contains the number of bytes sent or received for sent
 *							and receive events.
 *
 * Return value  :  None
 * Description   :  This function is the handler which performs processing to send data to the
 * 				   Uart16550. It is called from an interrupt context such that the amount of
 *				   processing performed should be minimized. It is called when the transmit
 * 				   FIFO of the Uart16550 is empty and more data can be sent through the UartLite.
 *
 * 				   This handler provides an example of how to handle data for the UartLite, but
 * 				   is application specific.
 *
 ***************************************************************************************************************/
/*********************************************** RW1RecvHandler ************************************************/
static void RW1RecvHandler(void *CallBackRef, unsigned int EventData)
{

	int i, ch, RecvCount, index;
	RecvCount = EventData;

	Rw1RecvIntrpt = RW1.Stats.ReceiveInterrupts;
	// repeat this loop for all chars received, i.e., for all ReceivedCount

	i = 0;
	while (i < RecvCount) {

		ch = RW1_RecieveBuffer[i++];  // get the received char from the buffer

		if(RW1_Start_byte_flag == 1)
		{

				// Stop Byte Check for RW1

				if (ch == 0xc0)
				{
					// Ignore one of the two successive start byte characters
					if (RW1_ReceivedCount > 1)
					{
						XGpio_DiscreteWrite(&RW1_DEBUG,1,0);
						RW1_Start_byte_flag = 0;
						RW1_Buffer[RW1_ReceivedCount++] = ch;
						RW1_Frame_complete_flag = 1;
						RW1_count_OBC++;
					}

				}
				else
				{
					if ((index = RW1_ReceivedCount) < TEST_BUFFER_SIZE) {
					    RW1_Buffer[index] = ch;
					    RW1_ReceivedCount++;
					}
					else
						RW1_Start_byte_flag = 0;
				}

		}
		// Start Byte Check for RW1

		else if (ch == 0xc0)
				{
					XGpio_DiscreteWrite(&RW1_DEBUG,1,1);
					RW1_Start_byte_flag = 1;
					RW1_ReceivedCount   = 0;
					RW1_Buffer[RW1_ReceivedCount++] = ch;

					// Note the cpu time when first character is received
					XTime_GetTime(&t_start_RW1);

					RW1_Frame_complete_flag = 0;


				}
	}
	if(RW1_Frame_complete_flag == 0)
	{
		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW1, RW1_RecieveBuffer, 1);
	}
}

/*********************************************** RW2RecvHandler ************************************************/
static void RW2RecvHandler(void *CallBackRef, unsigned int EventData)
{
	int i, ch, RecvCount, index;

	RecvCount = EventData;

	Rw2RecvIntrpt = RW2.Stats.ReceiveInterrupts;
	// repeat this loop for all chars received, i.e., for all ReceivedCount

	i = 0;
	while (i < RecvCount) {

		ch = RW2_RecieveBuffer[i++];  // get the received char from the buffer

		if(RW2_Start_byte_flag == 1)
		{

				// Stop Byte Check for RW2

				if(ch == 0xc0)
				{
					if (RW2_ReceivedCount > 1)
					{
						XGpio_DiscreteWrite(&RW2_DEBUG,1,0);
						RW2_Start_byte_flag = 0;
						RW2_Buffer[RW2_ReceivedCount++] = ch;
						RW2_Frame_complete_flag = 1;
						RW2_count_OBC++;
					}

				}
				else
				{
					if ((index = RW2_ReceivedCount) < TEST_BUFFER_SIZE) {
						RW2_Buffer[index] = ch;
						RW2_ReceivedCount++;

					}
					else
						RW2_Start_byte_flag = 0;
				}

		}
		// Start Byte Check for RW2

		else if (ch == 0xc0)
				{
					XGpio_DiscreteWrite(&RW2_DEBUG,1,1);
					RW2_Start_byte_flag = 1;
					RW2_ReceivedCount   = 0;
					RW2_Buffer[RW2_ReceivedCount++] = ch;

					// Note the cpu time when first character is received
					XTime_GetTime(&t_start_RW2);

					RW2_Frame_complete_flag = 0;

				}

	}
	if(RW2_Frame_complete_flag == 0)
	{
		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW2, RW2_RecieveBuffer, 1);
	}

}

/*********************************************** RW3RecvHandler ************************************************/
static void RW3RecvHandler(void *CallBackRef, unsigned int EventData)
{
	int i, ch, RecvCount, index;

	RecvCount = EventData;
	Rw3RecvIntrpt = RW3.Stats.ReceiveInterrupts;
	// repeat this loop for all chars received, i.e., for all ReceivedCount

	i = 0;
	while (i < RecvCount) {

		ch = RW3_RecieveBuffer[i++];  // get the received char from the buffer

		if(RW3_Start_byte_flag == 1)
		{

				// Stop Byte Check for RW3

				if(ch == 0xc0)
				{
					if (RW3_ReceivedCount > 1)
					{
						XGpio_DiscreteWrite(&RW3_DEBUG,1,0);
						RW3_Start_byte_flag = 0;
						RW3_Buffer[RW3_ReceivedCount++] = ch;
						RW3_Frame_complete_flag = 1;
						RW3_count_OBC++;
					}

				}
				else
				{
					if ((index = RW3_ReceivedCount) < TEST_BUFFER_SIZE) {
						RW3_Buffer[index] = ch;
						RW3_ReceivedCount++;

						}
					else
						RW3_Start_byte_flag = 0;
				}

		}
		// Start Byte Check for RW3

		else if (ch == 0xc0)
				{
					XGpio_DiscreteWrite(&RW3_DEBUG,1,1);
					RW3_Start_byte_flag = 1;
					RW3_ReceivedCount   = 0;
					RW3_Buffer[RW3_ReceivedCount++] = ch;

					// Note the cpu time when first character is received
					XTime_GetTime(&t_start_RW3);

					RW3_Frame_complete_flag = 0;

				}

	}

	if(RW3_Frame_complete_flag == 0)
	{
		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW3, RW3_RecieveBuffer, 1);
	}

}

/*********************************************** RW4RecvHandler ************************************************/
static void RW4RecvHandler(void *CallBackRef, unsigned int EventData)
{
	int i, ch, RecvCount, index;

	RecvCount = EventData;

	Rw4RecvIntrpt = RW4.Stats.ReceiveInterrupts;

	// repeat this loop for all chars received, i.e., for all ReceivedCount

	i = 0;
	while (i < RecvCount) {

		ch = RW4_RecieveBuffer[i++];  // get the received char from the buffer

		if(RW4_Start_byte_flag == 1)
		{

				// Stop Byte Check for RW4

				if(ch == 0xc0)
				{
					if (RW4_ReceivedCount > 1)
					{
						XGpio_DiscreteWrite(&RW4_DEBUG,1,0);
						RW4_Start_byte_flag = 0;
						RW4_Buffer[RW4_ReceivedCount++] = ch;
						RW4_Frame_complete_flag = 1;
						RW4_count_OBC++;
					}

				}
				else
				{
					if ((index = RW4_ReceivedCount) < TEST_BUFFER_SIZE) {
						RW4_Buffer[index] = ch;
						RW4_ReceivedCount++;

					}
					else
						RW4_Start_byte_flag = 0;

				}
		}
		// Start Byte Check for RW4

		else if (ch == 0xc0)
				{
					XGpio_DiscreteWrite(&RW4_DEBUG,1,1);
					RW4_Start_byte_flag = 1;
					RW4_ReceivedCount   = 0;
					RW4_Buffer[RW4_ReceivedCount++] = ch;

					// Note the cpu time when first character is received
					XTime_GetTime(&t_start_RW4);

					RW4_Frame_complete_flag = 0;

				}

	}
	if(RW4_Frame_complete_flag ==0)
	{
		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW4, RW4_RecieveBuffer, 1);
	}

}

static void GPS_DataRecvHandler(void *CallBackRef, unsigned int EventData)
{

}

/***************************************************************************************************************
                           void RW_Simulation(void)
 ***************************************************************************************************************
 * I/P Arguments : none
 * Return value  : XST_SUCCESS if successful, otherwise XST_FAILURE.
 * Description   : Reaction wheel simulation.
 ***************************************************************************************************************/
void RW_Simulation(void)
{
	//RW2_Simulation();
#ifdef RW_DEBUG
	RW1_Simulation();
	RW2_Simulation();
	RW3_Simulation();
	RW4_Simulation();
#endif

	RW_UI_SND();
	transfer_data();

}

/*********************************************** RW1_Simulation ************************************************/
void RW1_Simulation(void){
unsigned int i;

	// Frame complete flag check.

	 if (RW1_Frame_complete_flag == 1)
	{
		xil_printf("\nReceived count = %d\n", RW1_ReceivedCount);
		for(i = 0;i < RW1_ReceivedCount;i++)
		{
			xil_printf("RW1_Buffer[%d] = 0x%x\n",i, RW1_Buffer[i]);
		}

	}

}

/*********************************************** RW2_Simulation ************************************************/
void RW2_Simulation(void)
{
	unsigned int i;

	// Frame complete flag check.

	 if (RW2_Frame_complete_flag == 1)
	{
		xil_printf("\nReceived count = %d\n", RW2_ReceivedCount);
		for(i = 0;i < RW2_ReceivedCount;i++)
		{
			xil_printf("RW2_Buffer[%d] = 0x%x\n",i, RW2_Buffer[i]);
		}

	}

}

/*********************************************** RW3_Simulation ************************************************/
void RW3_Simulation(void)
{
	unsigned int i;

	// Frame complete flag check.

	 if (RW3_Frame_complete_flag == 1)
	{
		xil_printf("\nReceived count = %d\n", RW3_ReceivedCount);
		for(i = 0;i <= RW3_ReceivedCount;i++)
		{
			xil_printf("RW3_Buffer[%d] = 0x%x\r\n",i, RW3_Buffer[i]);
		}

	}

}

/*********************************************** RW4_Simulation ************************************************/
void RW4_Simulation(void)
{
	unsigned int i;

	// Frame complete flag check.

	 if (RW4_Frame_complete_flag == 1)
	{
		xil_printf("\nReceived count = %d\n", RW4_ReceivedCount);
		for(i = 0;i < RW4_ReceivedCount;i++)
		{
			xil_printf("RW4_Buffer[%d] = 0x%x\n",i, RW4_Buffer[i]);
		}

	}

}

/***************************************************************************************************************
                           	   crc_compute()
 ***************************************************************************************************************
 * I/P Arguments : RW_Buffer_Addr, Byte_Count
 * Return value  : unsigned short
 * Description   : This function computes the crc for RW data received from the OBC.
 *
 ***************************************************************************************************************/
unsigned short crc_compute(unsigned char* RW_Buffer_Addr,unsigned long int Byte_Count)
{
	//char msg[] = "End of crc_compute\n";
	unsigned short RW_crc;
	unsigned char  RW_crc_check_Byte;
	unsigned int   RW_crc_check_count;

	RW_crc = 0xFFFF;			// Initialize shift register value
	while(Byte_Count-- > 0)
	{
		RW_crc_check_Byte = *RW_Buffer_Addr++;
		for(RW_crc_check_count = 0; RW_crc_check_count<= 7; RW_crc_check_count++)
		{
			RW_crc = (RW_crc >> 1) ^ (((RW_crc_check_Byte ^ RW_crc) & 0x01) ? RW_POLY :0);
			RW_crc_check_Byte >>= 1;
		}
	}
	//tcp_print(msg,strlen(msg));
	return RW_crc;

}

/***************************************************************************************************************
                           	   reverse_slip_frame()
 ***************************************************************************************************************
 * I/P Arguments : slip_frame_Buffer_addr, RW_Buffer_Addr, Byte_Count
 * Return value  : unsigned int
 * Description   : This function will replace 'db' and 'dc' bytes of RW data frame by 'c0'.
 *
 ***************************************************************************************************************/
unsigned int reverse_slip_frame(unsigned char* slip_frame_Buffer_addr, unsigned char* RW_Buffer_Addr, unsigned long int Byte_Count)
{
	//char msg[] = "End of reverse_slip_frame\n";
	unsigned int reverse_slip_count = 0;
	while(Byte_Count-- > 0)
	{
		if(*RW_Buffer_Addr == 0xDB)
		{
			reverse_slip_count++;
			RW_Buffer_Addr++;

			if(*RW_Buffer_Addr == 0xDC)
			{
				RW_Buffer_Addr++;
				*slip_frame_Buffer_addr = 0xC0;
				slip_frame_Buffer_addr++;
			}
			else if(*RW_Buffer_Addr == 0xDD)
			{
				RW_Buffer_Addr++;
				*slip_frame_Buffer_addr = 0xDB;
				slip_frame_Buffer_addr++;

			}
			else{}
		}
		else
		{
			 *slip_frame_Buffer_addr = *RW_Buffer_Addr;
			 slip_frame_Buffer_addr++;
			 RW_Buffer_Addr++;
		}
	}
	//tcp_print(msg,strlen(msg));
	if(reverse_slip_count)
	{
		return reverse_slip_count;
	}
	return 0;
}

/***************************************************************************************************************
                           	   reverse_slip_frame()
 ***************************************************************************************************************
 * I/P Arguments : slip_frame_Buffer_addr, RW_Buffer_Addr, Byte_Count
 * Return value  : void
 * Description   : This function will replace 'c0' byte by 'db' and 'dc' of RW data frame except start and stop byte .
 *
 ***************************************************************************************************************/
unsigned char slip_frame(unsigned char* slip_frame_Buffer_addr, unsigned char* RW_Buffer_Addr, unsigned int byte)
{
	//char msg[] = "End of Analog_sensor\n";
    unsigned int slip_count = 0;
    byte -= 2;
    *slip_frame_Buffer_addr++ = *RW_Buffer_Addr++;
    while(byte-- > 0)
    {
        if(*RW_Buffer_Addr != 0xc0 && *RW_Buffer_Addr != 0xDB)
        {
            *slip_frame_Buffer_addr++ = *RW_Buffer_Addr++;
        }
        else
        {

            if(*RW_Buffer_Addr == 0xc0)
            {
                *slip_frame_Buffer_addr++ = 0xDB;
                *slip_frame_Buffer_addr++ = 0xDC;
                RW_Buffer_Addr++;
                slip_count++;
            }
            else if(*RW_Buffer_Addr == 0xDB)
            {
                *slip_frame_Buffer_addr++ = 0xDB;
                *slip_frame_Buffer_addr++ = 0xDD;
                RW_Buffer_Addr++;
                slip_count++;
            }
            else{}

        }
    }
     *slip_frame_Buffer_addr++ = *RW_Buffer_Addr++;
     //tcp_print(msg,strlen(msg));
    if(slip_count)
    {
        return slip_count;
    }
    return 0;

}


/***************************************************************************************************************
                           	   void RW_UI_SND()
 ***************************************************************************************************************
 * I/P Arguments : none
 * Return value  : none
 * Description   : Appends character 'p', 'q', 'r', 's' and length field
 * 				   with four reaction wheel packets respectively.
 *
 ***************************************************************************************************************/

void RW_UI_SND()
{
	//char msg[] = "End of RW_UI_SND \n";
	unsigned int i;
	unsigned char crc_1, crc_2, crc_send_1, crc_send_2;
	unsigned short computed_crc, crc, crc_send;
	int RW1_rx_count, RW2_rx_count, RW3_rx_count, RW4_rx_count;
	unsigned char slip_frame_Buffer[TEST_BUFFER_SIZE];
	unsigned int RW1_RSF_count, RW2_RSF_count, RW3_RSF_count, RW4_RSF_count;
	unsigned int RW1_SF_count, RW2_SF_count, RW3_SF_count, RW4_SF_count;

	u8 NACK_Buffer[] = {0xc0,0x11,0x40,0x88,0x00,0x05,0x00,0x00,0xc8,0x42,0x59,0x82,0xc0};
	u8 OBC_NACK_Buf[TEST_BUFFER_SIZE];
	unsigned int NACK_Buffer_Count;

/*************** UI RW1 send buffer operation********/
	if(RW1_Frame_complete_flag == 1)
	{
		RW1_rx_count = RW1_ReceivedCount;

		// Slip frame check
		RW1_RSF_count = reverse_slip_frame(&slip_frame_Buffer[0], &RW1_Buffer[0], RW1_rx_count);


		RW1_rx_count -= RW1_RSF_count;

		/************ Extract CRC ************/

		crc_1 = slip_frame_Buffer[RW1_rx_count-3];
		crc_2 = slip_frame_Buffer[RW1_rx_count-2];
		crc   = (crc_2<<8) | (crc_1);			// crc byte of rx data

		/************************************/
		// compute the crc for rx data frame
		computed_crc = crc_compute(&slip_frame_Buffer[1], RW1_rx_count - 4);

		// if computed and actual crc matches send data to UI. Otherwise send NACK to OBC
		if(computed_crc == crc)
		{
			RW1_SendBuffer[0] = 'p';
			RW1_SendBuffer[1] = RW1_ReceivedCount;
			for(i = 0; i < RW1_ReceivedCount ; i++)
			{
				RW1_SendBuffer[i+2] = RW1_Buffer[i];
			}
			RW1_Frame_complete_flag = 0;
			UI_RW1_Buffer_Count = RW1_ReceivedCount + 2;    // Enable Count for UI to transmit
		}
		else
		{
			// Reset the ACK bit in MCF for NACK to OBC
			slip_frame_Buffer[1]  = 0x11;	// Destination address
			slip_frame_Buffer[2]  = 0x40;	// Source address
			slip_frame_Buffer[3] &= 0xDF;	// Resetting ACK bit in MCF

			// Compute crc for sending NACK data frame
			crc_send   = crc_compute(&slip_frame_Buffer[1], RW1_rx_count - 4);
			crc_send_1 = crc_send & 0x00ff;
			crc_send_2 = (crc_send & 0xff00) >> 8;


			// Replace old crc by newly computed crc
            // Note: LSB byte first
			slip_frame_Buffer[RW1_rx_count - 3] = crc_send_1;
			slip_frame_Buffer[RW1_rx_count - 2] = crc_send_2;

			// replace 'c0' and 'db' by slip frame before sending NACK to OBC.
			RW1_SF_count  = slip_frame(&OBC_NACK_Buf[0],& slip_frame_Buffer[0], RW1_rx_count);

			RW1_SF_count += RW1_rx_count;

			if(RW1_SF_count > 1){
				// Sending NACK to the OBC.
				XUartNs550_Send (&RW1, OBC_NACK_Buf, RW1_SF_count);
			}

			// Clear the flag to receive next Data frame
			RW1_Frame_complete_flag = 0;

		}

		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW1, RW1_RecieveBuffer, 1);

	}
	else if(RW1_timeout_flag == 1)
	{
		NACK_Buffer_Count = 13;
		// Send a NACK to OBC on occurance of timeout
		XUartNs550_Send (&RW1, NACK_Buffer, NACK_Buffer_Count);

		RW1_Start_byte_flag = 0;
		RW1_timeout_flag = 0;

		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW1, RW1_RecieveBuffer, 1);

	}

/*************** UI RW2 send buffer operation********/
	if(RW2_Frame_complete_flag == 1)
	{
		RW2_rx_count = RW2_ReceivedCount;
		// Slip frame check
		RW2_RSF_count = reverse_slip_frame(&slip_frame_Buffer[0], &RW2_Buffer[0], RW2_rx_count);

		RW2_rx_count -= RW2_RSF_count;

		/************ Extract CRC ************/

		crc_1 = slip_frame_Buffer[RW2_rx_count-3];
		crc_2 = slip_frame_Buffer[RW2_rx_count-2];
		crc   = (crc_2<<8) | (crc_1);			// crc byte of rx data

		/************************************/

		// compute the crc for rx data frame
		computed_crc = crc_compute(&slip_frame_Buffer[1], RW2_rx_count - 4);

		// if computed and actual crc matches send data to UI. Otherwise send NACK to OBC
		if(computed_crc == crc)
		{
			RW2_SendBuffer[0] = 'q';
			RW2_SendBuffer[1] = RW2_ReceivedCount;
			for(i = 0; i < RW2_ReceivedCount; i++)
			{
				RW2_SendBuffer[i+2] = RW2_Buffer[i];
			}
			RW2_Frame_complete_flag = 0;
			UI_RW2_Buffer_Count = RW2_ReceivedCount + 2;    // Enable Count for UI to transmit
		}
		else
		{
			// Disable the ACK bit in MCF for NACK to OBC
			slip_frame_Buffer[1]  = 0x11;	// Destination address
			slip_frame_Buffer[2]  = 0x40;	// Source address
			slip_frame_Buffer[3] &= 0xDF;

			// Compute crc for sending NACK data frame
			crc_send   = crc_compute(&slip_frame_Buffer[1], RW2_rx_count - 4);
			crc_send_1 = crc_send & 0x00ff;
			crc_send_2 = (crc_send & 0xff00) >> 8;


			// Replace old crc by newly computed crc
			slip_frame_Buffer[RW2_rx_count - 3] = crc_send_1;
			slip_frame_Buffer[RW2_rx_count - 2] = crc_send_2;

			// replace 'c0' and 'db' by slip frame before sending NACK to OBC.
			RW2_SF_count  = slip_frame(&OBC_NACK_Buf[0],& slip_frame_Buffer[0], RW2_rx_count);

			RW2_SF_count += RW2_rx_count;

			if(RW2_SF_count > 1)
			{
				// Sending NACK to the OBC.
				XUartNs550_Send (&RW2, OBC_NACK_Buf, RW2_SF_count);
			}
			// Clear the flag to receive next Data frame
			RW2_Frame_complete_flag = 0;

		}
		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW2, RW2_RecieveBuffer, 1);
	}
	else if(RW2_timeout_flag == 1)
		{
			NACK_Buffer_Count = 13;
			// Send a NACK to OBC on occurance of timeout
			XUartNs550_Send (&RW2, NACK_Buffer, NACK_Buffer_Count);

			RW2_Start_byte_flag = 0;
			RW2_timeout_flag = 0;

			// set up the buffer for next char in interrupt mode

			XUartNs550_Recv(&RW2, RW2_RecieveBuffer, 1);
		}

/*************** UI RW3 send buffer operation********/
	if(RW3_Frame_complete_flag == 1)
	{
		RW3_rx_count = RW3_ReceivedCount;
		// Slip frame check
		RW3_RSF_count = reverse_slip_frame(&slip_frame_Buffer[0], &RW3_Buffer[0], RW3_rx_count);

		RW3_rx_count -= RW3_RSF_count;

		/************ Extract CRC ************/

		crc_1 = slip_frame_Buffer[RW3_rx_count-3];
		crc_2 = slip_frame_Buffer[RW3_rx_count-2];
		crc   = (crc_2<<8) | (crc_1);			// crc byte of rx data

		/*************************************/

		// compute the crc for rx data frame
		computed_crc = crc_compute(&slip_frame_Buffer[1], RW3_rx_count - 4);

		// if computed and actual crc matches send data to UI. Otherwise send NACK to OBC
		if(computed_crc == crc)
		{
			RW3_SendBuffer[0] = 'r';
			RW3_SendBuffer[1] = RW3_ReceivedCount;
			for(i = 0; i < RW3_ReceivedCount; i++)
			{
				RW3_SendBuffer[i+2] = RW3_Buffer[i];
			}
			RW3_Frame_complete_flag = 0;
			UI_RW3_Buffer_Count = RW3_ReceivedCount + 2;    // Enable Count for UI to transmit
		}
		else
		{
			// Disable the ACK bit in MCF for NACK to OBC
			slip_frame_Buffer[1]  = 0x11;	// Destination address
			slip_frame_Buffer[2]  = 0x40;	// Source address
			slip_frame_Buffer[3] &= 0xDF;

			// Compute crc for sending NACK data frame
			crc_send   = crc_compute(&RW3_Buffer[1], RW3_rx_count - 4);
			crc_send_1 = crc_send & 0x00ff;
			crc_send_2 = (crc_send & 0xff00) >> 8;


			// Replace old crc by newly computed crc
			slip_frame_Buffer[RW3_rx_count - 3] = crc_send_1;
			slip_frame_Buffer[RW3_rx_count - 2] = crc_send_2;

			// replace 'c0' and 'db' by slip frame before sending NACK to OBC.
			RW3_SF_count  = slip_frame(&OBC_NACK_Buf[0],& slip_frame_Buffer[0], RW3_rx_count);

			RW3_SF_count += RW3_rx_count;

			if(RW3_SF_count > 1)
			{
				// Sending NACK to the OBC.
				XUartNs550_Send (&RW3, OBC_NACK_Buf, RW3_SF_count);
			}
			// Clear the flag to receive next Data frame
			RW3_Frame_complete_flag = 0;
		}
		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW3, RW3_RecieveBuffer, 1);
	}
	else if(RW3_timeout_flag == 1)
		{
			NACK_Buffer_Count = 13;
			// Send a NACK to OBC on occurance of timeout
			XUartNs550_Send (&RW3, NACK_Buffer, NACK_Buffer_Count);

			RW3_Start_byte_flag = 0;
			RW3_timeout_flag = 0;

			// set up the buffer for next char in interrupt mode

			XUartNs550_Recv(&RW3, RW3_RecieveBuffer, 1);
		}

/*************** UI RW4 send buffer operation********/
	if(RW4_Frame_complete_flag == 1)
	{
		RW4_rx_count = RW4_ReceivedCount;
		// Slip frame check
		RW4_RSF_count = reverse_slip_frame(&slip_frame_Buffer[0], &RW4_Buffer[0], RW4_rx_count);

		RW4_rx_count -= RW4_RSF_count;

		/************ Extract CRC ************/

		crc_1 = slip_frame_Buffer[RW4_rx_count-3];
		crc_2 = slip_frame_Buffer[RW4_rx_count-2];
		crc   = (crc_2<<8) | (crc_1);			// crc byte of rx data

		/*************************************/

		// compute the crc for rx data frame
		computed_crc = crc_compute(&slip_frame_Buffer[1], RW4_rx_count - 4);

		// if computed and actual crc matches send data to UI. Otherwise send NACK to OBC
		if(computed_crc == crc)
		{
		RW4_SendBuffer[0] = 's';
		RW4_SendBuffer[1] = RW4_ReceivedCount;
		for(i = 0; i < RW4_ReceivedCount; i++)
		{
			RW4_SendBuffer[i+2] = RW4_Buffer[i];
		}
		RW4_Frame_complete_flag = 0;
		UI_RW4_Buffer_Count = RW4_ReceivedCount + 2;    // Enable Count for UI to transmit
		}
		else
		{
			// Disable the ACK bit in MCF for NACK to OBC
			slip_frame_Buffer[1]  = 0x11;	// Destination address
			slip_frame_Buffer[2]  = 0x40;	// Source address
			slip_frame_Buffer[3] &= 0xDF;

			// Compute crc for sending NACK data frame
			crc_send   = crc_compute(&slip_frame_Buffer[1], RW4_rx_count - 4);
			crc_send_1 = crc_send & 0x00ff;
			crc_send_2 = (crc_send & 0xff00) >> 8;


			slip_frame_Buffer[RW4_rx_count - 3] = crc_send_1;
			slip_frame_Buffer[RW4_rx_count - 2] = crc_send_2;


			// replace 'c0' and 'db' by slip frame before sending NACK to OBC.

			RW4_SF_count  = slip_frame(&OBC_NACK_Buf[0],& slip_frame_Buffer[0], RW4_rx_count);

			RW4_SF_count += RW4_rx_count;

			if(RW4_SF_count > 1)
			{
				// Sending NACK to the OBC.
				XUartNs550_Send (&RW4, OBC_NACK_Buf, RW4_SF_count);
			}
			// Clear the flag to receive next Data frame
			RW4_Frame_complete_flag = 0;

		}
		// set up the buffer for next char in interrupt mode

		XUartNs550_Recv(&RW4, RW4_RecieveBuffer, 1);
	}
	else if(RW4_timeout_flag == 1)
		{
			NACK_Buffer_Count = 13;
			// Send a NACK to OBC on occurance of timeout
			XUartNs550_Send (&RW4, NACK_Buffer, NACK_Buffer_Count);

			RW4_Start_byte_flag = 0;
			RW4_timeout_flag = 0;

			// set up the buffer for next char in interrupt mode

			XUartNs550_Recv(&RW4, RW4_RecieveBuffer, 1);

		}
	//tcp_print(msg,strlen(msg));
}
