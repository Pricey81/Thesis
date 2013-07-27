///////////////////////////////////////////
// THESIS- 2013
///////////////////////////////////////////
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
#include "math.h"
#include "fatfs/ff.h"
#include "fatfs/diskio.h"
#include "driverlib/pin_map.h"
#include <stdint.h>
#include "global.h"
#include "stdbool.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "arm_math.h"
// mass storage start
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "third_party/fatfs/src/diskio.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include <stdint.h>
#include <stdlib.h>
#include "driverlib/usb.h"
#include "driverlib/udma.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdmsc.h"
#include "usb_msc_structs.h"

#define LOWPASS 1
#define HIGHPASS 2
#define BANDPASS 3
#define NOTCH 4
#define BITCRUSHER_DECIMATOR 5
#define BITWISE_KO 6
///////////////////////////////////////////
// mass storage end
///////////////////////////////////////////
#define pi 3.14159
#define Fs 44100
#define	FA_CREATE_ALWAYS	0x08
#define	FA_WRITE			0x02

#define LED_RED  GPIO_PIN_1
#define LED_BLUE  GPIO_PIN_2
///////////////////////////////////////////
// Define rows for matrix
// PORT F
///////////////////////////////////////////
#define ROW_1 GPIO_PIN_0
#define ROW_2 GPIO_PIN_1
#define ROW_3 GPIO_PIN_3  //GPIO_PIN_2 used for led troubleshooting and indication
#define ROW_4 GPIO_PIN_4
///////////////////////////////////////////
// Define cols for matrix
// PORT C
///////////////////////////////////////////
#define COL_1 GPIO_PIN_4
#define COL_2 GPIO_PIN_5
#define COL_3 GPIO_PIN_6
#define COL_4 GPIO_PIN_7
///////////////////////////////////////////
//Port E - LED'S GND
#define LED_GND1 GPIO_PIN_5 // E
#define LED_GND2 GPIO_PIN_0 // E
///////////////////////////////////////////
//Port A - LED'S GND
#define LED_GND3 GPIO_PIN_6
#define LED_GND4 GPIO_PIN_7
///////////////////////////////////////////
// Port D - LED YELLOW
#define LED_YEL1 GPIO_PIN_2 // SW 4
#define LED_YEL2 GPIO_PIN_3 // SW 3
#define LED_YEL3 GPIO_PIN_6 // SW 2
#define LED_YEL4 GPIO_PIN_7 // SW 1
///////////////////////////////////////////
// Port D & B - LED GREEN
#define LED_GRN1 GPIO_PIN_0 // SW 4 - D
#define LED_GRN2 GPIO_PIN_1 // SW 3 - D
#define LED_GRN3 GPIO_PIN_0 // SW 2 -B
#define LED_GRN4 GPIO_PIN_1 // SW 1 - B
///////////////////////////////////////////
// Stethoscope
///////////////////////////////////////////
#define ChestSounds GPIO_PIN_2 // SW 1 - F
#define ExtendedMode GPIO_PIN_2 // SW 1 - B
#define TEMPO_LED GPIO_PIN_3
#define LED_1_GND GPIO_PIN_6
#define NUM_SAMPLES 128
#define BLOCK_SIZE 128
#define BUFFER_SIZE 256
///////////////////////////////////////////
// DACWrite variables
///////////////////////////////////////////
char test = 0;
unsigned short data;
unsigned short command;
///////////////////////////////////////////
// Biquad Gene
///////////////////////////////////////////
float32_t Fc;
float32_t Q;
int filter_1 = 1;
int filter_2 = 2;
q15_t pCoeffs1[5], pCoeffs2[5];
uint8_t numStages = 1;
q15_t pState1[4], pState2[4];
int8_t postShift = 1;
unsigned long ulADC0_Value[4];
///////////////////////////////////////////
// Biquad
///////////////////////////////////////////
arm_biquad_casd_df1_inst_q15 S1; // = {1, pState1, pCoeffs1};
///////////////////////////////////////////
//CIRCULAR BUFFER
///////////////////////////////////////////
q15_t buffer[3][BUFFER_SIZE];
short index = 0; // The tail of the loop
short micValue;
short inIndex = 0;
short outIndex = 0;
short readBlock = 0;
short filterBlock = 1;
short writeBlock = 2;
bool filterWaiting = false;
q15_t tempSrc[2][BLOCK_SIZE];
int i;
static FATFS fso; // The FILINFO structure holds a file information returned by f_stat and f_readdir function
///////////////////////////////////////////
// Prototypes
///////////////////////////////////////////
void DACWrite(unsigned short command,short data);
void InitSPI(void);
void ADCIntHandler(void);
void coeff_gen(char type, float32_t Fc, float32_t Q, q15_t *pCoeffs);
void applyFilter(int filterUsed);
// mass storage starts

//*****************************************************************************
//
// The number of ticks to wait before falling back to the idle state.  Since
// the tick rate is 100Hz this is approximately 3 seconds.
//
//*****************************************************************************
#define USBMSC_ACTIVITY_TIMEOUT 300

// Global Variables
volatile unsigned long g_ulTimeStamp = 0; // Time since boot in 10ms increments
extern unsigned long g_ulIdleTimeout;


//*****************************************************************************
//
// This enumeration holds the various states that the device can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
	//
	// Unconfigured.
	//
	MSC_DEV_DISCONNECTED,

	//
	// Connected but not yet fully enumerated.
	//
	MSC_DEV_CONNECTED,

	//
	// Connected and fully enumerated but not currently handling a command.
	//
	MSC_DEV_IDLE,

	//
	// Currently reading the SD card.
	//
	MSC_DEV_READ,

	//
	// Currently writing the SD card.
	//
	MSC_DEV_WRITE,
}
g_eMSCState;

//*****************************************************************************
//
// The Flags that handle updates to the status area to avoid drawing when no
// updates are required..
//
//*****************************************************************************
#define FLAG_UPDATE_STATUS      1
static unsigned long g_ulFlags;
static unsigned long g_ulIdleTimeout;

//******************************************************************************
//
// The DMA control structure table.
//
//******************************************************************************
#ifdef ewarm
#pragma data_alignment=1024
tDMAControlTable sDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(sDMAControlTable, 1024)
tDMAControlTable sDMAControlTable[64];
#else
tDMAControlTable sDMAControlTable[64] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
RxHandler(void *pvCBData, unsigned long ulEvent,
		unsigned long ulMsgValue, void *pvMsgData)
{
	return(0);
}
//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
		void *pvMsgData)
{
	return(0);
}

//*****************************************************************************
//
// This function is the call back notification function provided to the USB
// library's mass storage class.
//
//*****************************************************************************
unsigned long
USBDMSCEventCallback(void *pvCBData, unsigned long ulEvent,
		unsigned long ulMsgParam, void *pvMsgData)
{
	//
	// Reset the time out every time an event occurs.
	//
	g_ulIdleTimeout = USBMSC_ACTIVITY_TIMEOUT;

	switch(ulEvent)
	{
	//
	// Writing to the device.
	//
	case USBD_MSC_EVENT_WRITING:
	{
		//
		// Only update if this is a change.
		//
		if(g_eMSCState != MSC_DEV_WRITE)
		{
			//
			// Go to the write state.
			//
			g_eMSCState = MSC_DEV_WRITE;

			//
			// Cause the main loop to update the screen.
			//
			g_ulFlags |= FLAG_UPDATE_STATUS;
		}

		break;
	}

	//
	// Reading from the device.
	//
	case USBD_MSC_EVENT_READING:
	{
		//
		// Only update if this is a change.
		//
		if(g_eMSCState != MSC_DEV_READ)
		{
			//
			// Go to the read state.
			//
			g_eMSCState = MSC_DEV_READ;

			//
			// Cause the main loop to update the screen.
			//
			g_ulFlags |= FLAG_UPDATE_STATUS;
		}

		break;
	}
	//
	// The USB host has disconnected from the device.
	//
	case USB_EVENT_DISCONNECTED:
	{
		//
		// Go to the disconnected state.
		//
		g_eMSCState = MSC_DEV_DISCONNECTED;

		//
		// Cause the main loop to update the screen.
		//
		g_ulFlags |= FLAG_UPDATE_STATUS;

		break;
	}
	//
	// The USB host has connected to the device.
	//
	case USB_EVENT_CONNECTED:
	{
		//
		// Go to the idle state to wait for read/writes.
		//
		g_eMSCState = MSC_DEV_IDLE;

		break;
	}
	case USBD_MSC_EVENT_IDLE:
	default:
	{
		break;
	}
	}

	return(0);
}

/* SysTick Handler
 *
 * - Required by FatFS for SD card timings
 * - On a 10ms interrupt
 */
void SysTickHandler(void) {
	// Alert FatFS and increment the timestamp
	disk_timerproc();
	++g_ulTimeStamp;
	if(g_ulIdleTimeout > 0) {
		g_ulIdleTimeout--;
	}
}

// mass storage ends

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
unsigned long g_ulFlags;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//////////////////////////////////////////////////////////////////////
// SD disk initialize port A and mounting
//////////////////////////////////////////////////////////////////////
void SD_init(void) {

	while(disk_initialize(0));
	f_mount(0, &fso);
}

//////////////////////////////////////////////////////////////////////
// Set up DAC port B
//////////////////////////////////////////////////////////////////////
void InitSPI(void) {
	// Used SD to DAC communication
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// Connect SPI to PB4(CLK) and PB7(TX), PB5(CS)
	GPIOPinConfigure(GPIO_PB4_SSI2CLK);
	GPIOPinConfigure(GPIO_PB5_SSI2FSS);
	GPIOPinConfigure(GPIO_PB7_SSI2TX);
	GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);
	//
	// Configure SSI2
	//
	SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,  SSI_MODE_MASTER, 20000000, 16);
	//
	// Enable the SSI module.
	//
	SSIEnable(SSI2_BASE);
}

//////////////////////////////////////////////////////////////////////
// set command, mask and data commands
//////////////////////////////////////////////////////////////////////
void DACWrite(unsigned short command, short data) {
	uint16_t write = 0;
	// set command, mask and data commands
	write = 0x3000 |  write;

	SSIDataPut(SSI2_BASE, write);
	//
	// Wait until SSI is done transferring all the data in the transmit FIFO
	//
	while(SSIBusy(SSI2_BASE))
	{
	}
}

void ADC_initialise(void)
{
	// The ADC peripherals enabled
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlADCSpeedSet(SYSCTL_ADCSPEED_500KSPS);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

	// configure the steps of the Sequencer
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH9 | ADC_CTL_IE |
			ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 0);
	ADCIntEnable(ADC0_BASE, 0);
	IntEnable(INT_ADC0SS0);
	ADCIntClear(ADC0_BASE, 0);
}

void readIn(short micValue) {
	buffer[readBlock][outIndex] = micValue;
}

void ADCIntHandler(void)
{
	//while(!ADCIntStatus(ADC0_BASE, 0, false)) {
	//}
	ADCIntClear(ADC0_BASE, 0);
	ADCSequenceDataGet(ADC0_BASE, 0, ulADC0_Value);
	//   UARTprintf("FX-1A = %5d, FX-1B = %5d, FX-2A = %5d, FX-2B = %5d\r\n",
	//	 ulADC0_Value[0], ulADC0_Value[1], ulADC0_Value[2], ulADC0_Value[3]);
	readIn(ulADC0_Value[0]);

}

void applyFilter(int filterUsed){
	if (GPIOPinRead(GPIO_PORTF_BASE, ChestSounds) == ChestSounds) {
		Fc = 1000.0f; // Chest sounds
	} else if (GPIOPinRead(GPIO_PORTF_BASE, ExtendedMode) == ExtendedMode) {
		Fc = 4000.0f; // Extend mode
	} else {
		Fc = 250.0F; // Heart Ascultations
	}
	Q = 0.707f;
	coeff_gen('L', Fc, Q, pCoeffs1); // Not sure what pCoeffs1 is
    arm_copy_q15(buffer[filterBlock], &tempSrc[0][0], BLOCK_SIZE);
	arm_biquad_cascade_df1_q15(&S1, &tempSrc[0][0], &buffer[filterBlock], BLOCK_SIZE);
}

void coeff_gen(char type, float32_t Fc, float32_t Q, q15_t *pCoeffs) {
	float b1_output = 0;
	float b2_output = 0;
	float b3_output = 0;
	float a2_output = 0;
	float a3_output = 0;
	float a0, a1, a2, b0, b1, b2 = 0;
	float w0 = 2.0*pi*Fc/Fs; // digital cutoff frequency
	float c1 = arm_cos_f32(w0);
	float alpha = arm_sin_f32(w0) / (2.0 * Q);
	switch (type) {
	case 'L':                      // LOW PASS
		b0 = (1.0 - c1)/2.0;
		b1 = 1.0 - c1;
		b2 = (1.0 - c1)/2.0;
		a0 = 1.0 + alpha;
		a1 = -2.0 * c1;
		a2 = 1.0 - alpha;
		break;
	case 'H':                      // HIGH PASS
		b0 = (1.0 + c1)/2.0;
		b1 = -1.0 - c1;
		b2 = (1.0 + c1)/2.0;
		a0 = 1.0 + alpha;
		a1 = -2.0 * c1;
		a2 = 1.0 - alpha;
		break;
	case 'B':                      // BAND PASS
		b0 = alpha;
		b1 = 0.0;
		b2 = -alpha;
		a0 = 1.0 + alpha;
		a1 = -2.0 * c1;
		a2 = 1.0 - alpha;
		break;
	case 'N':                      // NOTCH
		b0 = 1.0;
		b1 = -2.0*c1;
		b2 = 1.0;
		a0 = 1.0 + alpha;
		a1 = -2.0*c1;
		a2 = 1.0 - alpha;
		break;
	default:
		break;
	}

	b1_output = b0/a0;
	b2_output = b1/a0;
	b3_output = b2/a0;

	//a1_output = 1;//wont need
	a2_output = (a1/a0)*-1;
	a3_output = (a2/a0)*-1;

	pCoeffs[0] = b1_output;
	pCoeffs[1] = b2_output;
	pCoeffs[2] = b3_output;
	pCoeffs[3] = a2_output;
	pCoeffs[4] = a3_output;
}


void Timer1IntHandler(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	DACWrite(3, buffer[writeBlock][outIndex]);
	outIndex++;

	if(outIndex >= BUFFER_SIZE) {
		outIndex = 0;
		inIndex = 0;
		readBlock++;
		filterBlock++;
		writeBlock++;
		if(readBlock > 2) {
			readBlock = 0;
		}
		if(filterBlock > 2) {
			filterBlock = 0;
		}
		if(writeBlock > 2) {
			writeBlock = 0;
		}
		filterWaiting = true;
	}

	ADCProcessorTrigger(ADC0_BASE, 0);
}

void Timer2IntHandler(void) {
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

}

void Timer0IntHandler(void) {
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

PortFunctionInit(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 );
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | LED_BLUE | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, TEMPO_LED);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, ChestSounds);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ExtendedMode);
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,
			GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD); //PUTS LOW ON COL'S WHEN NOT USED
	ROM_GPIODirModeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_DIR_MODE_OUT);
	ROM_GPIODirModeSet(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_DIR_MODE_IN);

	//First open the lock and select the bits we want to modify in the GPIO commit register.
	//
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;


	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_5);

	// initialise filters
	arm_biquad_cascade_df1_init_q15(&S1, 1, pCoeffs1, pState1, postShift);

}


void initialLED(){

	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, LED_YEL1 | LED_YEL2 | LED_YEL3 | LED_YEL4);

	GPIOPadConfigSet(GPIO_PORTD_BASE, LED_YEL1 | LED_YEL2 | LED_YEL3 | LED_YEL4,
			GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, LED_GND1 | LED_GND2);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, LED_GND3 | LED_GND4);

	GPIOPinWrite(GPIO_PORTE_BASE, LED_GND1 | LED_GND2, LED_GND1 | LED_GND2);
	GPIOPinWrite(GPIO_PORTA_BASE, LED_GND3 | LED_GND4, LED_GND3 | LED_GND4);
	GPIOPinWrite(GPIO_PORTD_BASE, LED_YEL1 | LED_YEL2 | LED_YEL3 | LED_YEL4, 0);
}

///////////////////////////////////////////
// MAIN.C
///////////////////////////////////////////
void main(void) {

	///////////////////////////////////////////////////////////////
	// Set the clocking to run directly from the crystal.
	///////////////////////////////////////////////////////////////
	unsigned long ulRetcode;

	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL| SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);

	// mass storage start
	// Enable SysTick for FatFS at 10ms intervals
	ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / 100);
	ROM_SysTickEnable();
	ROM_SysTickIntEnable();
	//
	// Configure and enable uDMA
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	SysCtlDelay(10);
	ROM_uDMAControlBaseSet(&sDMAControlTable[0]);
	ROM_uDMAEnable();
	//
	// Enable the USB controller.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
	//
	// Set the USB pins to be controlled by the USB controller.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	// Initialize the idle timeout and reset all flags.
	//
	g_ulIdleTimeout = 0;
	g_ulFlags = 0;

	//
	// Initialize the state to idle.
	//
	g_eMSCState = MSC_DEV_DISCONNECTED;

	//
	// Set the USB stack mode to Device mode with VBUS monitoring.
	//
	USBStackModeSet(0, USB_MODE_DEVICE, 0);

	//
	// Pass our device information to the USB library and place the device
	// on the bus.
	//
	USBDMSCInit(0, (tUSBDMSCDevice *)&g_sMSCDevice);

	//
	// Determine whether or not an SDCard is installed.  If not, print a
	// warning and have the user install one and restart.
	//
	ulRetcode = disk_initialize(0);

	// mass storage end

	FPULazyStackingEnable();
	FPUEnable();

	///////////////////////////////////////////////////////////////
	// Initialize the UART and write status.
	///////////////////////////////////////////////////////////////
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioInit(0);
	InitSPI();
	while(disk_initialize(0));
	f_mount(0, &fso);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	// Enable processor interrupts.
	//
	IntMasterEnable();
	// Configure the two 32-bit periodic timers.
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 44100);
	//
	// Setup the interrupts for the timer timeouts.
	//
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	ADC_initialise();
	PortFunctionInit();
	// Enable the timers.
	TimerEnable(TIMER1_BASE, TIMER_A);
	while (1){
//		if(filterWaiting) {
//			filterWaiting = 0;
//			applyFilter(filter_1);
//		}
	}
}
