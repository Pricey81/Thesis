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
#include <stdio.h>
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
#define Fs 44100.0
#define	FA_CREATE_ALWAYS	0x08
#define	FA_WRITE			0x02

#define LED_RED  GPIO_PIN_1
#define LED_BLUE  GPIO_PIN_2
///////////////////////////////////////////
// Define rows for matrix
// PORT F
///////////////////////////////////////////
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
///////////////////////////////////////////
// Stethoscope
///////////////////////////////////////////
#define ChestSounds GPIO_PIN_2 // SW 1 - F
#define ExtendedMode GPIO_PIN_2 // SW 1 - B
#define HeartSounds GPIO_PIN_3
#define HighRange GPIO_PIN_0 // SW 2 -B
#define HighRange2 GPIO_PIN_1 // SW 1 - B
#define Recording GPIO_PIN_0 // Port F
#define LED_1_GND GPIO_PIN_6
#define NUM_SAMPLES 512
#define BLOCK_SIZE 512
#define BUFFER_SIZE 512
///////////////////////////////////////////
// DACWrite variables
///////////////////////////////////////////
char test = 0;
//unsigned short testData; // changed from unsigned short
unsigned short command;
///////////////////////////////////////////
// Biquad Gene
///////////////////////////////////////////
float32_t Fc;
float32_t Q;
int filter_1 = 1;
int filter_2 = 2;
float32_t coeffs[5]; //5
uint8_t numStages = 1;
static float32_t state[4]; //4

int8_t postShift = 1;
unsigned long ulADC0_Value[4];
///////////////////////////////////////////
//CIRCULAR BUFFER
///////////////////////////////////////////
float buffer[3][BUFFER_SIZE];
short index = 0; // The tail of the loop
short inIndex = 0;
short outIndex = 0;
short readBlock = 0;
short filterBlock = 1;
short writeBlock = 2;
unsigned short filterWaiting = 0;
float tempSrc[BLOCK_SIZE];
int i;
///////////////////////////////////////////
// SD Card
static FATFS fso; // The FILINFO structure holds a file information returned by f_stat and f_readdir function
char Info[]= "steth.wav";
//char testData[] = "hello";
static FIL file;
WORD size;
WORD bw;
uint16_t write;
///////////////////////////////////////////
// Biquad
///////////////////////////////////////////
arm_biquad_casd_df1_inst_f32 S;
///////////////////////////////////////////
// Wav file struct
///////////////////////////////////////////
typedef struct  WAV_HEADER
{
	char                RIFF[4];        /* RIFF Header      */ //Magic header
	unsigned long       ChunkSize;      /* RIFF Chunk Size  */
	char                WAVE[4];        /* WAVE Header      */
	char                fmt[4];         /* FMT header       */
	unsigned long       Subchunk1Size;  /* Size of the fmt chunk                                */
	unsigned short      AudioFormat;    /* Audio format 1=PCM,6=mulaw,7=alaw, 257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM */
	unsigned short      NumOfChan;      /* Number of channels 1=Mono 2=Sterio                   */
	unsigned long       SamplesPerSec;  /* Sampling Frequency in Hz                             */
	unsigned long       bytesPerSec;    /* bytes per second */
	unsigned short      blockAlign;     /* 2=16-bit mono, 4=16-bit stereo */
	unsigned short      bitsPerSample;  /* Number of bits per sample      */
	char                Subchunk2ID[4]; /* "data"  string   */
	unsigned long       Subchunk2Size;  /* Sampled data length    */
} wav_hdr;
///////////////////////////////////////////
// Prototypes
///////////////////////////////////////////
void DACWrite(unsigned short command,float data);
void InitSPI(void);
void ADCIntHandler(void);
void coeff_gen(char type, float32_t Fc, float32_t Q);
void applyFilter(int filterUsed);
void record_Wav(char *Info, float *testData,  WORD size);
void init_after_header_commence(FIL* file);
void record_Stop(void);
//void write_Header(file, WAV_HEADER *wav_hdr);


int filterCount[1];

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
// SD open and read a file, used for testing purposes
//////////////////////////////////////////////////////////////////////
void record_Wav(char *Info, float *testData,  WORD size) {
	WORD bw;

	f_open(&file, Info, FA_CREATE_ALWAYS | FA_WRITE);
	init_after_header_commence(&file);
	f_write(&file, testData, size, &bw);
	f_sync(&file);
}

void record_Stop(void){
	f_lseek(&file, 0);
//	write_Header(file, struct WAV_HEADER *wav_hdr);
	f_close(&file);
}

void init_after_header_commence(FIL* file) {
	f_lseek(file, sizeof(wav_hdr));
}
/*
void header_Values(void){
	memcpy(&wav_hdr->RIFF, 0x64649425, 4); // little endian 0x64649425 //"RIFF"
	memcpy(&wav_hdr->ChunkSize, 4 + (8 + SubChunk1Size) + (8 + SubChunk2Size), 4);
	memcpy(&wav_hdr->WAVE, 0x54651475, 4); // 0x54651475 //"WAVE"
	memcpy(&wav_hdr->fmt, "fmt ", 4);
	wav_hdr->AudioFormat = 3;
	wav_hdr->NumOfChan = 1;
	wav_hdr->SamplesPerSec = 44100;
	wav_hdr->bytesPerSec = 44100 * BitsPerSample/8;
	wav_hdr->blockAlign = BitsPerSample/8;
	wav_hdr->bitsPerSample = 32;
	wav_hdr->Subchunk2ID = "data";
	wav_hdr->Subchunk2Size =  NumSamples * 4; num samples = 256??

	return wav_hdr;
}*/
/*
void write_Header(file, struct WAV_HEADER *wav_hdr){

	f_write(file, &wav_hdr->RIFF, 4, &bw);
	f_write(file, &wav_hdr->ChunkSize, 4, &bw);
	f_write(file, &wav_hdr->WAVE, 4, &bw);
	f_write(file, &wav_hdr->fmt, 4, &bw);
	f_write(file, &wav_hdr->Subchunk1Size, 4, &bw);
	f_write(file, &wav_hdr->AudioFormat, 2, &bw);
	f_write(file, &wav_hdr->NumOfChan, 2, &bw);
	f_write(file, &wav_hdr->SamplesPerSec, 4, &bw);
	f_write(file, &wav_hdr->bytesPerSec, 4, &bw);
	f_write(file, &wav_hdr->blockAlign, 2, &bw);
	f_write(file, &wav_hdr->bitsPerSample, 2, &bw);
	f_write(file, &wav_hdr->Subchunk2ID, 4,&bw);
	f_write(file, &wav_hdr->Subchunk2Size, 4, &bw);


}*/
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
void DACWrite(unsigned short command, float data) {
	uint16_t write = 0;

	data = data * (4096 / 2);
	data = data + (4096 / 2);
	// set command, mask and data commands
 	write = (uint16_t) data;
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
	float fval = (float) micValue;
	fval = fval * 2;
	fval = fval - 4096.0f;
	fval = fval / 4096.0f;
	if(fval > 1) {
		fval = 1;
	}
	if(fval < -1) {
		fval = -1;
	}
	buffer[readBlock][outIndex] = fval;
}

void ADCIntHandler(void)
{
	ADCIntClear(ADC0_BASE, 0);
	ADCSequenceDataGet(ADC0_BASE, 0, ulADC0_Value);
	readIn(ulADC0_Value[0]);

}

void applyFilter(int filterUsed){

	if (GPIOPinRead(GPIO_PORTF_BASE, ChestSounds) == ChestSounds) {
		Fc = 800.0f; // Chest sounds
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTB_BASE, ExtendedMode) == ExtendedMode) {
		Fc = 2000.0f; // Extend mode
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTB_BASE, HeartSounds) == HeartSounds)  {
		Fc = 100.0f; // Heart Ascultations
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTB_BASE, HighRange) == HighRange) {
		Fc = 10000.0f; // high ranges
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTB_BASE, HighRange2) == HighRange2) {
		Fc = 15000.0f; // high ranges
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else {
		// Normal Use
		Fc = 1000.0f; // high ranges
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	}

	int k;
	for(k = 0; k < 4; k++) {
		if(isnan(state[k])) {
			state[k] = 0;
		}
	}

	arm_copy_f32(&buffer[filterBlock][0], &tempSrc[0], BLOCK_SIZE);
	arm_biquad_cascade_df1_f32(&S, &tempSrc[0], &buffer[filterBlock][0], BLOCK_SIZE);

	// Send data to SD card upon record button pressed
	if (GPIOPinRead(GPIO_PORTB_BASE, Recording) == Recording){
		record_Wav(Info, buffer[filterBlock], size);
		record_Stop();
	}
}

void coeff_gen(char type, float32_t Fc, float32_t Q) {
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
	a2_output = (a1/a0)*-1.0;
	a3_output = (a2/a0)*-1.0;

	coeffs[0] = b1_output;
	coeffs[1] = b2_output;
	coeffs[2] = b3_output;
	coeffs[3] = a2_output;
	coeffs[4] = a3_output;
}


void Timer1IntHandler(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	DACWrite(3, buffer[writeBlock][outIndex]);
	outIndex++;

	if(outIndex >= BUFFER_SIZE) {
		outIndex = 0;

		readBlock--;
		filterBlock--;
		writeBlock--;

		if(readBlock < 0) {
			readBlock = 2;
		}
		if(filterBlock < 0) {
			filterBlock = 2;
		}
		if(writeBlock <0) {
			writeBlock = 2;
		}
		filterWaiting = 1;
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
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, HeartSounds);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, ChestSounds);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ExtendedMode);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, HighRange);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, HighRange2);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, Recording);
//	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,
//			GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD); //PUTS LOW ON COL'S WHEN NOT USED
	ROM_GPIODirModeSet(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_DIR_MODE_OUT);
	ROM_GPIODirModeSet(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_DIR_MODE_IN);

	//First open the lock and select the bits we want to modify in the GPIO commit register.
	//
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_5);
    // Initialise biquad filter
	arm_biquad_cascade_df1_init_f32(&S, 1, &coeffs[0], &state[0]);
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

	filterCount[0] = 0;

	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL| SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);

	// mass storage start
	// Enable SysTick for FatFS at 10ms intervals
//	ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / 100);
//	ROM_SysTickEnable();
//	ROM_SysTickIntEnable();
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
//	USBDMSCInit(0, (tUSBDMSCDevice *)&g_sMSCDevice);
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
	PortFunctionInit();
	ADC_initialise();

    while(disk_initialize(0)); // sd card
	f_mount(0, &fso);
	//char testData[] = "hello there";
	//SD_Write(Info, testData, size);
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
	// Enable the timers.
	TimerEnable(TIMER1_BASE, TIMER_A);
	// testing SD Card

	//SD_Write(Info, "hello there", size);
	while (1){
		if(filterWaiting == 1) {
			filterWaiting = 0;
			applyFilter(filter_1);
		}
	}
}
