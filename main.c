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
#define Fs 88200.0
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
///////////////////////////////////////////
// Stethoscope
///////////////////////////////////////////
#define HeartSounds GPIO_PIN_0 // PB0
#define ChestSounds GPIO_PIN_1 // PB1
#define ExtendedMode1 GPIO_PIN_2 // PB2
#define ExtendedMode2 GPIO_PIN_3 // PB3
#define Mixer GPIO_PIN_0 // PD0
#define Recording GPIO_PIN_2 // PF2
#define FiltSpare1 GPIO_PIN_1 // PF1
#define FiltSpare2 GPIO_PIN_3 // PF3
// Testing
#define LED_1_GND GPIO_PIN_6
#define NUM_SAMPLES 512
#define BLOCK_SIZE 512
#define BUFFER_SIZE 512 //512
///////////////////////////////////////////
// DACWrite variables
///////////////////////////////////////////
char test = 0;
unsigned short command;
///////////////////////////////////////////
// Biquad Gene
///////////////////////////////////////////
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
float rec[BUFFER_SIZE];
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
static FIL file;
WORD bw;
uint8_t prevRecordState = 0;
uint32_t noOfSamples = 0;
///////////////////////////////////////////
// High Frequency Segment
///////////////////////////////////////////
short sine_Gen[200] = { 2048, 2112, 2176, 2240, 2304, 2368, 2431, 2494, 2557, 2619, 2680, 2741, 2801, 2861, 2919, 2977, 3034, 3090, 3145, 3198, 3251, 3302, 3353, 3402, 3449, 3495, 3540,
3583, 3625, 3665, 3704, 3741, 3776, 3810, 3842, 3872, 3900, 3927, 3951, 3974, 3995, 4014, 4031, 4046, 4059, 4070, 4079, 4086, 4091, 4094, 4095, 4094, 4091, 4086,
4079, 4070, 4059, 4046, 4031, 4014, 3995, 3974, 3951, 3927, 3900, 3872, 3842, 3810, 3776, 3741, 3704, 3665, 3625, 3583, 3540, 3495, 3449, 3402, 3353, 3302, 3251,
3198, 3145, 3090, 3034, 2977, 2919, 2861, 2801, 2741, 2680, 2619, 2557, 2494, 2431, 2368, 2304, 2240, 2176, 2112, 2048, 1983, 1919, 1855, 1791, 1727, 1664, 1601,
1538, 1476, 1415, 1354, 1294, 1234, 1176, 1118, 1061, 1005, 950, 897, 844, 793, 742, 693, 646, 600, 555, 512, 470, 430, 391, 354, 319, 285, 253, 223, 195, 168,
144, 121, 100, 81, 64, 49, 36, 25, 16, 9, 4, 1, 0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100, 121, 144, 168, 195, 223, 253, 285, 319, 354, 391, 430, 470, 512, 555, 600,
646, 693, 742, 793, 844, 897, 950, 1005, 1061, 1118, 1176, 1234, 1294, 1354, 1415, 1476, 1538, 1601, 1664, 1727, 1791, 1855, 1919, 1983};

float val = 0;
///////////////////////////////////////////
// Biquad
///////////////////////////////////////////
arm_biquad_casd_df1_inst_f32 S;
///////////////////////////////////////////
// Wav file struct
///////////////////////////////////////////
typedef struct  WAV_HEADER
{
	uint32_t      RIFF;        /* RIFF Header      */ //Magic header
	uint32_t      ChunkSize;      /* RIFF Chunk Size  */
	uint32_t      WAVE;        /* WAVE Header      */
	uint32_t      fmt;         /* FMT header       */
	uint32_t	  Subchunk1Size;  /* Size of the fmt chunk                                */
	uint16_t      AudioFormat;    /* Audio format 1=PCM,6=mulaw,7=alaw, 257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM */
	uint16_t      NumOfChan;      /* Number of channels 1=Mono 2=Sterio                   */
	uint32_t      SamplesPerSec;  /* Sampling Frequency in Hz                             */
	uint32_t      bytesPerSec;    /* bytes per second */
	uint16_t      blockAlign;     /* 2=16-bit mono, 4=16-bit stereo */
	uint16_t      bitsPerSample;  /* Number of bits per sample      */      // should this be uint32_t
	uint32_t      Subchunk2ID; /* "data"  string   */
	uint32_t      Subchunk2Size;  /* Sampled data length    */

}wav_hdr;


///////////////////////////////////////////
// Prototypes
///////////////////////////////////////////
void DACWrite(unsigned short command,float data);
void InitSPI(void);
void ADCIntHandler(void);
void coeff_gen(char type, float32_t Fc, float32_t Q);
void applyFilter(int filterUsed);
void record_start(char *Info);
void record_Stop(void);
void write_Header(FIL* file, uint32_t noOfSamples);
void sine_mixer(uint16_t sineFreq);
void applyFilter2(int filterUsed);


int cycles = 0;

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
void record_start(char *Info) {
	f_open(&file, Info, FA_CREATE_ALWAYS | FA_WRITE);
	f_lseek(&file, sizeof(wav_hdr));
}

void record_Stop(void){
	f_lseek(&file, 0);
	write_Header(&file, noOfSamples);
	f_sync(&file);
}

void write_Header(FIL* file, uint32_t noOfSamples) {
	wav_hdr hdr;

	hdr.RIFF = 0x46464952; //0x52494646  big endian was - "RIFF"
	hdr.ChunkSize = 4*noOfSamples+36;
	hdr.WAVE =  0x45564157; // little endian was - "WAVE"  0x57415645
	hdr.fmt = 0x20746d66; //0x666d7420
	hdr.Subchunk1Size = 16;
	hdr.AudioFormat = 3;
	hdr.NumOfChan = 1;
	hdr.SamplesPerSec = 88200;
	hdr.bytesPerSec  = 352800; //176400;    //352800; // 44100 * 4;
	hdr.blockAlign = 4;
	hdr.bitsPerSample = 32;
	hdr.Subchunk2ID =  0x61746164; // 0x64617461 "data"
	hdr.Subchunk2Size = 4*noOfSamples;  //NumSamples * 4;

	f_write(file, &hdr, sizeof hdr, &bw);
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

void sine_mixer(uint16_t sineFreq) {
    float sine;
    int m;
    short fOld = filterBlock;
    for (m = 0; m <= BLOCK_SIZE; m++){
    	if (fOld != filterBlock) {
    		break;
    	}
    	sine = (((float)sine_Gen[(uint32_t)val] * 2) - 4096) / 4096;
    	buffer[filterBlock][m] *= sine;
    	val += (sineFreq/(441)); // 882 220.5 (441/2) @ 44100kHz
      	if (val >= 200.0) {
    		val -= 200.0;
    	}
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
	float32_t Fc;
	float32_t Q;
	if (GPIOPinRead(GPIO_PORTB_BASE, HeartSounds) == HeartSounds) {
		Fc = 300.0f; // Extend mode
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTB_BASE, ChestSounds) == ChestSounds)  {
		Fc = 1000.0f; // Heart Ascultations
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTB_BASE, ExtendedMode1) == ExtendedMode1)  {
		Fc = 2000.0f; // Extended1
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTB_BASE, ExtendedMode2) == ExtendedMode2) {
		Fc = 4000.0f; // Extended2
		Q = 0.3f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTD_BASE, Mixer) == Mixer) {
		Fc = 16000.0f; // HF Mixing
		Q = 0.7f;
		coeff_gen('H', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTF_BASE, FiltSpare1) == FiltSpare1) {
		Fc = 100.0f; // Spare 1
		Q = 0.7f;
		coeff_gen('L', Fc, Q);
	} else if (GPIOPinRead(GPIO_PORTF_BASE, FiltSpare2) == FiltSpare2) {
		Fc = 1000.0f; // Spare 2
		Q = 0.7f;
		coeff_gen('H', Fc, Q);
	} else {
		// Normal Use
		Fc = 15000.0f;
		Q = 0.7f;
		coeff_gen('L', Fc, Q);
	}

	arm_copy_f32(&buffer[filterBlock][0], &tempSrc[0], BLOCK_SIZE);
	arm_biquad_cascade_df1_f32(&S, &tempSrc[0], &buffer[filterBlock][0], BLOCK_SIZE);

	uint8_t recordState = (GPIOPinRead(GPIO_PORTF_BASE, Recording) == Recording);

	if (!prevRecordState && (GPIOPinRead(GPIO_PORTF_BASE, Recording) == Recording)) {	 //Rising edge
		record_start(Info);
	}
	//if (recordState) {		//High state
	if (GPIOPinRead(GPIO_PORTF_BASE, Recording) == Recording){     //High state
		f_write(&file, buffer[filterBlock], BLOCK_SIZE*sizeof(float), &bw); // buffer[filterBlock]
		noOfSamples += BLOCK_SIZE;
	} else
	if (prevRecordState != 0) {		//Falling edge
		record_Stop();
	}
	prevRecordState = recordState;

}

void coeff_gen(char type, float32_t Fc, float32_t Q) {
	float b1_output = 0;
	float b2_output = 0;
	float b3_output = 0;
	float a2_output = 0;
	float a3_output = 0;
	float a0, a1, a2, b0, b1, b2 = 0;
	float w0 = 2.0*pi*Fc/Fs; // 44.1kHz or 82.0kHz sampling
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

void PortFunctionInit(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 );
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | LED_BLUE | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, HeartSounds);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ChestSounds);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, FiltSpare1);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, FiltSpare2);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ExtendedMode1);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ExtendedMode2);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, Mixer);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, Recording);
	ROM_GPIODirModeSet(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_DIR_MODE_OUT);
	ROM_GPIODirModeSet(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_DIR_MODE_IN);

	//First open the lock and select the bits we want to modify in the GPIO commit register.
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_5);

	// Initialise biquad filter
	arm_biquad_cascade_df1_init_f32(&S, 1, &coeffs[0], &state[0]);
}

void initialLED(){
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, LED_GND1 | LED_GND2);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, LED_GND3 | LED_GND4);
	GPIOPinWrite(GPIO_PORTE_BASE, LED_GND1 | LED_GND2, LED_GND1 | LED_GND2);
	GPIOPinWrite(GPIO_PORTA_BASE, LED_GND3 | LED_GND4, LED_GND3 | LED_GND4);
}

///////////////////////////////////////////
// MAIN.C
///////////////////////////////////////////
void main(void) {

	///////////////////////////////////////////////////////////////
	// Set the clocking to run directly from the crystal.
	///////////////////////////////////////////////////////////////
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL| SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);

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
	while(disk_initialize(0)); // SD card
	// mass storage end
	FPUStackingEnable();
	FPUFlushToZeroModeSet(FPU_FLUSH_TO_ZERO_EN);
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
	f_mount(0, &fso);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	// Enable processor interrupts.
	IntMasterEnable();
	// Configure the two 32-bit periodic timers.
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 88200);
    // Setup the interrupts for the timer timeouts.
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER1_BASE, TIMER_A);


	while (1){
		if(filterWaiting == 1) {
			if (GPIOPinRead(GPIO_PORTD_BASE, Mixer) == Mixer){
				applyFilter(1);
				sine_mixer(30000); // 900 Hz approx
			} else {
			   applyFilter(1);

			}
			filterWaiting = 0;
		}

	}
}
