/**************************************************************
*
* Lattice Semiconductor Corp. Copyright 2008
* 
*
***************************************************************/


/**************************************************************
* 
* Revision History of hardware.c
* 
* 
* 09/11/07 NN type cast all the mismatch variables
***************************************************************/


#if defined(WIN32)
#include <windows.h>
#include <mmsystem.h>
#include <conio.h>
#else
#include <unistd.h>
#endif

#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <stdio.h>
#include <assert.h>

#include <libusb-1.0/libusb.h>
#include "ftdi.h"	// libftdi1-1.2 (needed for FT230X) See http://www.intra2net.com/en/developer/libftdi/ (or install Ubuntu libftdi1-dev)

#include "vmopcode.h"
#include "hardware.h"

#define	FT_OK	0

#define STR1(x) #x
#define STR(x) STR1(x)
#define	FT_CHECK(x)	(ftdi_status = (x), (ftdi_status >= 0 ? 0 : printf("FTDI: %s returned %d in %s:%d: %s\n", STR(x), (int32_t)ftdi_status, __FUNCTION__, __LINE__, ftdi_get_error_string(ftdi))), ftdi_status)

/********************************************************************************
* Declaration of global variables 
*
*********************************************************************************/

uint8_t  g_siIspPins        = 0x00;   /*Keeper of JTAG pin state*/

/*********************************************************************************
* This is the definition of the bit locations of each respective
* signal in the global variable g_siIspPins.
*
* NOTE: Users must add their own implementation here to define
*       the bit location of the signal to target their hardware.
*       The example below is for the Lattice download cable on
*       on the parallel port.
*
*********************************************************************************/

const uint8_t g_ucPinTDI          = 0x01;    /* Bit address of TDI */
const uint8_t g_ucPinTMS          = 0x02;    /* Bit address of TMS */
const uint8_t g_ucPinTDO          = 0x04;    /* Bit address of TDO*/
const uint8_t g_ucPinTCK          = 0x08;    /* Bit address of TCK */
const uint8_t g_ucPinENABLE       = 0x00;    /* Bit address of ENABLE */
const uint8_t g_ucPinTRST         = 0x00;    /* Bit address of TRST */

/***************************************************************
*
* Functions declared in hardware.c module.
*
***************************************************************/
void writePort( uint8_t a_ucPins, uint8_t a_ucValue );
uint8_t readPort(void);
void sclock(void);
void ispVMDelay( uint16_t a_usTimeDelay );
void calibration(void);

/*************************************************************
*                                                            *
* EXTERNAL FUNCTION                                          *
*                                                            *
*************************************************************/

//extern void ispVMStateMachine( char a_cNextState );

typedef int FT_STATUS;
typedef struct ftdi_context* FT_HANDLE;

FT_STATUS	ftdi_status;
FT_HANDLE	ftdi;

int32_t	CBUS_IO;
int32_t ABBM_IO;

static int32_t last_secs;
static int32_t last_clocks;
static int32_t last_ms;

int16_t	last_siIspPins;	// logical state of parallel port ISP pins
int16_t	last_siUSBPins;	// physical state of USB pins
int32_t	last_bitMode;
uint32_t OutputRunCount;
uint32_t LongestRun;
uint32_t USBTransactions;
uint32_t NumberClockRuns;
uint32_t TotalClocks;
uint32_t ClockUpdateFlush;
int32_t	TDOToggle;
uint32_t written;
int32_t line_open;
uint8_t JTAGBuffer[MAX_CLOCK_RUN*4];
uint32_t  JTAGCount;

char FleaFPGA_Default_Desc[64] = "FleaFPGA v2.5";	// this string can be overridden with -m
char *FleaFPGA_Desc[] = 
{
	FleaFPGA_Default_Desc,
	"FleaFPGA v2.4",
	"FleaFPGA v2.3",
	"FleaFPGA v2.2",
	"FT230X Basic UART",
};

int32_t gColumn;
int32_t gVerbose;
int32_t gParanoidSafety;
int32_t gSpecifiedDevice;
int32_t gAutomatic;
int32_t gJTAGMode = JTAG_FTDI_BITBANG_CBUS_READ;
char gKillTitleStrs[256];
char gRestartPath[256];
uint8_t in_byte = 0x00;

const char *FTDIModeName[JTAG_NUM_MODES] =
{
	"Test JTAG (no-op but expected input verifies correctly)",
	"prototype CBUS (CB0=TDI/CB1=TDO/CB2=TMS/CB3=TCK)",
	"bit-bang write, CBUS read (RTS=TDI/TX=TMS/CTS=TCK/CB1=TDO)"
};

static uint8_t HL[2] = { abbmTCK, 0 };

uint32_t g_TotalTime;
uint32_t g_JTAGTime;

uint32_t getMilliseconds(uint32_t start)
{
	struct timeval now;
	gettimeofday(&now, NULL);

	uint32_t nowms = (now.tv_sec * 1000) + (now.tv_usec / 1000);
	uint32_t ms = (nowms - start);

	if (ms == 0)
		ms = 1;	// avoid divide by zero

	return ms;
}

//******************
//Source
//******************

/*************************************************************
*                                                            *
* FlushData                                                  *
*                                                            *
* INPUT:                                                     *
*                                                            *
* RETURN:                                                    *
*                                                            *
* DESCRIPTION:                                               *
*     Flush any pending TDI/TMS bits to JTAG port            *
*                                                            *
*************************************************************/

void flushPort(void)
{
	switch (gJTAGMode)
	{
	case JTAG_FTDI_CBUS:
		{
			if (last_siUSBPins != g_siIspPins)
			{
				USBTransactions++;
				// write data & TCK
				FT_CHECK(ftdi_set_bitmode(ftdi, CBUS_IO | (g_siIspPins & g_ucPinTCK ? cbusTCK : 0) | (g_siIspPins & g_ucPinTMS ? cbusTMS : 0) | (g_siIspPins & g_ucPinTDI ? cbusTDI :0), BITMODE_CBUS));
				last_siUSBPins = g_siIspPins;
			}
		}
		break;

	case JTAG_FTDI_BITBANG_CBUS_READ:
		{
			if (OutputRunCount)
			{
				if (last_bitMode != BITMODE_BITBANG)
				{
					USBTransactions++;
					FT_CHECK(ftdi_set_bitmode(ftdi, ABBM_IO, BITMODE_BITBANG));
					last_bitMode = BITMODE_BITBANG;
				}
				USBTransactions++;
				FT_CHECK(written = ftdi_write_data(ftdi, JTAGBuffer, JTAGCount));		// clock -> L -> H -> L
				if (written != JTAGCount)
					printf("\n%s(%d): short ftdi_write_data result (%d vs %d)?\n", __FUNCTION__, __LINE__, (int32_t)written, JTAGCount);

				NumberClockRuns++;
				if (LongestRun < OutputRunCount)
				LongestRun = OutputRunCount;

				JTAGCount = 0;
				OutputRunCount = 0;
			}
		}
		break;

	default:
		break;
	}
}

/*************************************************************
*                                                            *
* WRITEPORT                                                  *
*                                                            *
* INPUT:                                                     *
*     a_ucPins: a byte to indicate which pin will be         *
*     depending on the value.                                *
*                                                            *
*     a_ucValue: the value to determine of the pin above     *
*     will be written out or not.                            *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     To apply the specified value to the pins indicated.    *
*     This routine will likely be modified for specific      *
*     systems. As an example, this code is for the PC, as    *
*     described below.                                       *
*                                                            *
*     This routine uses the IBM-PC standard Parallel port,   *
*     along with the schematic shown in Lattice              *
*     documentation, to apply the signals to the programming *
*     loop.                                                  *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/

void writePort( uint8_t a_ucPins, uint8_t a_ucValue )
{
	if ( a_ucValue ) {
		g_siIspPins = (int16_t) ((int16_t)a_ucPins | g_siIspPins);
	}
	else {
		g_siIspPins = (int16_t) ((int16_t)~a_ucPins & g_siIspPins);
	}

	switch (gJTAGMode)
	{
	case JTAG_FTDI_CBUS:
		{
			// rising edge of TCK?
			if (!(last_siIspPins & g_ucPinTCK) && (g_siIspPins & g_ucPinTCK))
			{
				// if the data bits aren't correct or clock is still high, set them now before the rising clock edge
				if (last_siUSBPins != (g_siIspPins & (g_ucPinTMS | g_ucPinTDI)))
				{
					USBTransactions++;
					FT_CHECK(ftdi_set_bitmode(ftdi, CBUS_IO | (g_siIspPins & g_ucPinTMS ? cbusTMS : 0) | (g_siIspPins & g_ucPinTDI ? cbusTDI : 0), BITMODE_CBUS));	// write data and low TCK
				}
				USBTransactions++;
				// write data & TCK
				FT_CHECK(ftdi_set_bitmode(ftdi, CBUS_IO | (g_siIspPins & g_ucPinTCK ? cbusTCK : 0) | (g_siIspPins & g_ucPinTMS ? cbusTMS : 0) | (g_siIspPins & g_ucPinTDI ? cbusTDI :0), BITMODE_CBUS));
				last_siUSBPins = g_siIspPins;
			}
		}
		break;

	case JTAG_FTDI_BITBANG_CBUS_READ:
		{
			// rising edge of TCK?
			if (!(last_siIspPins & g_ucPinTCK) && (g_siIspPins & g_ucPinTCK))
			{
				OutputRunCount++;

				JTAGBuffer[JTAGCount++] = (g_siIspPins & g_ucPinTDI ? abbmTDI : 0) | (g_siIspPins & g_ucPinTMS ? abbmTMS : 0);
				JTAGBuffer[JTAGCount++] = (g_siIspPins & g_ucPinTDI ? abbmTDI : 0) | (g_siIspPins & g_ucPinTMS ? abbmTMS : 0) | abbmTCK;
				JTAGBuffer[JTAGCount++] = (g_siIspPins & g_ucPinTDI ? abbmTDI : 0) | (g_siIspPins & g_ucPinTMS ? abbmTMS : 0);

				last_siUSBPins = g_siIspPins;

				if (OutputRunCount >= MAX_CLOCK_RUN)
					flushPort();
			}
		}
		break;

	default:
		break;
	}

	// is this a clock rising edge
	if (!(last_siIspPins & g_ucPinTCK) && (g_siIspPins & g_ucPinTCK))
	{
		TotalClocks++;
		if (gVerbose)
		{
			gColumn += printf(">%1X", g_siIspPins & (g_ucPinTDI|g_ucPinTMS));
			if (gColumn >= PRINT_MAX_COLUMN)
			{
				gColumn = 0;
				printf("\n");
			}
		}
	}
	last_siIspPins = g_siIspPins;

	if (TotalClocks >= ClockUpdateFlush)
	{
		ClockUpdateFlush = TotalClocks + 256;
		OpenLine(0);
	}
}

void OpenLine(int32_t force)
{
	if (!gVerbose)
	{
		int32_t ms = getMilliseconds(g_JTAGTime);
		int32_t secs = 1;
		uint32_t clocks = 0;
		if (force == 2)
		ms = g_JTAGTime;
		secs = ms / 1000;

		if (secs != last_secs || force)
		{
			clocks = ((TotalClocks - last_clocks) * 1000) / (ms - last_ms);
			last_secs = secs;
			printf("  JTAG bits clocked out/in: %d/%d  (time: %d:%02d.%d, bps: %d)    \r", TotalClocks, TDOToggle, ms / (60 * 1000), (ms / 1000) % 60, (ms % 1000) / 100, force != 2 ? clocks : (TotalClocks * 1000) / ms);
			last_clocks = TotalClocks;
			last_ms = ms;
			line_open = 1;
		}
	}
	if (!force)
	fflush(stdout);
}

void CloseLine(void)
{
	if (line_open)
	{
		printf("                                                                        \r");
		line_open = 0;
	}
}

/*************************************************************
*                                                            *
* READPORT                                                   *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     Returns the bit read back from the device.             *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to read the TDO pin from the     *
*     input port.                                            *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
uint8_t readPort(void)
{
	uint8_t ucRet = 0;
	uint8_t byte = 0x00;

	switch (gJTAGMode)
	{
	case JTAG_FTDI_CBUS:
		{
			flushPort();

			USBTransactions++;
			FT_CHECK(ftdi_read_pins(ftdi, &byte));
			ucRet = (byte & cbusTDO) ? 0x01 : 0x00;
		}
		break;

	case JTAG_FTDI_BITBANG_CBUS_READ:
		{
			TDOToggle++;

			flushPort();

			// this is required to get proper CBUS input (I believe forgets CBUS mode when in async bit-bang)
			USBTransactions++;
			FT_CHECK(ftdi_set_bitmode(ftdi, 0, BITMODE_CBUS));
			last_bitMode = BITMODE_CBUS;

			USBTransactions++;
			FT_CHECK(ftdi_read_pins(ftdi, &byte));
			ucRet = (byte & cbusTDO) ? 0x01 : 0x00;
		}
		break;

	default:
		break;
	}

	if (gVerbose && gJTAGMode != JTAG_NONE)	// expected input is printed at a higher level in test mode
	{
		gColumn += printf("=%1X", ucRet); 
		if (gColumn >= PRINT_MAX_COLUMN)
		{
			gColumn = 0;
			printf("\n");
		}
		fflush(stdout);
	}

	return ( ucRet );
} 

/*********************************************************************************
* sclock
*
* Apply a pulse to TCK.
*
* This function is located here so that users can modify to slow down TCK if
* it is too fast (> 25MHZ). Users can change the IdleTime assignment from 0 to 
* 1, 2... to effectively slowing down TCK by half, quarter...
*
*********************************************************************************/
void sclock(void)
{	
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );
}

/********************************************************************************
*
* ispVMDelay
*
*
* Users must implement a delay to observe a_usTimeDelay, where
* bit 15 of the a_usTimeDelay defines the unit.
*      1 = milliseconds
*      0 = microseconds
* Example:
*      a_usTimeDelay = 0x0001 = 1 microsecond delay.
*      a_usTimeDelay = 0x8001 = 1 millisecond delay.
*
* This subroutine is called upon to provide a delay from 1 millisecond to a few 
* hundreds milliseconds each time. 
* It is understood that due to a_usTimeDelay is defined as uint16_t, a 16 bits
* integer, this function is restricted to produce a delay to 64000 micro-seconds 
* or 32000 milli-second maximum. The VME file will never pass on to this function
* a delay time > those maximum number. If it needs more than those maximum, the VME
* file will launch the delay function several times to realize a larger delay time
* cumulatively.
* It is perfectly all right to provide a longer delay than required. It is not 
* acceptable if the delay is shorter.
*
* Delay function example--using the machine clock signal of the native CPU------
* When porting ispVME to a native CPU environment, the speed of CPU or 
* the system clock that drives the CPU is usually known. 
* The speed or the time it takes for the native CPU to execute one for loop 
* then can be calculated as follows:
*       The for loop usually is compiled into the ASSEMBLY code as shown below:
*       LOOP: DEC RA;
*             JNZ LOOP;
*       If each line of assembly code needs 4 machine cycles to execute, 
*       the total number of machine cycles to execute the loop is 2 x 4 = 8.
*       Usually system clock = machine clock (the internal CPU clock). 
*       Note: Some CPU has a clock multiplier to double the system clock for 
*		the machine clock.
*
*       Let the machine clock frequency of the CPU be F, or 1 machine cycle = 1/F.
*       The time it takes to execute one for loop = (1/F ) x 8.
*       Or one micro-second = F(MHz)/8;
*
* Example: The CPU internal clock is set to 100Mhz, then one micro-second = 100/8 = 12
*
* The C code shown below can be used to create the milli-second accuracy. 
* Users only need to enter the speed of the CPU.
*
**********************************************************************************/
void ispVMDelay( uint16_t a_usTimeDelay )
{
	flushPort();

	uint32_t delay = a_usTimeDelay;

	if (gJTAGMode == JTAG_NONE)
		return;

	if ( delay & 0x8000 ) /*Test for unit*/
	{
		delay &= ~0x8000; /*unit in milliseconds*/
		delay = delay*1000; /*convert to microseconds*/
	}

	if (gParanoidSafety)		// USB JTAG will typically always introduce enough delay
	{
		if ( delay <= 0 ) {
			delay = 1000; /*delay is 1 millisecond minimum*/
		}
	}

	if (delay)
	{
//		Sleep(a_usTimeDelay);
		usleep(delay);
	}
}

/*********************************************************************************
*
* calibration
*
* It is important to confirm if the delay function is indeed providing 
* the accuracy required. Also one other important parameter needed 
* checking is the clock frequency. 
* Calibration will help to determine the system clock frequency 
* and the loop_per_micro value for one micro-second delay of the target 
* specific hardware.
*              
**********************************************************************************/
void calibration(void)
{
	/*Apply 2 pulses to TCK.*/
	writePort( g_ucPinTCK, 0x00 );
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );

	/*Delay for 1 millisecond. Pass on 1000 or 0x8001 both = 1ms delay.*/
	ispVMDelay(0x8001);

	/*Apply 2 pulses to TCK*/
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );
	writePort( g_ucPinTCK, 0x01 );
	writePort( g_ucPinTCK, 0x00 );
}

/*************************************************************
*                                                            *
* ENABLEHARDWARE                                             *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     This function is called to enable the hardware.        *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/

void EnableHardware(void)
{
	last_bitMode = 0;
	LongestRun = 0;
	NumberClockRuns = 0;
	TotalClocks = 0;
	USBTransactions = 0;
	ClockUpdateFlush = 0;
	last_siIspPins = 0;
	last_siUSBPins = 0;
	g_siIspPins = 0;
	last_secs = 0;
	last_clocks = 0;
	last_ms = 0;

	if (gJTAGMode == JTAG_FTDI_BITBANG_CBUS_READ)
	{
		if (gParanoidSafety)
			FT_CHECK(ftdi_set_baudrate(ftdi, ASYNC_BB_RATE_SLOW));
		else
			FT_CHECK(ftdi_set_baudrate(ftdi, ASYNC_BB_RATE));

		FT_CHECK(ftdi_set_bitmode(ftdi, ABBM_IO, BITMODE_BITBANG));
		FT_CHECK(written = ftdi_write_data(ftdi, &HL[1], 1));	// initialize clock to LOW
		if (written != 1)
			printf("%s(%d): short FT_Write result (%d vs %d)?\n", __FUNCTION__, __LINE__, (int32_t)written, 1);
	}

	if (gJTAGMode != JTAG_NONE)
	{
		FT_CHECK(ftdi_set_bitmode(ftdi, 0, BITMODE_CBUS));
		last_bitMode = BITMODE_CBUS;
	}
}

/*************************************************************
*                                                            *
* DISABLEHARDWARE                                            *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     This function is called to disable the hardware.       *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/

void DisableHardware(void)
{
//	ispVMStateMachine( RESET );
	ispVMDelay( 1 );
	if (gJTAGMode != JTAG_NONE)
	{
		FT_CHECK(ftdi_set_bitmode(ftdi, ABBM_IO, BITMODE_BITBANG));
		FT_CHECK(written = ftdi_write_data(ftdi, &HL[1], 1));
		if (written != 1)
			printf("%s(%d): short ftdi_write_data result (%d vs %d)?\n", __FUNCTION__, __LINE__, (int32_t)written, 1);
		FT_CHECK(ftdi_set_bitmode(ftdi, 0, BITMODE_CBUS));	// leave all as inputs
	}
}
/*************************************************************
*                                                            *
* CHECK_CABLE_POWER                                          *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     Returns 0 if passing, or negative number if failure.   *
*                                                            *
* DESCRIPTION:                                               *
*     This function checks to see if the cable has power.    *
*     This function is tailored to the Windows platform and  *
*     must be overwritten if user is targeting another       *
*     platform.  It is not required to be overwritten.       *
*                                                            *
*************************************************************/

void ListDevices(void)
{
	static const char *FTDI_Type[] =
	{
		"FTDI-BM   ",
		"FTDI-AM   ",
		"FTDI-2232C",
		"FTDI-232R ",
		"FTDI-2232H",
		"FTDI-4232H",
		"FTDI-232H ",
		"FTDI-230X "
	};

	int32_t	num_devices = 0;
	struct ftdi_device_list	*devlist = NULL;
	struct ftdi_device_list	*dev_info = NULL;
	int32_t i;

	if ((ftdi = ftdi_new()) == 0)
	{
		printf("\nError: FATAL: ftdi_new failed.\n");
		return;
	}

	printf("Available FTDI devices:  (with * next to possible FleaFPGA devices)\n");
	FT_CHECK(num_devices = ftdi_usb_find_all(ftdi, &devlist, 0, 0));

	if (ftdi_status < 0)
	{
		printf("\nError: Can't list devices (error %d:%s)\n", ftdi_status, ftdi_get_error_string(ftdi));
		return;
	}

	if (num_devices != 0)
	{
		for (i = 0, dev_info = devlist; i < num_devices && dev_info != NULL; i++, dev_info = dev_info->next)
		{
			char manufac[128] = { 0 };
			char desc[128] = { 0 };
			char serial[128] = { 0 };

			int rc = ftdi_usb_get_strings(ftdi, dev_info->dev, manufac, sizeof (manufac), desc, sizeof(desc), serial, sizeof (serial));

			if (rc < 0 && (rc != -7 && rc != -8 && rc != -9))
			{
				printf("Error %d querying device: %s\n", rc, ftdi_get_error_string(ftdi));

				if (rc == -4)
					printf("\nDo you have permissions (perhaps you need \"sudo\")?\n");

				break;
			}

			if (ftdi_usb_open_dev(ftdi, dev_info->dev) == 0)
			{
				printf(" %c%2d %s - %-40.40s #%s\n", (ftdi->type == TYPE_230X) ? '*' : ' ', i,
					ftdi->type <= TYPE_230X ? FTDI_Type[ftdi->type] : "FTDI-??   ", 
					desc[0] ? desc : "(none)", 
					serial[0] ? serial : "(none)"); 
					
				FT_CHECK(ftdi_usb_close(ftdi)); 
			}
			else
			{
				printf(" %c%2d %s - %s %s #%s\n", ' ', i,
					"(can't open)", 
					manufac[0] ? manufac : "(none)",
					desc[0] ? desc : "(none)", 
					serial[0] ? serial : "(none)"); 
			}
		}
	}
	else
	{
		printf(" (no devices found)\n");
	}

	printf("\n");
	if (devlist)
		ftdi_list_free(&devlist);
	if (ftdi)
	{
		ftdi_free(ftdi);
		ftdi = NULL;
	}
}

void closeJTAGDevice(void)
{
	static uint8_t def_handshake = 0x04 | 0x01;
	if (ftdi != 0)
	{
		FT_CHECK(ftdi_set_bitmode(ftdi, def_handshake, BITMODE_BITBANG));	// RTS, TXD = HIGH
		FT_CHECK(ftdi_set_bitmode(ftdi, 0, BITMODE_CBUS));					// leave all CBUS as inputs
		ftdi_usb_close(ftdi);
		ftdi_free(ftdi);
	}
	ftdi = 0;
}

int32_t openJTAGDevice(void)
{
	uint32_t i = 0;

	if (gJTAGMode == JTAG_NONE)
		return 0;

	if ((ftdi = ftdi_new()) == 0)
	{
		printf("\nError: FATAL: ftdi_new failed.\n");
		return -1;
	}

	printf("Searching for FleaFPGA...");
	fflush(stdout);
	for (i = 0; i < (gSpecifiedDevice ?  1 : sizeof (FleaFPGA_Desc) / sizeof (FleaFPGA_Desc[0])); i++)
	{
		if (FleaFPGA_Desc[i][0] == '#')
		{
			ftdi_status = ftdi_usb_open_desc(ftdi, 0x0403, 0x6015, NULL, &FleaFPGA_Desc[i][1]);
		}
		else
		{
			ftdi_status = ftdi_usb_open_desc(ftdi, 0x0403, 0x6015, FleaFPGA_Desc[i], NULL);
		}
			
		if (ftdi_status == FT_OK)
		{
			break;
		}
	}

	if (ftdi_status != FT_OK)
	{
		printf("not found (see -m or -l option, open returned %d: %s).\n", (int32_t)ftdi_status, ftdi_get_error_string(ftdi));

		if (ftdi_status == -4)
			printf("\nDo you have permissions (perhaps you need \"sudo\")?\n");

		return -1;
	}
	
	{
		unsigned char serial_num[16] = "????????";
		struct libusb_device_descriptor usb_desc = { 0 };
		
		if (libusb_get_device_descriptor(libusb_get_device(ftdi->usb_dev), &usb_desc) == 0)
		{
			if (usb_desc.iSerialNumber > 0)
			{
				libusb_get_string_descriptor_ascii(ftdi->usb_dev, usb_desc.iSerialNumber, serial_num, 9);
			}
		}
		printf("found %s #%s\n", FleaFPGA_Desc[i], serial_num);
		fflush(stdout);
	}
	atexit(closeJTAGDevice);

	switch (gJTAGMode)
	{
	case JTAG_FTDI_CBUS:
		CBUS_IO = (cbusTDI|cbusTMS|cbusTCK)<<4;
		ABBM_IO = 0;
		break;
	case JTAG_FTDI_BITBANG_CBUS_READ:
		CBUS_IO = 0;
		ABBM_IO = abbmTCK|abbmTDI|abbmTMS;
		break;
	default:
		break;
	}

	return 0;
}

int BenchmarkUSB(void)
{
	int tps = 0;
	uint32_t bt = 0;

	if (ftdi)
	{
		bt = getMilliseconds(0);
		while (bt == getMilliseconds(0))
			;
		bt = getMilliseconds(0);
		while (getMilliseconds(bt) < 25)
		{
			tps++;
			FT_CHECK(ftdi_set_bitmode(ftdi, 0x00, BITMODE_CBUS));
			FT_CHECK(ftdi_read_pins(ftdi, &in_byte));
		}
	}
	return tps * 40 * 2;
}

// initialize FT230X EEPROM for FleaFPGA use
int32_t InitFleaFPGA()
{
//	uint32_t i = 0;
	printf("Not supported using libftdi1.2 (yet)...");
	fflush(stdout);

	return 0;
#if 0
	printf("Searching for FT230X to initialize...");
	fflush(stdout);
//	if (FleaFPGA_Desc[i][0] == '#')
//		ftdi_status = FT_OpenEx(&FleaFPGA_Desc[i][1], FT_OPEN_BY_SERIAL_NUMBER, &ftdi);
//	else
//		ftdi_status = FT_OpenEx("FT230X Basic UART", FT_OPEN_BY_DESCRIPTION, &ftdi);

	if (ftdi_status != FT_OK || ftdi == 0)
	{
		printf("not found (see -m or -l option).\n");
		exit(5);
	}

	printf("found %s\n", FleaFPGA_Desc[i]);
	fflush(stdout);

	atexit(closeJTAGDevice);

//	if (FT_ResetPort(ftdi) != FT_OK)
//	{
//		closeJTAGDevice();
//		printf("FTDI ResetPort failed: Check cable, connection and FTDI driver.\n");
//		return -1;
//	}

	if (gJTAGMode == JTAG_NONE)
		return 0;

	printf("Reading FT230X EEPROM and configuration data...\n");

	char Manufacturer[64] = { 0 };
	char ManufacturerId[64] = { 0 };
	char Description[64] = { 0 };
	char SerialNumber[64] = { 0 };

//	FT_EEPROM_X_SERIES	ft230x_eeprom;
//	memset(&ft230x_eeprom, 0, sizeof(ft230x_eeprom));

//	ft230x_eeprom.common.deviceType = FT_DEVICE_X_SERIES;

//	FT_CHECK(FT_EEPROM_Read(ftdi, &ft230x_eeprom, sizeof(ft230x_eeprom), Manufacturer, ManufacturerId, Description, SerialNumber));

//	if (ftdi_status == FT_OK)
//	{
//		printf("Manufacturer: \"%s\"\n", Manufacturer);
//		printf("Description : \"%s\"\n", Description);
//		printf("Serial #%s   Manufacturer ID: %s\n", SerialNumber, ManufacturerId); 

//		uint8_t *rawdata = (uint8_t *)&ft230x_eeprom;
//		for (i = 0; i < sizeof (ft230x_eeprom); i+=16)
//		{
//			printf("%02x:", i);
//			int b;
//			for (b = 0; b < 16; b++)
//			{
//				printf(" %02x", rawdata[i+b]);
//			}
//			printf(" ");
//			for (b = 0; b < 16; b++)
//			{
//				printf("%c", rawdata[i+b] >= ' ' && rawdata[i+b] <= '~' ? rawdata[i+b] : '.');
//			}
//			printf("\n");
//		}
//		printf("\n");
//	}

	printf("Programming FT230X EEPROM with FleaFPGA settings...\n");

//	strcpy(Manufacturer, "FleaSystems");
//	strcpy(Description, "FleaFPGA v2.5");

//	ft230x_eeprom.Cbus0 = FT_X_SERIES_CBUS_IOMODE;
//	ft230x_eeprom.Cbus1 = FT_X_SERIES_CBUS_IOMODE;
//	ft230x_eeprom.Cbus2 = FT_X_SERIES_CBUS_IOMODE;
//	ft230x_eeprom.Cbus3 = FT_X_SERIES_CBUS_IOMODE;
//	ft230x_eeprom.common.MaxPower = 500;

//	FT_CHECK(FT_EEPROM_Program(ftdi, &ft230x_eeprom, sizeof(ft230x_eeprom), Manufacturer, ManufacturerId, Description, SerialNumber));

//	if (ftdi_status == FT_OK)
//	{
//		printf("Completed, cycling USB port...\n");
//
//		FT_CyclePort(ftdi);
//		Sleep(10000);
//	}

//	FT_Close(ftdi);
//	ftdi = 0;

//	return 0;
#endif
}
