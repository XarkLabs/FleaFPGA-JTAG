/**************************************************************
*
* Lattice Semiconductor Corp. Copyright 2008
* 
* ispVME Embedded allows programming of Lattice's suite of FPGA
* devices on embedded systems through the JTAG port.  The software
* is distributed in source code form and is open to re - distribution
* and modification where applicable.
*
* ispVME Embedded C Source comprised with 3 modules:
* ispvm_ui.c is the module provides input and output support.
* ivm_core.c is the module interpret the VME file(s).
* hardware.c is the module access the JTAG port of the device(s).                 
*
* The optional module cable.c is for supporting Lattice's parallel 
* port ispDOWNLOAD cable on DOS and Windows 95/98 O/S. It can be 
* requested from Lattice's ispVMSupport.
*
***************************************************************/


/**************************************************************
* 
* Revision History of ispvm_ui.c
* 
* 3/6/07 ht Added functions vme_out_char(),vme_out_hex(), 
*           vme_out_string() to provide output resources.
*           Consolidate all printf() calls into the added output 
*           functions.	
*
* 09/11/07 NN Added Global variables initialization
* 09/24/07 NN Added a switch allowing users to do calibration.
* Calibration will help to determine the system clock frequency
* and the count value for one micro-second delay of the target 
* specific hardware.
* Removed Delay Percent support
* 11/15/07  NN moved the checking of the File CRC to the end of processing
* 08/28/08 NN Added Calculate checksum support.
***************************************************************/


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#if defined(WIN32)
#include "windows.h"
#include <sys/stat.h>
#include <signal.h>

//#define stat(path, buffer) _stat(path, buffer)
#elif defined(__linux__)
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>	

#define _stat stat

#endif

#include "vmopcode.h"
#include "hardware.h"

/***************************************************************
*
* File pointer to the VME file.
*
***************************************************************/

FILE * g_pVMEFile = NULL;

/***************************************************************
*
* Functions declared in this ispvm_ui.c module
*
***************************************************************/
unsigned char GetByte(void);
void vme_out_char(unsigned char charOut);
void vme_out_hex(unsigned char hexOut);
void vme_out_string(char *stringOut);
void ispVMMemManager( signed char cTarget, unsigned short usSize );
void ispVMFreeMem(void);
void error_handler( short a_siRetCode, char * pszMessage );
signed char ispVM( const char * a_pszFilename );
void writeConfig(void);

/***************************************************************
*
* Global variables.
*
***************************************************************/
unsigned short g_usPreviousSize = 0;
unsigned short g_usExpectedCRC = 0;
char szCommandLineArg[ 4096 ]      = { 0 };
char cmd_str[4096] = { 0 };
char tmp_str[4096] = { 0 };
char op_str[4096] = { 0 };
char orig_op_str[4096] = { 0 };
char vme_file[4096] = { 0 };
char xcf_file[4096] = { 0 };
char out_file[4096] = { 0 };
char fpga_file[4096] = { 0 };
char ddtcmd_path[4096] = { 0 };
int gRetCode = 0;
FILE *out_fp = NULL;

/***************************************************************
*
* External variables and functions declared in ivm_core.c module.
*
***************************************************************/
extern uint32_t InitFleaFPGA();	// initialize FT230X EEPROM for FleaFPGA use
extern signed char ispVMCode();
extern void ispVMCalculateCRC32( unsigned char a_ucData );
extern void ispVMStart();
extern void ispVMEnd();
extern unsigned short g_usCalculatedCRC;
extern unsigned short g_usDataType;
extern unsigned char * g_pucOutMaskData,
					 * g_pucInData,
					 * g_pucOutData,
					 * g_pucHIRData,
					 * g_pucTIRData,
					 * g_pucHDRData,
					 * g_pucTDRData,
					 * g_pucOutDMaskData,
					 * g_pucIntelBuffer;
extern unsigned char * g_pucHeapMemory;
extern unsigned short g_iHeapCounter;
extern unsigned short g_iHEAPSize;
extern unsigned short g_usIntelDataIndex;
extern unsigned short g_usIntelBufferSize;
extern LVDSPair * g_pLVDSList;
//08/28/08 NN Added Calculate checksum support.
extern uint8_t g_usCalculateChecksum;
extern unsigned int g_usChecksum;
extern unsigned int g_uiChecksumIndex;
/***************************************************************
*
* External variables and functions declared in hardware.c module.
*
***************************************************************/
extern unsigned short g_usCpu_Frequency;

/***************************************************************
*
* Supported VME versions.
*
***************************************************************/

const char * const g_szSupportedVersions[] = { "__VME2.0", "__VME3.0", "____12.0", "____12.1", 0 };

void SetTitleBar(const char *str)
{
#if defined(WIN32)
	SetConsoleTitle(str);
#else
	(void)str;
#endif
}

void SetGreenText(void)
{
#if defined(WIN32)
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN | FOREGROUND_INTENSITY);
#endif
}

void SetPurpleText(void)
{
#if defined(WIN32)
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_BLUE);
#endif
}

void SetRedText(void)
{
#if defined(WIN32)
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_INTENSITY);
#endif
}

void SetYellowText(void)
{
#if defined(WIN32)
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
#endif
}

void SetCyanText(void)
{
#if defined(WIN32)
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
#endif
}

void SetBoldText(void)
{
#if defined(WIN32)
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY );
#endif
}

void SetNormalText(void)
{
#if defined(WIN32)
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE );
#endif
}



/***************************************************************
*
* GetByte
*
* Returns a byte to the caller. The returned byte depends on the
* g_usDataType register. If the HEAP_IN bit is set, then the byte
* is returned from the HEAP. If the LHEAP_IN bit is set, then
* the byte is returned from the intelligent buffer. Otherwise,
* the byte is returned directly from the VME file.
*
***************************************************************/

unsigned char GetByte()
{
	unsigned char ucData = 0;
	
	if ( g_usDataType & HEAP_IN )
	{

		/***************************************************************
		*
		* Get data from repeat buffer.
		*
		***************************************************************/

		if ( g_iHeapCounter > g_iHEAPSize )
		{

			/***************************************************************
			*
			* Data over-run.
			*
			***************************************************************/

			return 0xFF;
		}

		ucData = g_pucHeapMemory[ g_iHeapCounter++ ];
	}
	else if ( g_usDataType & LHEAP_IN )
	{

		/***************************************************************
		*
		* Get data from intel buffer.
		*
		***************************************************************/

		if ( g_usIntelDataIndex >= g_usIntelBufferSize ) {

			/***************************************************************
			*
			* Data over-run.
			*
			***************************************************************/

			return 0xFF;
		}

		ucData = g_pucIntelBuffer[ g_usIntelDataIndex++ ];
	}
	else
	{

		/***************************************************************
		*
		* Get data from file.
		*
		***************************************************************/

		ucData = (unsigned char)fgetc( g_pVMEFile );

		if ( feof( g_pVMEFile ) ) {

			/***************************************************************
			*
			* Reached EOF.
			*
			***************************************************************/

			return 0xFF;
		}
		/***************************************************************
		*
		* Calculate the 32-bit CRC if the expected CRC exist.
		*
		***************************************************************/
		if( g_usExpectedCRC != 0)
		{
			ispVMCalculateCRC32(ucData);
		}
	}
	
	return ( ucData );
}

/***************************************************************
*
* vme_out_char
*
* Send a character out to the output resource if available. 
* The monitor is the default output resource. 
*
*
***************************************************************/
void vme_out_char(unsigned char charOut)
{
	CloseLine();
	printf("%c",charOut);
	if (charOut == '\n' || charOut == '\r')
		OpenLine(1);
}
/***************************************************************
*
* vme_out_hex
*
* Send a character out as in hex format to the output resource 
* if available. The monitor is the default output resource. 
*
*
***************************************************************/
void vme_out_hex(unsigned char hexOut)
{
	if (!out_fp)
	{
		out_fp = fopen(out_file, "w");
		CloseLine();
		if (out_fp)
		{
			printf("Data read will be written to file:\n  \"");
			SetBoldText();
			printf("%s", out_file);
			SetNormalText();
			printf("\"\n\n");
		}
		else
		{
			SetRedText();
			printf("\nError: Unable to write to file:\n  \"%s\"\n", out_file);
			SetNormalText();
			gRetCode = -1;
			exit(gRetCode);
		}
		OpenLine(1);
	}
	
	fputc(hexOut, out_fp);
}
/***************************************************************
*
* vme_out_string
*
* Send a text string out to the output resource if available. 
* The monitor is the default output resource. 
*
*
***************************************************************/
void vme_out_string(char *stringOut)
{
	if(stringOut && strlen(stringOut))
	{
		CloseLine();
		printf("%s",stringOut);
		if (stringOut[strlen(stringOut)-1] == '\n' || stringOut[strlen(stringOut)-1] == '\r')
			OpenLine(1);
	}
}
/***************************************************************
*
* ispVMMemManager
*
* Allocate memory based on cTarget. The memory size is specified
* by usSize.
*
***************************************************************/

void ispVMMemManager( signed char cTarget, unsigned short usSize )
{
	switch ( cTarget )
	{
	case XTDI:
	case TDI:  
		if ( g_pucInData != NULL )
		{
			if ( g_usPreviousSize == usSize ) {/*memory exist*/
				break;
			}
			else {
				free( g_pucInData );
				g_pucInData = NULL;
			}
		}
		g_pucInData = ( unsigned char * ) malloc( usSize / 8 + 2 );
		g_usPreviousSize = usSize;
	case XTDO:
	case TDO:
		if ( g_pucOutData!= NULL )
		{ 
			if ( g_usPreviousSize == usSize ) { /*already exist*/
				break;
			}
			else {
				free( g_pucOutData );
				g_pucOutData = NULL;
			}
		}
		g_pucOutData = ( unsigned char * ) malloc( usSize / 8 + 2 );
		g_usPreviousSize = usSize;
		break;
	case MASK:
		if ( g_pucOutMaskData != NULL )
		{
			if ( g_usPreviousSize == usSize ) {/*already allocated*/
				break;
			}
			else {
				free( g_pucOutMaskData ); 
				g_pucOutMaskData = NULL;
			}
		}
		g_pucOutMaskData = ( unsigned char * ) malloc( usSize / 8 + 2 );
		g_usPreviousSize = usSize;
		break;
	case HIR:
		if ( g_pucHIRData != NULL )
		{
			free( g_pucHIRData );
			g_pucHIRData = NULL;
		}
		g_pucHIRData = ( unsigned char * ) malloc( usSize / 8 + 2 );
		break;
	case TIR:
		if ( g_pucTIRData != NULL )
		{
			free( g_pucTIRData );
			g_pucTIRData = NULL;
		}
		g_pucTIRData = ( unsigned char * ) malloc( usSize / 8 + 2 );
		break;
	case HDR:
		if ( g_pucHDRData != NULL )
		{
			free( g_pucHDRData );
			g_pucHDRData = NULL;
		}
		g_pucHDRData = ( unsigned char * ) malloc( usSize / 8 + 2 );
		break;
	case TDR:
		if ( g_pucTDRData != NULL )
		{
			free( g_pucTDRData );
			g_pucTDRData = NULL;
		}
		g_pucTDRData = ( unsigned char * ) malloc( usSize / 8 + 2 );
		break;
	case HEAP:
		if ( g_pucHeapMemory != NULL )
		{
			free( g_pucHeapMemory );
			g_pucHeapMemory = NULL;
		}
		g_pucHeapMemory = ( unsigned char * ) malloc( usSize + 2 );
		break;
	case DMASK: 
		if ( g_pucOutDMaskData != NULL )
		{
			if ( g_usPreviousSize == usSize ) { /*already allocated*/
				break;
			}
			else {
				free( g_pucOutDMaskData ); 
				g_pucOutDMaskData = NULL;
			}
		}
		g_pucOutDMaskData = ( unsigned char * ) malloc( usSize / 8 + 2 );
		g_usPreviousSize = usSize;
		break;
	case LHEAP:
		if ( g_pucIntelBuffer != NULL )
		{
			free( g_pucIntelBuffer );
			g_pucIntelBuffer = NULL;
		}
		g_pucIntelBuffer = ( unsigned char * ) malloc( usSize + 2 );
		break;
	case LVDS:
		if ( g_pLVDSList != NULL )
		{
			free( g_pLVDSList );
			g_pLVDSList = NULL;
		}
		g_pLVDSList = ( LVDSPair * ) calloc( usSize, sizeof( LVDSPair ) );
		break;
	default:
		return;
	}
}

/***************************************************************
*
* ispVMFreeMem
*
* Free memory that were dynamically allocated.
*
***************************************************************/

void ispVMFreeMem()
{
	if ( g_pucHeapMemory != NULL )
	{
		free( g_pucHeapMemory ); 
		g_pucHeapMemory = NULL;
	}

	if ( g_pucOutMaskData != NULL )
	{
		free( g_pucOutMaskData );
		g_pucOutMaskData = NULL;
	}
	
	if ( g_pucInData != NULL )
	{
		free( g_pucInData );
		g_pucInData = NULL;
	}
	
	if ( g_pucOutData != NULL )
	{
		free( g_pucOutData );
		g_pucOutData = NULL;
	}
	
	if ( g_pucHIRData != NULL )
	{
		free( g_pucHIRData );
		g_pucHIRData = NULL;
	}
	
	if ( g_pucTIRData != NULL )
	{
		free( g_pucTIRData );
		g_pucTIRData = NULL;
	}
	
	if ( g_pucHDRData != NULL )
	{
		free( g_pucHDRData );
		g_pucHDRData = NULL;
	}
	
	if ( g_pucTDRData != NULL )
	{
		free( g_pucTDRData );
		g_pucTDRData = NULL;
	}
	
	if ( g_pucOutDMaskData != NULL )
	{
		free( g_pucOutDMaskData );
		g_pucOutDMaskData = NULL;
	}
	
	if ( g_pucIntelBuffer != NULL )
	{
		free( g_pucIntelBuffer );
		g_pucIntelBuffer = NULL;
	}

	if ( g_pLVDSList != NULL )
	{
		free( g_pLVDSList );
		g_pLVDSList = NULL;
	}
} 

/***************************************************************
*
* error_handler
*
* Reports the error message.
*
***************************************************************/

void error_handler( short a_siRetCode, char * pszMessage )
{
	static const char * pszErrorMessage[] =
	{
		"pass",
		"verification fail",
		"can't find the file",
		"wrong file type",
		"file error",
		"option error",
		"CRC verification error"
	};

	strcpy( pszMessage, pszErrorMessage[ -a_siRetCode ] );
}
/***************************************************************
*
* ispVM
*
* The entry point of the ispVM embedded. If the version and CRC
* are verified, then the VME will be processed.
*
***************************************************************/

signed char ispVM( const char * a_pszFilename )
{
	char szFileVersion[ 9 ]      = { 0 };
	signed char cRetCode         = 0;
	signed char cIndex           = 0;
	signed char cVersionIndex    = 0;
	unsigned char ucReadByte     = 0;
	
	/***************************************************************
	*
	* Global variables initialization.
	*
	* 09/11/07 NN Added
	***************************************************************/
	g_pucHeapMemory		= NULL;
	g_iHeapCounter		= 0;
	g_iHEAPSize			= 0;
	g_usIntelDataIndex	= 0;
	g_usIntelBufferSize	= 0;
	g_usPreviousSize     = 0;

	/***************************************************************
	*
	* Open a file pointer to the VME file.
	*
	***************************************************************/

	if ( ( g_pVMEFile = fopen( a_pszFilename, "rb" ) ) == NULL )
	{
		return VME_FILE_READ_FAILURE;
	}
	g_usCalculatedCRC = 0;
	g_usExpectedCRC   = 0;
	ucReadByte = GetByte();
	switch( ucReadByte ) {
	case FILE_CRC:

		/***************************************************************
		*
		* Read and store the expected CRC to do the comparison at the end.  
		* Only versions 3.0 and higher support CRC protection.
		*
		***************************************************************/

		g_usExpectedCRC = (unsigned char ) fgetc( g_pVMEFile );
		g_usExpectedCRC <<= 8;
		g_usExpectedCRC |= fgetc( g_pVMEFile );
		

		/***************************************************************
		*
		* Read and store the version of the VME file.
		*
		***************************************************************/

		for ( cIndex = 0; cIndex < 8; cIndex++ )
		{
			szFileVersion[ cIndex ] = GetByte();
		}

		break;
	default:

		/***************************************************************
		*
		* Read and store the version of the VME file.  Must be version 2.0.
		*
		***************************************************************/

		szFileVersion[ 0 ] = ( signed char ) ucReadByte;
		for ( cIndex = 1; cIndex < 8; cIndex++ )
		{
			szFileVersion[ cIndex ] = GetByte();
		}

		break;
	}

	/***************************************************************
	*
	* Compare the VME file version against the supported version.
	*
	***************************************************************/

	for ( cVersionIndex = 0; g_szSupportedVersions[ cVersionIndex ] != 0; cVersionIndex++ )
	{
		for ( cIndex = 0; cIndex < 8; cIndex++ ) {
			if ( szFileVersion[ cIndex ] != g_szSupportedVersions[ cVersionIndex ][ cIndex ] )
			{
				cRetCode = VME_VERSION_FAILURE;
				break;
			}	
			cRetCode = 0;
		}

		if ( cRetCode == 0 )
		{

			/***************************************************************
			*
			* Found matching version, break.
			*
			***************************************************************/

			break;
		}
	}

	if ( cRetCode < 0 )
	{

		/***************************************************************
		*
		* VME file version failed to match the supported versions.
		*
		***************************************************************/

		fclose( g_pVMEFile );
		g_pVMEFile = NULL;
		return VME_VERSION_FAILURE;
	}

	/***************************************************************
	*
	* Enable the JTAG port to communicate with the device.
	* Set the JTAG state machine to the Test-Logic/Reset State.
	*
	***************************************************************/

	EnableHardware();
	
	ispVMStart();

	/***************************************************************
	*
	* Process the VME file.
	*
	***************************************************************/

	cRetCode = ispVMCode();

	/***************************************************************
	*
	* Set the JTAG State Machine to Test-Logic/Reset state then disable
	* the communication with the JTAG port.
	*
	***************************************************************/

	ispVMEnd();

	DisableHardware();
				   
	fclose( g_pVMEFile );
	g_pVMEFile = NULL;


	ispVMFreeMem();

	/***************************************************************
	*
	* Compare the expected CRC versus the calculated CRC.
	*
	***************************************************************/

	if ( cRetCode == 0 && g_usExpectedCRC != 0 && ( g_usExpectedCRC != g_usCalculatedCRC ) )
	{
		printf( "Expected CRC:   0x%.4X\n", g_usExpectedCRC );
		printf( "Calculated CRC: 0x%.4X\n", g_usCalculatedCRC );
		return VME_CRC_FAILURE;
	}
	
	return ( cRetCode );
}

void quitmsg(void)
{
	SetNormalText();

#if defined(WIN32)
	if (gRetCode >= 0 && gRetCode != 0xc0de && gRestartPath[0])
	{
		RestartApp();
	}
#endif
	
	if (gRetCode == 0xc0de1)	// quiet exit code for auto
		printf("\n");
	if (gRetCode != 0xc0de)	// quiet exit code for -?
		printf("Done...");
	if (gRetCode < 0)
		printf("\a");	// beep
	fflush(stdout);
#if defined(WIN32)
	if (getenv("PROMPT") == NULL && getenv("SHELL") == NULL)	// clue that not at command prompt
	{
		if (gRetCode < 0)
		{
			printf("\nPress ENTER to continue:");
			fflush(stdout);
			fgets(szCommandLineArg, sizeof(szCommandLineArg), stdin);
		}
		else
		{
			Sleep(3500);
		}
	}
#endif
	if (gRetCode != 0xc0de)	// quiet exit code for -?
		printf("\n");
	fflush(stdout);
}

void showHelp(void)
{
	printf("FleaFPGA-JTAG: JTAG utility for FleaFPGA using Lattice FPGA.\n\n");
	printf("Usage: FleaFPGA-JTAG [-f | -s | -t] [-l] [-v] [-p] [-m <desc>] [-c <path>]\n");
	printf("                     [ <XCF_file | VME_file> ... ]\n");
	printf("\n");
	printf("  -f        = Fast FTDI bit-bang JTAG with CBUS read\n");
	printf("               (RTS=TDI, CB1=TDO, TX=TMS, CTS=TCK)\n");
	printf("  -s        = Prototype board FTDI all CBUS JTAG\n");
	printf("               (CB0=TDI, CB1=TDO, CB2=TMS, CB3=TCK)\n");
	printf("  -t        = Test JTAG (fake JTAG with no actual hardware)\n");
	printf("  -c <path> = Full path to Lattice Diamond Programmer \"ddtcmd.exe\" utility\n");
	printf("               (used for automatic XCF processing to generate VME JTAG files)\n");
	printf("  -l        = List information about all available FTDI devices\n");
	printf("  -m <desc> = Search for FleaFPGA FTDI device description <desc> or #<serial>\n");
	printf("  -p        = Paranoia mode (slower conservative settings for troubleshooting)\n");
	printf("  -v        = Verbose output (show all JTAG output as \">\" and input as \"=\")\n");
	printf("  -w        = Write current options to settings file (new defaults)\n");
	printf("  -q <title>= Quit app where window title contains comma separated strings\n");
	printf("  -?        = This help text\n");
	
	gRetCode = 0xc0de;
	exit(1);
}

#if defined(WIN32)
int FileRequest(const char *title, const char *filterlist, char *outstring, int outlength)
{
	static OPENFILENAME ofn;
	int rc = 0;
	
	memset( &ofn , 0, sizeof( ofn));
	ofn.lStructSize = sizeof ( ofn );
	ofn.hwndOwner = NULL  ;
	ofn.lpstrFile = outstring;
	ofn.nMaxFile = outlength;
	ofn.lpstrFilter = filterlist;
	ofn.nFilterIndex =0;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0 ;
	ofn.lpstrInitialDir=NULL ;
	ofn.Flags = OFN_PATHMUSTEXIST|OFN_FILEMUSTEXIST ;
	ofn.lpstrTitle = title;

	rc = GetOpenFileName(&ofn);

	return rc;
}
#endif

// find & replace (replacement length must be <= find)
void strReplace(char *str, char *find, char *replace)
{
	int findlen = strlen(find);
	int replacelen = strlen(replace);
	char *e = NULL;

	if (replacelen > findlen)
		return;

	while ((e = strstr(str, find)) != NULL)
	{
		strcpy(e + replacelen, e + findlen);
		memcpy(e, replace, replacelen);
	}
}

int processXCF(char *xcf, char *vme)
{
	int rc = 0;
	FILE *xf = NULL;
	char *op = NULL;
	char *fn = NULL;
	char *e  = NULL;
	int ask_loc = 1;
	char *fr = NULL;
	
	memset(tmp_str, 0, sizeof (tmp_str));
	memset(op_str, 0, sizeof (op_str));
	memset(fpga_file, 0, sizeof (fpga_file));
	
	// verify ddtcmd_path set and there is a file at ddtcmd_path
	if (ddtcmd_path[0] && (xf = fopen(ddtcmd_path, "r")) != NULL)
	{
		fclose(xf);
		xf = NULL;
		ask_loc = 0;
	}

	if (ask_loc)
	{
#if defined(WIN32)
		printf("Requesting \"ddtcmd.exe\" location...\n");
		fflush(stdout);
		rc = FileRequest("Select the \"ddtcmd.exe\" utility under Lattice Programmer \"bin\" directory (saved to settings)", "Executable (*.exe)\0*.exe\0All files\0*.*\0", ddtcmd_path, sizeof (ddtcmd_path));
		if (rc)
			writeConfig();
		else
#endif
		{
			SetRedText();
			printf("\nError: Path to Lattice \"ddtcmd.exe\" required to process XCF into VME files.\n");
			printf("       Please supply it with the -c option.\n");
			SetNormalText();
			gRetCode = -1;
			exit(gRetCode);
		}
	}
	
	if ((xf = fopen(xcf, "rt")) != NULL)
	{
		for (;;)
		{
			tmp_str[0] = '\0';
			fr = fgets(tmp_str, sizeof (tmp_str) -1, xf);

			if (fr == NULL || feof(xf))
				break;
			
			if (!op && (op = strstr(tmp_str, "<Operation>")))
			{
				strcpy(op_str, op + strlen("<Operation>"));
				op = op_str;
				if ((e = strchr(op, '<')) != NULL)
					*e = '\0';
				strncpy(orig_op_str, op_str, 40);
			}
			else if (!fn && (fn = strstr(tmp_str, "<File>")))
			{
				strcpy(fpga_file, fn + strlen("<File>"));
				fn = fpga_file;
				if ((e = strchr(fn, '<')) != NULL)
					*e = '\0';
			}
		}
		
		fclose(xf);
		
		if (op)
		{
			while ((e = strchr(op, ' ')) != NULL)
				*e = '_';
			while ((e = strchr(op, ',')) != NULL)
				*e = '_';
			while ((e = strchr(op, '-')) != NULL)
				*e = '_';

			strReplace(op, "Erase", "Era");
			strReplace(op, "Program", "Prgm");
			strReplace(op, "Verify", "Vrfy");
			strReplace(op, "Control", "Ctrl");
			strReplace(op, "Option", "Opt");
			strReplace(op, "Display", "Disp");
			strReplace(op, "Register", "Reg");
			strReplace(op, "Calculate", "Calc");
			strReplace(op, "Checksum", "Chsm");
			strReplace(op, "Register", "Reg");
		}
		
		strcpy(vme, xcf);
		if ((e = strrchr(vme, '.')))
			*e = '\0';
		if (op)
		{
			strcat(vme, "-");
			strcat(vme, op);
		}
		strcat(vme, ".vme");
		
		sprintf(cmd_str, "%s -oft -fullvme -if \"%s\" -nocompress -noheader -of \"%s\"", 
			ddtcmd_path,
			xcf,
			vme);
			
		printf("Processing XCF file:\n  \"");
		SetBoldText();
		printf("%s", xcf);
		SetNormalText();
		printf("\"\n\n");
		
		fflush(stdout);

		SetCyanText();
		rc = system(cmd_str);
		SetNormalText();
	}
	else
	{
		rc = -1;
	}
	
	printf("\n");

	return rc;
}

char gConfigLoaded;

void writeConfig(void)
{
	char config_file[4096];
	const char *userprofile = NULL;
	FILE *cf;

#if defined(WIN32)	
	userprofile = getenv("LOCALAPPDATA");
	if (!userprofile)
		userprofile = getenv("APPDATA");
	if (!userprofile)
		userprofile = "./";
	
	sprintf(config_file, "%s\\FleaFPGA", userprofile);
	CreateDirectory(config_file, NULL);
	sprintf(config_file, "%s\\FleaFPGA\\FleaFPGA-JTAG-settings.cfg", userprofile);
#else
	userprofile = getenv("HOME");
	if (!userprofile)
		userprofile = "./";

	sprintf(config_file, "%s/.FleaFPGA-JTAG-settings.cfg", userprofile);
#endif
	
	if ((cf = fopen(config_file, "wt")) != NULL)
	{
		fprintf(cf, "# FleaFPGA-JTAG default settings file\n");
		fprintf(cf, "# This will be read upon start up and can contain default command line options.\n");
		fprintf(cf, "# One per line starting with hyphen and without using quotes.\n");
		fprintf(cf, "# Run \"FleaFPGA-JTAG -?\" for more info on options.\n");
		fprintf(cf, "# Examples:\n");
		fprintf(cf, "# -v\n");
		fprintf(cf, "# -a\n");
		fprintf(cf, "# -p\n");
		fprintf(cf, "# -t\n");
		fprintf(cf, "# -s\n");
		fprintf(cf, "# -f\n");
		fprintf(cf, "# -c /usr/local/lscc/bin/ddtcmd\n");
		fprintf(cf, "# -m %s\n", FleaFPGA_Default_Desc);
		fprintf(cf, "#\n");

		if (ddtcmd_path[0])
			fprintf(cf, "-c %s\n", ddtcmd_path);
		if (gConfigLoaded && gSpecifiedDevice)
			fprintf(cf, "-m %s\n", FleaFPGA_Default_Desc);
		if (gConfigLoaded && gVerbose)
			fprintf(cf, "-v\n");
		if (gConfigLoaded && gAutomatic)
			fprintf(cf, "-a\n");
		if (gConfigLoaded && gParanoidSafety)
			fprintf(cf, "-p\n");
		if (gConfigLoaded && gKillTitleStrs[0])
			fprintf(cf, "-q %s\n", gKillTitleStrs);
		if (gConfigLoaded && gRestartPath[0])
			fprintf(cf, "-r %s\n", gRestartPath);
		if (gConfigLoaded)
		{
			switch (gJTAGMode)
			{
				case JTAG_NONE:
					fprintf(cf, "-t\n");
					break;
				case JTAG_FTDI_CBUS:
					fprintf(cf, "-s\n");
					break;
				case JTAG_FTDI_BITBANG_CBUS_READ:
					fprintf(cf, "-f\n");
					break;
				default:
					break;
			}
		}

		fclose(cf);

		printf("Wrote settings file:\n  \"");
		SetBoldText();
		printf("%s", config_file);
		SetNormalText();
		printf("\"\n");
		fflush(stdout);
	}
}

void readConfig(void)
{
	char config_file[4096];
	char str[4096];
	const char *userprofile = NULL;
	char *fr = NULL;
	FILE *cf;

#if defined(WIN32)	
	userprofile = getenv("LOCALAPPDATA");
	if (!userprofile)
		userprofile = getenv("APPDATA");
	if (!userprofile)
		userprofile = "./";
	
	sprintf(config_file, "%s\\FleaFPGA\\FleaFPGA-JTAG-settings.cfg", userprofile);
#else
	userprofile = getenv("HOME");
	if (!userprofile)
		userprofile = "./";

	sprintf(config_file, "%s/.FleaFPGA-JTAG-settings.cfg", userprofile);

#warning linux TODO ddtcmd
#endif
	
	if ((cf = fopen(config_file, "rt")) != NULL)
	{
		gConfigLoaded = 1;
		for (;;)
		{
			str[0] = '\0';
			fr = fgets(str, sizeof (str)-1, cf);
			if (fr == NULL || feof(cf))
				break;

			if (strlen(str) && str[strlen(str)-1] == '\n')
				str[strlen(str)-1] = '\0';
			
			if (str[0] == '-')
			{
				switch(str[1])
				{
					case 'v':
						gVerbose = 1;
						printf("Verbose mode enabled\n");
						break;
					case 'c':
						strncpy(ddtcmd_path, &str[3], sizeof (ddtcmd_path)-1);
						break;
					case 'm':
						strncpy(FleaFPGA_Default_Desc, &str[3], sizeof (FleaFPGA_Default_Desc)-1);
						gSpecifiedDevice = 1;
						break;
					case 't':
						gJTAGMode = JTAG_NONE;
						break;
					case 's':
						gJTAGMode = JTAG_FTDI_CBUS;
						break;
					case 'f':
						gJTAGMode = JTAG_FTDI_BITBANG_CBUS_READ;
						break;
					case 'p':
						gParanoidSafety = 1;
						printf("Using \"paranoid\" slow conservative settings.\n");
						break;
					default:
						SetRedText();
						printf("\nError: Bad option \"%s\" in settings file: %s\n", str, config_file);
						SetNormalText();
						gRetCode = -1;
						exit(gRetCode);
						break;
				}
			}
			
		}
	
		fclose(cf);
	}

	if (!gConfigLoaded)
	{
		if (!ddtcmd_path[0])
		{
			printf("Requesting \"ddtcmd.exe\" location...\n");
			fflush(stdout);
		
//			FileRequest("Select the \"ddtcmd.exe\" utility under Lattice Programmer \"bin\" directory (saved to settings)", "Executable (*.exe)\0*.exe\0All files\0*.*\0", ddtcmd_path, sizeof (ddtcmd_path));
		}
		writeConfig();
	}
}

static char generic_op_msg[] = "JTAG OPERATION";

#if defined(__linux__)
void strlwr(char *str)
{
	int i;
	for (i = 0; str[i]; i++)
	{
	  str[i] = tolower(str[i]);
	}
}
#endif

int ProcessFile(char *filename)
{
	int rc  = 0;
	char *e = NULL;
	
	memset(orig_op_str, 0, sizeof (orig_op_str));
	strcpy(xcf_file, filename);
	strcpy(vme_file, filename);

	if ((e = strrchr(xcf_file, '.')))
	{
		strlwr(e);
		if (strcmp(e, ".xcf") == 0)
		{
			if (processXCF(xcf_file, vme_file) != 0)
			{
				SetRedText();
				printf("Error: XCF processing failed.\n");
				SetNormalText();
				gRetCode = -1;
				exit(gRetCode);
			}
		}
		else
			memset(xcf_file, 0, sizeof(xcf_file));
	}
	else
		memset(xcf_file, 0, sizeof(xcf_file));

	if (gJTAGMode != JTAG_NONE && (rc = openJTAGDevice()) != 0)
	{
		SetRedText();

		if (rc == -1)
		{
			printf("\nError: FleaFPGA FT230x USB device not detected.\n\n");
		}
		else if (rc == -2)
		{
			printf("\nError: Check that FleaFPGA FT230x EEPROM is set for CBUS0-3 GPIO.\n\n");
		}
		else
		{
			printf("\nError: FTDI error?\n\n");
		}
		SetNormalText();
		gRetCode = -1;
		exit(gRetCode);
	}
	
	printf("JTAG method: %s\n\n", FTDIModeName[gJTAGMode]);

	e = strrchr(vme_file, '-');
	if (e && strstr(e, "Vrfy") && !strstr(e, "ID"))
	{
		int usb_tps = BenchmarkUSB();
		if (usb_tps < 1000)
		{
			SetYellowText();
			printf("WARNING: Verify is EXTREMELY slow (limited by slow ~%d USB transactions/sec).\n", usb_tps);
			printf("         Around 6 hours for SRAM/Flash verify (USB hub may increase speed 10x)!\n\n");
			SetNormalText();
		}
		else if (usb_tps < 10000)
		{
			SetYellowText();
			printf("NOTE: Verify can be slow (limited by ~%d USB transactions/sec).\n", usb_tps);
			printf("      Typically ~14 minutes for SRAM/Flash verify at this speed.\n\n");
			SetNormalText();
		}
	}

	if (e && (strstr(e, "UFM_Read") || strstr(e, "Read_and_Save")))
	{
		int usb_tps = BenchmarkUSB();
		if (usb_tps < 500)
		{
			SetYellowText();
			printf("WARNING: Read is EXTREMELY slow (limited by slow ~%d USB transactions/sec).\n", usb_tps);
			printf("         Around 6 hours for SRAM/Flash read (USB hub may increase speed 10x)!\n\n");
			SetNormalText();
		}
		else if (usb_tps < 10000)
		{
			SetYellowText();
			printf("NOTE: Read can be slow (limited by ~%d USB transactions/sec).\n", usb_tps);
			printf("      Typically ~14 minutes for SRAM/Flash read at this speed.\n\n");
			SetNormalText();
		}
	}

	strncpy(out_file, vme_file, sizeof (out_file)-1);
	e = strrchr(out_file, '.');
	if (e)
		*e = '\0';
	
	strcat(out_file, "-out.bin");

	printf( "Processing VME file:\n  \"");
	SetBoldText();
	printf("%s", vme_file);
	SetNormalText();
	printf("\"\n\n");
	
	g_JTAGTime = getMilliseconds(0);

	setvbuf(stdout, NULL, _IOLBF, 4096);
	
	rc = ispVM( vme_file );
	
	g_JTAGTime = getMilliseconds(g_JTAGTime);
//	OpenLine(2);
	CloseLine();

	if (out_fp)
		fclose(out_fp);

	closeJTAGDevice();
		
	fflush(stdout);

	setvbuf(stdout, NULL, _IONBF, 0);

	if ( rc >= 0 )
	{
		SetGreenText();
		printf("+==========================================+\n" );
		if (orig_op_str[0])
			printf("|%*.s%s%*s|\n", (42 - strlen(orig_op_str))/2, " ", orig_op_str, (42 - strlen(orig_op_str)+1)/2, " ");
		else
			printf("|%*.s%s%*s|\n", (42 - strlen(generic_op_msg))/2, " ", generic_op_msg, (42 - strlen(generic_op_msg)+1)/2, " ");

		printf("|               SUCCESSFUL!                |\n" );
		printf("+==========================================+\n\n" );
		SetNormalText();

		//08/28/08 NN Added Calculate checksum support.
		if(g_usChecksum != 0)
		{
			g_usChecksum &= 0xFFFF;
			printf("Data Checksum: %.4X\n\n",g_usChecksum);
			g_usChecksum = 0;
		}
		
		if (USBTransactions)
			printf("Operation time    : %d:%02d.%d\n", g_JTAGTime / (60 * 1000), (g_JTAGTime / 1000) % 60, (g_JTAGTime % 1000) / 100);

		if (NumberClockRuns)
			printf("JTAG bits out/in  : %d/%d (bps %d, avg. run %d)\n", TotalClocks, TDOToggle, (TotalClocks * 1000) / g_JTAGTime, TotalClocks / NumberClockRuns);
		else
			printf("JTAG bits out/in  : %d/%d (bps %d)\n", TotalClocks, TDOToggle, (TotalClocks * 1000) / g_JTAGTime);
	}
	
	return rc;
}

/***************************************************************
*
* main
*
***************************************************************/

int main( int argc, char * argv[] )
{
	int i  = 0;

	g_TotalTime = getMilliseconds(0);
	
	setvbuf(stdout, NULL, _IONBF, 0);

	atexit(quitmsg);

	//08/28/08 NN Added Calculate checksum support.
	g_usChecksum = 0;
	g_uiChecksumIndex = 0;

	SetTitleBar("FleaFPGA-JTAG");
	SetPurpleText();
	printf( "    FleaFPGA-JTAG utility by Xark (built: " __DATE__ " "__TIME__")\n");
	printf( "        based on ispVME(tm) V");
	printf( VME_VERSION_NUMBER );
	printf(" Copyright 1998-2011.\n");
	printf( "                 Lattice Semiconductor Corp.\n" );
	printf( "\n");
	SetNormalText();

	readConfig();
	
	gRetCode = 0;
	for ( i = 1; i < argc; i++ )
	{   /* Process all VME files sequentially */
	
		if (argv[i][0] == '-')
		{
			switch(argv[i][1])
			{
				case 'v':
					gVerbose = !gVerbose;
					if (gVerbose)
						printf("Verbose mode enabled.\n");
					else
						printf("Verbose mode disabled.\n");
					break;
				case 'a':
					gAutomatic = 1;
					printf("Automatic monitoring mode enabled (process when JED/BIT changes)\n");
					break;
				case 'l':
					ListDevices();
					break;
				case 'c':
					if (argv[i][2] != '\0')
					{
						strncpy(ddtcmd_path, &argv[i][2], sizeof (ddtcmd_path)-1);
					}
					else if (i+1 < argc)
					{
						i++;
						strncpy(ddtcmd_path, argv[i], sizeof (ddtcmd_path)-1);
					}
					else
					{
						SetRedText();
						printf("\nError: Need full path to \"ddtcmd.exe\" following \"-c\"\n");
						SetNormalText();
						gRetCode = -1;
						exit(gRetCode);
					}
					break;
				case 'm':
					if (argv[i][2] != '\0')
					{
						strncpy(FleaFPGA_Default_Desc, &argv[i][2], sizeof (FleaFPGA_Default_Desc)-1);
						gSpecifiedDevice = 1;
					}
					else if (i+1 < argc)
					{
						i++;
						strncpy(FleaFPGA_Default_Desc, argv[i], sizeof (FleaFPGA_Default_Desc)-1);
						gSpecifiedDevice = 1;
					}
					else
					{
						SetRedText();
						printf("\nError: Need description string following \"-m\"\n");
						SetNormalText();
						gRetCode = -1;
						exit(gRetCode);
					}
					break;
				case 't':
					gJTAGMode = JTAG_NONE;
					break;
				case 's':
					gJTAGMode = JTAG_FTDI_CBUS;
					break;
				case 'f':
					gJTAGMode = JTAG_FTDI_BITBANG_CBUS_READ;
					break;
				case 'p':
					gParanoidSafety = !gParanoidSafety;
					if (gParanoidSafety)
						printf("Using \"paranoid\" slow conservative settings for bit-rate and delays.\n");
					else
						printf("Using default fast settings.\n");
					break;
				case 'q':
					memset(gKillTitleStrs, 0, sizeof(gKillTitleStrs));
					if (argv[i][2] != '\0')
					{
						strncpy(gKillTitleStrs, &argv[i][2], sizeof (gKillTitleStrs)-2);
					}
					else if (i+1 < argc)
					{
						i++;
						strncpy(gKillTitleStrs, argv[i], sizeof (gKillTitleStrs)-1);
					}
					else
					{
						SetRedText();
						printf("\nError: Need comma separated window title substring(s) for \"-q\"\n");
						SetNormalText();
						gRetCode = -1;
						exit(gRetCode);
					}
					break;
				case 'r':
					memset(gRestartPath, 0, sizeof(gRestartPath));
					if (argv[i][2] != '\0')
					{
						strncpy(gRestartPath, &argv[i][2], sizeof (gRestartPath)-1);
					}
					else if (i+1 < argc)
					{
						i++;
						strncpy(gRestartPath, argv[i], sizeof (gRestartPath)-1);
					}
					else
					{
						SetRedText();
						printf("\nError: Need application path for \"-r\"\n");
						SetNormalText();
						gRetCode = -1;
						exit(gRetCode);
					}
					break;
				case 'w':
					writeConfig();
					break;
				case 'i':
					if (strcmp(argv[i], "-init") == 0)
					{
						InitFleaFPGA();
						break;
					}
				default:
					SetRedText();
					printf("\nError: Unrecognised option '%c'\n", argv[i][1]);
					SetNormalText();
				case '?':
					showHelp();
					exit(1);
					break;
			}
			continue;
		}
		else
		{
			strcpy( szCommandLineArg, argv[ i ] );
		}

		gRetCode = ProcessFile(szCommandLineArg);
		
		if (gRetCode < 0)
			break;
	}

#if defined(WIN32)
	if (!did_file)
	{
		printf("Requesting file to process...\n");
		fflush(stdout);
	
		int rc = 0;
		rc = FileRequest("Select FleaFPGA JTAG file for processing", "JTAG file (*.xcf;*.vme)\0*.xcf;*.vme\0All files\0*.*\0", szCommandLineArg, sizeof (szCommandLineArg));
		
		if (rc)
			gRetCode = ProcessFile(szCommandLineArg);
	}
#endif			
	
	if ( gRetCode < 0 )
	{
		error_handler( gRetCode, szCommandLineArg );
		SetRedText();
		printf( "\n");
		printf("+==========================================+\n" );
		if (orig_op_str[0])
			printf("|%*.s%s%*s|\n", (42 - strlen(orig_op_str))/2, " ", orig_op_str, (42 - strlen(orig_op_str)+1)/2, " ");
		else
			printf("|%*.s%s%*s|\n", (42 - strlen(generic_op_msg))/2, " ", generic_op_msg, (42 - strlen(generic_op_msg)+1)/2, " ");
		printf("|                  FAILED!                 |\n" );
		printf("+==========================================+\n\n" );
		printf( "Error: %s\n\n", szCommandLineArg );
		SetNormalText();
	}

	g_TotalTime = getMilliseconds(g_TotalTime);
	
	if (g_TotalTime)
	{
		printf("Total time        : %d:%02d.%d\n", g_TotalTime / (60 * 1000), (g_TotalTime / 1000) % 60, (g_TotalTime % 1000) / 100);
	}
	
	exit( gRetCode );
} 
