/* FleaFPGA FT230X JTAG defines */

#define MAX_USB_DEVICES		16
#define ASYNC_BB_RATE		921600
#define ASYNC_BB_RATE_SLOW	115200
#define PRINT_MAX_COLUMN	77
#define MAX_CLOCK_RUN		(65536/3)	// L H L
#define AUTO_POLL_DELAY_MS	3500

enum
{
	JTAG_NONE,						// -t "test"
	JTAG_FTDI_CBUS,					// -s "slow"
	JTAG_FTDI_BITBANG_CBUS_READ,	// -n "new"
	JTAG_NUM_MODES
};

// FTDI CBUS pins
#define	cbusTDI		0x1	// CB0
#define	cbusTDO		0x2	// CB1 (always used)
#define	cbusTMS		0x4	// CB2
#define	cbusTCK		0x8	// CB3

// FTDI bit-bang pins (still uses CBUS for TDO)
#define	abbmTMS		0x1	// TXD
						// CB1
#define	abbmTDI		0x4	// RTS
#define	abbmTCK		0x8 // CTS

extern unsigned int TotalClocks;
extern unsigned int LongestRun;
extern unsigned int USBTransactions;
extern unsigned int NumberClockRuns;
extern unsigned int g_TotalTime;
extern unsigned int g_JTAGTime;
extern int gRetCode;
extern int gColumn;
extern int gVerbose;
extern int gWait;
extern char gKillTitleStrs[256];
extern char gRestartPath[256];
extern int TDOToggle;
extern const char *FTDIModeName[JTAG_NUM_MODES];
extern int gJTAGMode;
extern int gParanoidSafety;
extern int gAutomatic;
extern int gSpecifiedDevice;
extern char FleaFPGA_Default_Desc[64];
extern uint32_t getMilliseconds(uint32_t start);
extern void OpenLine(int force);
extern void CloseLine(void);
extern void Close_FTDI(void);
extern int BenchmarkUSB(void);
extern int openJTAGDevice(void);
extern void closeJTAGDevice(void);
extern void ListDevices(void);
extern void EnableHardware(void);
extern void DisableHardware(void);
extern void TerminateApp(void);
extern void RestartApp(void);

