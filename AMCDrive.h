//
//  AMCDrive.h
//  AMCDrive
//
//  Created by Rodolphe Pineau on 10/23/2017.
//  AMCDrive X2 plugin

#ifndef __AMCDrive__
#define __AMCDrive__

// standard C includes
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#include <math.h>
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif
#if defined(SB_LINUX_BUILD) || defined(SB_MAC_BUILD)
// #include <pthread.h>
#endif

// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

// SB includes
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

// CRC16 stuff
extern "C"
{
#include "checksum.h"
}

#define SERIAL_BUFFER_SIZE 1024
#define MAX_TIMEOUT 1000
#define LOG_BUFFER_SIZE 2048

#define LOG_DEBUG

#ifdef LOG_DEBUG
#if defined(SB_WIN_BUILD)
#define AMC_LOGFILENAME "C:\\AMCDriveLog.txt"
#elif defined(SB_LINUX_BUILD)
#define AMC_LOGFILENAME "/tmp/AMCDriveLog.txt"
#elif defined(SB_MAC_BUILD)
#define AMC_LOGFILENAME "/tmp/AMCDriveLog.txt"
#endif
#endif

// header define
#define SOF         0xA5
#define DA          0x3F
#define CB_WRITE    0x02
#define CB_READ     0x01

// gain write access
#define WR_ACCESS_I 0x07
#define WR_ACCESS_O 0x00
#define WR_ACCESS_L 0x01
#define WR_ACCESS_D 0x000F

// bridge access
#define BRIDGE_I 0x01
#define BRIDGE_O 0x00
#define BRIDGE_L 0x01
// enable bridge
#define EN_BRIDGE_D 0x0000
// disable bridge
#define DIS_BRIDGE_D 0x0001

// goto position
#define GOTO_I  0x45
#define GOTO_O  0x00
#define GOTO_L  0x02

// get position
#define POS_I  0x12
#define POS_O  0x00
#define POS_L  0x02

// clear error
#define CLEAR_ERR 0x1000

// Home
#define HOME_I 0x01
#define HOME_O 0x00
#define HOME_L 0x01
#define HOME_D 0x0020

// Stop
#define STOP_I 0x01
#define STOP_O 0x00
#define STOP_L 0x01
#define STOP_D 0x0040

// Sync
#define SYNC_I 0x01
#define SYNC_O 0x01
#define SYNC_L 0x01
#define SYNC_D 0x0008

// Drive status
#define STATUS_I 0x02
#define STATUS_1_O 0x03
#define STATUS_2_O 0x04
#define STATUS_3_O 0x05
#define STATUS_L 0x01
#define HOMING   0x1000
#define HOMING_COMPLETE  0x4000
#define MOVING   0x0001

// error codes
// Error code
enum AMCDriveErrors {OK=0, NOT_CONNECTED, CANT_CONNECT, BAD_CMD_RESPONSE, COMMAND_FAILED};
enum AMCDriveShutterState {OPEN=1, OPENING, CLOSED, CLOSING, SHUTTER_ERROR};

class CAMCDrive
{
public:
    CAMCDrive();
    ~CAMCDrive();

    int        Connect(const char *pszPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return m_bIsConnected; }

    void        setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void        setSleeprPinter(SleeperInterface *p) {m_pSleeper = p; }
    void        setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double dNewAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(char *szVersion, int nStrMaxLen);
    int getFirmwareVersion(float &fVersion);
    int goHome();
    int calibrate();

    // command complete functions
    int isGoToComplete(bool &bComplete);
    int isOpenComplete(bool &bComplete);
    int isCloseComplete(bool &bComplete);
    int isParkComplete(bool &bComplete);
    int isUnparkComplete(bool &bComplete);
    int isFindHomeComplete(bool &bComplete);
    int isCalibratingComplete(bool &bComplete);

    int abortCurrentCommand();

    // getter/setter
    int getNbTicksPerRev();
    int setNbTicksPerRev(int nTicks);

    double getHomeAz();
    int setHomeAz(double dAz);

    double getParkAz();
    int setParkAz(double dAz);

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();

    void setDebugLog(bool bEnable);
/*
#if defined(SB_LINUX_BUILD) || defined(SB_MAC_BUILD)
    static void threadCallback(void *param);
#else if defined(SB_WIN_BUILD)
    DWORD WINAPI threadCallback(LPVOID param);
#endif
*/

protected:
    
    int             getDomeAz(double &domeAz);
    int             getDomeEl(double &domeEl);
    int             getShutterState(int &state);
    int             getDomeTicksPerRev(int &ticksPerRev);

    bool            isDomeMoving();
    bool            isDomeAtHome();
    int             gainWriteAccess();
    int             enableBridge();
    int             disableBridge();

    int             domeCommand(const unsigned char *cmd, int nCmdSize, unsigned char *result, int resultMaxLen);
    int             readResponse(unsigned char *respBuffer, int bufferLen);
    int             parseFields(char *pszResp, std::vector<std::string> &svFields, char cSeparator);

    void            AzToTicks(double pdAz, int &ticks);
    void            TicksToAz(int ticks, double &pdAz);
    int             gotoTicksPosition(int ticks);

    SerXInterface   *m_pSerx;
    SleeperInterface *m_pSleeper;
    LoggerInterface *m_pLogger;
    bool            m_bDebugLog;
    
    bool            m_bIsConnected;
    bool            m_bHomed;
    bool            m_bParked;
    bool            m_bShutterOpened;
    bool            m_bCalibrating;
    
    int             m_nNbTicksPerRev;
    double          m_dHomeAz;
    double          m_dParkAz;
    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;
    double          m_dGotoAz;

    float           m_fVersion;

    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    int             m_nShutterState;
    bool            m_bShutterOnly;
    char            m_szLogBuffer[LOG_BUFFER_SIZE];
    int             m_nHomingTries;
    int             m_nGotoTries;
#ifdef LOG_DEBUG
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;	  // LogFile
#endif
    void            hexdump(const unsigned char* pszInputBuffer, unsigned char *pszOutputBuffer, int nInputBufferSize, int nOutpuBufferSize);
    
};

#endif

