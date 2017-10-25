//
//  AMCDrive.cpp
//  AMCDrive X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "AMCDrive.h"

CAMCDrive::CAMCDrive()
{
    // set some sane values
    m_bDebugLog = true;
    
    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nNbTicksPerRev = 0;

    m_dHomeAz = 0.0;
    m_dParkAz = 0.0;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

    m_bCalibrating = false;
    
    m_bShutterOpened = false;
    
    m_bParked = true;
    m_bHomed = false;

    m_nHomingTries = 0;
    m_nGotoTries = 0;

    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,LOG_BUFFER_SIZE);
    
#ifdef LOG_DEBUG
    Logfile = fopen(AMC_LOGFILENAME, "w");
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive Constructor Called.\n", timestamp);
    fflush(Logfile);
#endif

}

CAMCDrive::~CAMCDrive()
{
#ifdef	LOG_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif

}

int CAMCDrive::Connect(const char *pszPort)
{
    int nErr;
    
#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect Called %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif


    // 9600 8N1
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect connected to %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::Connect] Connected.");
        m_pLogger->out(m_szLogBuffer);

        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::Connect] Getting Firmware.");
        m_pLogger->out(m_szLogBuffer);
    }

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect Getting Firmware\n", timestamp);
    fflush(Logfile);
#endif


#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect gain write access %s\n", timestamp, m_szFirmwareVersion);
    fflush(Logfile);
#endif

    nErr = gainWriteAccess();

    nErr = enableBridge();

    return SB_OK;
}


void CAMCDrive::Disconnect()
{

    disableBridge();

    if(m_bIsConnected) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
}


int CAMCDrive::readResponse(unsigned char *szRespBuffer, int nBufferLen)
{
    int nErr = OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    unsigned char *pszBufPtr;
    unsigned int nDataLen = 0;
    uint8_t s1,s2;
    uint16_t nCRC;

    memset(szRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = szRespBuffer;

    // read response header
    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::readResponse] header readFile error.");
                m_pLogger->out(m_szLogBuffer);
            }
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::readResponse] header readFile Timeout.");
                m_pLogger->out(m_szLogBuffer);
            }
#ifdef LOG_DEBUG
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] CAMCDrive::readResponse Timeout while waiting for response header from controller\n", timestamp);
            fflush(Logfile);
#endif
            return BAD_CMD_RESPONSE;
        }
        ulTotalBytesRead += ulBytesRead;
        pszBufPtr++;
    } while (ulTotalBytesRead < 8); // header with CRC is 8 bytes

    // crc check the header
    nCRC = crc_xmodem(szRespBuffer, 6);
#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(szRespBuffer, cHexBuf, 8, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::readResponse response header : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
    hexdump((unsigned char*)&nCRC, cHexBuf, 2, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::readResponse response header CRC : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif


    // if(!memcmp(&nCRC, szRespBuffer+6, 2)) // CRC error
    //  return BAD_CMD_RESPONSE;

    s1 = szRespBuffer[3];
    s2 = szRespBuffer[4];

    if(s1 != 1) {// error ?
        return BAD_CMD_RESPONSE;
    }

    nDataLen = szRespBuffer[5] * 2; // value is in 2 word (2 bytes)
    if(nDataLen){

        // read data
        ulTotalBytesRead = 0;
        do {
            nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
            if(nErr) {
                if (m_bDebugLog) {
                    snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::readResponse] data readFile error.");
                    m_pLogger->out(m_szLogBuffer);
                }
                return nErr;
            }

            if (ulBytesRead !=1) {// timeout
                if (m_bDebugLog) {
                    snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::readResponse] data readFile Timeout.");
                    m_pLogger->out(m_szLogBuffer);
                }
#ifdef LOG_DEBUG
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] CAMCDrive::readResponse Timeout while waiting for response data from controller\n", timestamp);
                fflush(Logfile);
#endif
                return BAD_CMD_RESPONSE;
            }
            ulTotalBytesRead += ulBytesRead;
            pszBufPtr++;
        } while (ulTotalBytesRead < (nDataLen + 2)); // datalen + crc

        // crc check the data
        nCRC = crc_xmodem(szRespBuffer + 8, nDataLen);
#ifdef LOG_DEBUG
        unsigned char cHexBuf[LOG_BUFFER_SIZE];
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        hexdump(szRespBuffer + 8, cHexBuf, nDataLen, LOG_BUFFER_SIZE);
        fprintf(Logfile, "[%s] CAMCDrive::readResponse response data : %s\n", timestamp, cHexBuf);
        fflush(Logfile);
        hexdump((unsigned char*)&nCRC, cHexBuf, 2, LOG_BUFFER_SIZE);
        fprintf(Logfile, "[%s] CAMCDrive::readResponse response data CRC : %s\n", timestamp, cHexBuf);
        fflush(Logfile);
#endif

        // if(!memcmp(&nCRC, szRespBuffer + 8 + nDataLen, 2)) // CRC error
        //  return BAD_CMD_RESPONSE;
    }

    return nErr;
}


int CAMCDrive::domeCommand(const unsigned char *pszCmd, int nCmdSize, unsigned char *pszResult, int nResultMaxLen)
{
    int nErr = 0;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(pszCmd , cHexBuf, nCmdSize, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::domeCommand sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, nCmdSize, ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    // read response
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {

#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        int resp_len = szResp[5]*2;
        if(resp_len)
            resp_len +=2;
        resp_len += 8; // header
        hexdump(szResp , cHexBuf, resp_len , LOG_BUFFER_SIZE);
        fprintf(Logfile, "[%s] CAMCDrive::domeCommand ***** ERROR READING RESPONSE **** error = %d , response : %s\n\n", timestamp, nErr, cHexBuf);
        fflush(Logfile);
#endif

        return nErr;
    }
#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    int resp_len = szResp[5]*2;
    if(resp_len)
        resp_len +=2;
    resp_len += 8; // header
    hexdump(szResp , cHexBuf, resp_len, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::domeCommand response : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
    fprintf(Logfile, "[%s] .................................\n", timestamp);
    fflush(Logfile);
#endif

    if(pszResult)
        memcpy(pszResult, szResp, nResultMaxLen);

    return nErr;

}

int CAMCDrive::getDomeAz(double &dDomeAz)
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    int nTicks = 0;
    
    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_READ;
    cmdBuf[3] = POS_I;
    cmdBuf[4] = POS_O;
    cmdBuf[5] = POS_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::getDomeAz sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif
    // send command and get response.
    nErr = domeCommand(cmdBuf, 8, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    memcpy(&nTicks, szResp+8, 4);
    TicksToAz(nTicks, m_dCurrentAzPosition);
    dDomeAz = m_dCurrentAzPosition;

    return nErr;
}

int CAMCDrive::getDomeEl(double &dDomeEl)
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterOpened)
        dDomeEl = 0.0;
    else
        dDomeEl = 90.00;

    return nErr;
}


int CAMCDrive::getShutterState(int &nState)
{
    int nErr = 0;
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nState = OPEN;

    return nErr;
}


int CAMCDrive::getDomeTicksPerRev(int &nTicksPerRev)
{
    int nErr = 0;
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nTicksPerRev = m_nNbTicksPerRev;
    return nErr;
}

void CAMCDrive::setDebugLog(bool bEnable)
{
    m_bDebugLog = bEnable;
}

bool CAMCDrive::isDomeMoving()
{
    bool bIsMoving = false;
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t nStatus;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_READ;
    cmdBuf[3] = STATUS_I;
    cmdBuf[4] = STATUS_2_O;
    cmdBuf[5] = STATUS_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::isDomeMoving sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return false;

    memcpy(&nStatus, szResp+8, 2);
#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::isDomeMoving nStatus : %04x\n", timestamp, nStatus);
    fflush(Logfile);
#endif

    if((nStatus & STATUS_MOVING) == STATUS_MOVING)
        bIsMoving = true;

    return bIsMoving;
}

bool CAMCDrive::isDomeAtHome()
{
    bool bAthome = false;
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t nStatus;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_READ;
    cmdBuf[3] = STATUS_I;
    cmdBuf[4] = STATUS_2_O;
    cmdBuf[5] = STATUS_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::isDomeAtHome sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return false;

    memcpy(&nStatus, szResp+8, 2);
#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::isDomeAtHome nStatus : %04x\n", timestamp, nStatus);
    fflush(Logfile);
#endif

    if((nStatus & STATUS_HOMING) == STATUS_HOMING)
        bAthome = false;

    if((nStatus & STATUS_AT_HOME) == STATUS_AT_HOME)
        bAthome = true;

    return bAthome;
}

// use for calibration.. maybe if it's fast enough and can be consistent.
/*
#if defined(SB_LINUX_BUILD) || defined(SB_MAC_BUILD)
void CAMCDrive::threadCallback(void *param)
#else if defined(SB_WIN_BUILD)
DWORD WINAPI CAMCDrive::threadCallback(LPVOID param)
#endif
{
    class CAMCDrive* self = static_cast<class CAMCDrive *>(param);
    int nTicks;
    int nLastTicks = 0;
    while(true) {
        if(self->isDomeAtHome()) { // calibration is done
            self->setNbTicksPerRev(nLastTicks);
            break;
        }
        // check the current position
        self->getDomeTicksPerRev(nTicks);
        // is it above the previous one
        // store if yes.
        if(nTicks > nLastTicks)
            nLastTicks = nTicks;

        // yield to other threads
#if defined(SB_LINUX_BUILD)
        pthread_yield();
#elseif defined(SB_MAC_BUILD)
        pthread_yield_np();
#elseif defined(SB_WIN_BUILD)
        SwitchToThread();
#endif
    }
}
*/

int CAMCDrive::gainWriteAccess()
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t data[WR_ACCESS_L];

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE;
    cmdBuf[3] = WR_ACCESS_I;
    cmdBuf[4] = WR_ACCESS_O;
    cmdBuf[5] = WR_ACCESS_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    data[0] = WR_ACCESS_D;
    memcpy(cmdBuf + 8, data, WR_ACCESS_L*2);
    nCRC = crc_xmodem((const unsigned char *)data, WR_ACCESS_L * 2);

    cmdBuf[8+(WR_ACCESS_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(WR_ACCESS_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+WR_ACCESS_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::gainWriteAccess sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    // send command and get response.
    nErr = domeCommand(cmdBuf, 8+WR_ACCESS_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CAMCDrive::enableBridge()
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    uint16_t data[BRIDGE_L];

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE;
    cmdBuf[3] = BRIDGE_I;
    cmdBuf[4] = BRIDGE_O;
    cmdBuf[5] = BRIDGE_L;
    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    data[0] = 0x0000;
    memcpy(cmdBuf + 8, data, BRIDGE_L*2);

    nCRC = crc_xmodem((const unsigned char *)data, BRIDGE_L * 2);

    cmdBuf[8+(BRIDGE_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(BRIDGE_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+BRIDGE_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::enableBridge sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    // send command and get response.
    nErr = domeCommand(cmdBuf, 8+BRIDGE_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}
int CAMCDrive::disableBridge()
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    uint16_t data[BRIDGE_L];

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE;
    cmdBuf[3] = BRIDGE_I;
    cmdBuf[4] = BRIDGE_O;
    cmdBuf[5] = BRIDGE_L;
    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    data[0] = 0x0001;
    memcpy(cmdBuf + 8, data, BRIDGE_L*2);

    nCRC = crc_xmodem((const unsigned char *)data, BRIDGE_L * 2);

    cmdBuf[8+(BRIDGE_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(BRIDGE_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+BRIDGE_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::disableBridge sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif
    // send command and get response.
    nErr = domeCommand(cmdBuf, 8+BRIDGE_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}


int CAMCDrive::syncDome(double dAz, double dEl)
{
    int nErr = 0;
    char szBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    return nErr;
}

int CAMCDrive::parkDome()
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = gotoAzimuth(m_dParkAz);
    return nErr;

}

int CAMCDrive::unparkDome()
{
    m_bParked = false;
    m_dCurrentAzPosition = m_dParkAz;
    // syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
    return 0;
}

int CAMCDrive::gotoAzimuth(double dNewAz)
{
    int nErr = 0;
    int nPosInTicks;

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    while(dNewAz >= 360)
        dNewAz = dNewAz - 360;

    AzToTicks(dNewAz, nPosInTicks);
    nErr = gotoTicksPosition(nPosInTicks);
    // if(nErr)
    //    return nErr;

    m_dGotoAz = dNewAz;
    return nErr;
}

int CAMCDrive::gotoTicksPosition(int ticks)
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE;
    cmdBuf[3] = GOTO_I;
    cmdBuf[4] = GOTO_O;
    cmdBuf[5] = GOTO_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    memcpy(cmdBuf + 8, &ticks, GOTO_L*2);
    nCRC = crc_xmodem((const unsigned char *)&ticks, GOTO_L * 2);

    cmdBuf[8+(GOTO_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(GOTO_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+GOTO_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::gotoTicksPosition sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8 + GOTO_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        printf("nErr = %d\n", nErr);

    return nErr;
}


int CAMCDrive::openShutter()
{
    int nErr = 0;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::openShutter] Opening shutter");
        m_pLogger->out(m_szLogBuffer);
    }

    // TBD : REST call

    if(nErr) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::openShutter] ERROR gotoAzimuth");
        m_pLogger->out(m_szLogBuffer);
    }

    return nErr;
}

int CAMCDrive::closeShutter()
{
    int nErr = 0;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::closeShutter] Closing shutter");
        m_pLogger->out(m_szLogBuffer);
    }

    // TBD : REST call

    if(nErr) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::closeShutter] ERROR Closing shutter");
        m_pLogger->out(m_szLogBuffer);
    }

    return nErr;
}

int CAMCDrive::getFirmwareVersion(char *szVersion, int nStrMaxLen)
{
    int nErr = 0;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> firmwareFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    // FIXME
    snprintf(szVersion, nStrMaxLen, "1.0" );

    return nErr;
}

int CAMCDrive::getFirmwareVersion(float &fVersion)
{
    int nErr = OK;

    if(m_fVersion == 0.0) {
        nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
        if(nErr)
            return nErr;
    }

    fVersion = m_fVersion;

    return nErr;
}

int CAMCDrive::goHome()
{
    int nErr = 0;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating) {
        return SB_OK;
    }
    else if(isDomeAtHome()){
            m_bHomed = true;
            return OK;
    }
#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::goHome \n", timestamp);
    fflush(Logfile);
#endif
    // TBD : send homing command
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t data[HOME_L];

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE;
    cmdBuf[3] = HOME_I;
    cmdBuf[4] = HOME_O;
    cmdBuf[5] = HOME_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    data[0] = HOME_D;
    memcpy(cmdBuf + 8, data, HOME_L*2);

    nCRC = crc_xmodem((const unsigned char *)data, BRIDGE_L * 2);

    cmdBuf[8+(HOME_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(HOME_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+HOME_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::goHome sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8 + HOME_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CAMCDrive::calibrate()
{
    int nErr = 0;
    if(!m_bIsConnected)
        return NOT_CONNECTED;


    // m_bCalibrating = true;
    
    return nErr;
}

int CAMCDrive::isGoToComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        bComplete = false;
        getDomeAz(dDomeAz);
        return nErr;
    }

    getDomeAz(dDomeAz);
    if(dDomeAz >0 && dDomeAz<1)
        dDomeAz = 0;
    
    while(ceil(m_dGotoAz) >= 360)
          m_dGotoAz = ceil(m_dGotoAz) - 360;

    while(ceil(dDomeAz) >= 360)
        dDomeAz = ceil(dDomeAz) - 360;

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::isGoToComplete DomeAz = %3.2f\n", timestamp, dDomeAz);
    fflush(Logfile);
#endif

    // we need to test "large" depending on the heading error
    if ((ceil(m_dGotoAz) <= ceil(dDomeAz)+3) && (ceil(m_dGotoAz) >= ceil(dDomeAz)-3)) {
        bComplete = true;
        m_nGotoTries = 0;
    }
    else {
#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CAMCDrive::isGoToComplete ***** ERROR **** DomeAz = %3.2f\n", timestamp, dDomeAz);
        fflush(Logfile);
#endif
        // we're not moving and we're not at the final destination !!!
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::isGoToComplete] domeAz = %f, m_dGotoAz = %f", ceil(dDomeAz), ceil(m_dGotoAz));
            m_pLogger->out(m_szLogBuffer);
        }
        if(m_nGotoTries == 0) {
            bComplete = false;
            m_nGotoTries = 1;
            gotoAzimuth(m_dGotoAz);
        }
        else {
            m_nGotoTries = 0;
            nErr = ERR_CMDFAILED;
        }
    }

    return nErr;
}

int CAMCDrive::isOpenComplete(bool &bComplete)
{
    int nErr = 0;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
    if(nState == OPEN){
        m_bShutterOpened = true;
        bComplete = true;
        m_dCurrentElPosition = 90.0;
    }
    else {
        m_bShutterOpened = false;
        bComplete = false;
        m_dCurrentElPosition = 0.0;
    }

    return nErr;
}

int CAMCDrive::isCloseComplete(bool &bComplete)
{
    int nErr = 0;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
    if(nState == CLOSED){
        m_bShutterOpened = false;
        bComplete = true;
        m_dCurrentElPosition = 0.0;
    }
    else {
        m_bShutterOpened = true;
        bComplete = false;
        m_dCurrentElPosition = 90.0;
    }

    return nErr;
}


int CAMCDrive::isParkComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz=0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        getDomeAz(dDomeAz);
        bComplete = false;
        return nErr;
    }

    getDomeAz(dDomeAz);

    if (ceil(m_dParkAz) == ceil(dDomeAz))
    {
        m_bParked = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}

int CAMCDrive::isUnparkComplete(bool &bComplete)
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bParked = false;
    bComplete = true;

    return nErr;
}

int CAMCDrive::isFindHomeComplete(bool &bComplete)
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        m_bHomed = false;
        bComplete = false;
#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete still moving\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;
    }

    if(isDomeAtHome()){
        m_bHomed = true;
        bComplete = true;
        syncDome(m_dHomeAz, m_dCurrentElPosition);
        m_nHomingTries = 0;
#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete At Home\n", timestamp);
        fflush(Logfile);
#endif
    }
    else {
        // we're not moving and we're not at the home position !!!
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::isFindHomeComplete] Not moving and not at home !!!");
            m_pLogger->out(m_szLogBuffer);
        }
#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CAMCDrive::isFindHomeComplete] Not moving and not at home !!!\n", timestamp);
        fflush(Logfile);
#endif
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        // sometimes we pass the home sensor so give it another try
        if(m_nHomingTries == 0 ) {
            m_nHomingTries = 1;
            goHome();
        }
        else {
            m_nHomingTries = 0;
            nErr = ERR_CMDFAILED;
        }
    }

    return nErr;
}


int CAMCDrive::isCalibratingComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        // getDomeAz(dDomeAz);
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

    
    nErr = getDomeAz(dDomeAz);

    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
        m_bHomed = true;
        bComplete = true;
    }

    // nErr = getDomeTicksPerRev(m_nNbTicksPerRev);
    m_bHomed = true;
    bComplete = true;
    m_bCalibrating = false;
    return nErr;
}


int CAMCDrive::abortCurrentCommand()
{
    int nErr = 0;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    // 0xC9
    
    m_bCalibrating = false;

    return nErr;
}


/*
 Convert pdAz to number of ticks from home.
 */
void CAMCDrive::AzToTicks(double pdAz, int &ticks)
{
    ticks = (int) floor(0.5 + (pdAz - m_dHomeAz) * m_nNbTicksPerRev / 360.0);
    while (ticks > m_nNbTicksPerRev) ticks -= m_nNbTicksPerRev;
    while (ticks < 0) ticks += m_nNbTicksPerRev;
}


/*
 Convert ticks from home to Az
 */

void CAMCDrive::TicksToAz(int ticks, double &pdAz)
{

    pdAz = m_dHomeAz + (ticks * 360.0 / m_nNbTicksPerRev);
    while (pdAz < 0) pdAz += 360;
    while (pdAz >= 360) pdAz -= 360;
}


#pragma mark - Getter / Setter

int CAMCDrive::getNbTicksPerRev()
{
    return m_nNbTicksPerRev;
}

int CAMCDrive::setNbTicksPerRev(int nTicks)
{
    int nErr = 0;
    m_nNbTicksPerRev = nTicks;
    return nErr;
}

double CAMCDrive::getHomeAz()
{
    return m_dHomeAz;
}

int CAMCDrive::setHomeAz(double dAz)
{
    int nErr = 0;

    m_dHomeAz = dAz;

    return nErr;
}


double CAMCDrive::getParkAz()
{
    return m_dParkAz;

}

int CAMCDrive::setParkAz(double dAz)
{
    int nErr = 0;

    m_dParkAz = dAz;

    return nErr;
}


double CAMCDrive::getCurrentAz()
{
    if(m_bIsConnected)
        getDomeAz(m_dCurrentAzPosition);
    
    return m_dCurrentAzPosition;
}

double CAMCDrive::getCurrentEl()
{
    if(m_bIsConnected)
        getDomeEl(m_dCurrentElPosition);
    
    return m_dCurrentElPosition;
}

int CAMCDrive::getCurrentShutterState()
{
    if(m_bIsConnected)
        getShutterState(m_nShutterState);

    return m_nShutterState;
}



int CAMCDrive::parseFields(char *pszResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = OK;
    std::string sSegment;
    std::stringstream ssTmp(pszResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}


void CAMCDrive::hexdump(const unsigned char* pszInputBuffer, unsigned char *pszOutputBuffer, int nInputBufferSize, int nOutpuBufferSize)
{
    unsigned char *pszBuf = pszOutputBuffer;
    int nIdx=0;

    memset(pszOutputBuffer, 0, nOutpuBufferSize);
    for(nIdx=0; nIdx < nInputBufferSize && pszBuf < (pszOutputBuffer + nOutpuBufferSize -3); nIdx++){
        snprintf((char *)pszBuf,4,"%02X ", pszInputBuffer[nIdx]);
        pszBuf+=3;
    }
}

