//
//  AMCDrive.cpp
//  AMCDrive X2 plugin
//
//  Created by Rodolphe Pineau on 10/23/2017.


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
    m_goto_find_home = true;

    m_cSeqNumber = 0;

    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szProdInfo,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,LOG_BUFFER_SIZE);
    
#ifdef LOG_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\AMCDriveLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = "/tmp/AMCDriveLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = "/tmp/AMCDriveLog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
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

#pragma mark - communications

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
    }

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect gain write access\n", timestamp);
    fflush(Logfile);
#endif

    nErr = gainWriteAccess();
    nErr = abortCurrentCommand();
    nErr = enableBridge();

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect Getting Product Info\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getProductInformation(m_szProdInfo, SERIAL_BUFFER_SIZE);

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect m_szProdInfo : %s\n", timestamp, m_szProdInfo);
    fflush(Logfile);
#endif


#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect Getting Firmware\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::Connect m_szFirmwareVersion :  %s\n", timestamp, m_szFirmwareVersion);
    fflush(Logfile);
#endif

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
    fprintf(Logfile, "[%s] CAMCDrive::readResponse response header CRC : %04X\n", timestamp, nCRC);
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
        fprintf(Logfile, "[%s] CAMCDrive::readResponse response data CRC : %04X\n", timestamp, nCRC);
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

#pragma mark - Dome coordinate and state

int CAMCDrive::getDomeAz(double &dDomeAz)
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    uint32_t nTicks = 0;
    
    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_READ | (( m_cSeqNumber++ & 0x0F)<<2);
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
#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::getDomeAz got : %08X (%d ticks)\n", timestamp, nTicks, nTicks);
    fflush(Logfile);
#endif

    TicksToAz(nTicks, m_dCurrentAzPosition);
    dDomeAz = m_dCurrentAzPosition;
    m_nCurrentTicks = nTicks;

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::getDomeAz got : %3.2f degrees\n", timestamp, dDomeAz);
    fflush(Logfile);
#endif

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

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nState = OPEN;

    return nErr;
}


int CAMCDrive::getDomeTicksPerRev(int &nTicksPerRev)
{
    int nErr = 0;

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
    uint16_t nStatus;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(timer.GetElapsedSeconds()<2) {
            // we're checking for movement to quickly, assume it's moving for now
            // this also help at the end of a goto when it slowly get to the actual position
            return true;
    }

    nStatus = getStatus(STATUS_2_O);

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::isDomeMoving nStatus : %04x\n", timestamp, nStatus);
    fflush(Logfile);
#endif

    if((nStatus & MOVING) == 0) { // we're moving.. "Zero Velocity" is 0
        bIsMoving = true;
#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CAMCDrive::isDomeMoving Dome is moving\n", timestamp);
        fflush(Logfile);
#endif
    }
    else if( (nStatus & MOVING) != 0 &&                         // not moving
             (nStatus & HOMING) == HOMING &&                    // homing
             (nStatus & HOMING_COMPLETE) != HOMING_COMPLETE) {  // homing has started but we haven't moved yet
        bIsMoving = true;
#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CAMCDrive::isDomeMoving Dome is homing but not moving yet... assuming we're moving\n", timestamp);
        fflush(Logfile);
#endif
    } else {
        bIsMoving = false;
    }

    return bIsMoving;
}

bool CAMCDrive::isDomeAtHome()
{
    bool bAthome = false;
    uint16_t nStatus;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nStatus = getStatus(STATUS_2_O);

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::isDomeAtHome nStatus : %04x\n", timestamp, nStatus);
    fflush(Logfile);
#endif

    if( (nStatus & HOMING) == HOMING && (nStatus & IN_HOME_POSITION) != IN_HOME_POSITION)
        bAthome = false;

    else if((nStatus & IN_HOME_POSITION) == IN_HOME_POSITION)
        bAthome = true;

    return bAthome;
}

int CAMCDrive::syncDome(double dAz, double dEl)
{
    int nErr = 0;
    int nPosInTicks;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    enableBridge();

    while(dAz >= 360)
        dAz = dAz - 360;

    AzToTicks(dAz, nPosInTicks);
    nErr = syncTicksPosition(nPosInTicks);
    // if(nErr)
    //    return nErr;

    return nErr;
}

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


#pragma mark - Dome motions

int CAMCDrive::gotoAzimuth(double dNewAz)
{
    int nErr = 0;
    int nPosInTicks;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    enableBridge();

    while(dNewAz >= 360)
        dNewAz = dNewAz - 360;

    AzToTicks(dNewAz, nPosInTicks);
    nErr = gotoTicksPosition(nPosInTicks);
    // if(nErr)
    //    return nErr;

    m_dGotoAz = dNewAz;
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

    enableBridge();

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::goHome \n", timestamp);
    fflush(Logfile);
#endif


    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t data[HOME_L];
    uint16_t regValue;

    // set homing bit
    regValue = HOME_D;

    // write register
    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
    cmdBuf[3] = HOME_I;
    cmdBuf[4] = HOME_O;
    cmdBuf[5] = HOME_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    data[0] = regValue;
    memcpy(cmdBuf + 8, data, HOME_L*2);

    nCRC = crc_xmodem((const unsigned char *)data, HOME_L * 2);

    cmdBuf[8+(HOME_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(HOME_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+HOME_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::goHome sending for homing : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8 + HOME_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);

    timer.Reset();
    m_goto_find_home = true;
    return nErr;
}


int CAMCDrive::calibrate()
{
    int nErr = 0;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    enableBridge();

    // m_bCalibrating = true;

    return nErr;
}


int CAMCDrive::parkDome()
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::parkDome parking to %3.2f \n", timestamp, m_dParkAz);
    fflush(Logfile);
#endif

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

int CAMCDrive::isGoToComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        bComplete = false;
        // getDomeAz(dDomeAz);
        return nErr;
    }

    if(!isPositionReached()) {
        bComplete = false;
        // getDomeAz(dDomeAz);
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
    fprintf(Logfile, "[%s] CAMCDrive::isGoToComplete DomeAz = %3.2f, m_dGotoAz =  %3.2f\n", timestamp, dDomeAz, m_dGotoAz);
    fflush(Logfile);
#endif

    // we need to test "large" depending on the heading error
    if ((floor(m_dGotoAz) <= floor(dDomeAz)+1) && (floor(m_dGotoAz) >= floor(dDomeAz)-1)) {
        bComplete = true;
        m_nGotoTries = 0;
    }
    else {
#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CAMCDrive::isGoToComplete ***** ERROR **** DomeAz = %3.2f, m_dGotoAz =  %3.2f\n", timestamp, dDomeAz, m_dGotoAz);
        fflush(Logfile);
#endif
        // we're not moving and we're not at the final destination !!!
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CAMCDrive::isGoToComplete] domeAz = %3.2f, m_dGotoAz = %3.2f", floor(dDomeAz), floor(m_dGotoAz));
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

int CAMCDrive::isParkComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz=0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        // getDomeAz(dDomeAz);
        bComplete = false;
        return nErr;
    }

    getDomeAz(dDomeAz);

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::isParkComplete dDomeAz = %3.2f\n", timestamp, dDomeAz);
    fprintf(Logfile, "[%s] CAMCDrive::isParkComplete m_dParkAz = %3.2f\n", timestamp, m_dParkAz);
    fprintf(Logfile, "[%s] CAMCDrive::isParkComplete floor(dDomeAz) = %3.2f\n", timestamp, floor(dDomeAz));
    fprintf(Logfile, "[%s] CAMCDrive::isParkComplete floor(m_dParkAz) = %3.2f\n", timestamp, floor(m_dParkAz));
    fflush(Logfile);
#endif

    if (floor(m_dParkAz) == floor(dDomeAz))
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
    double dDomeAz;
    bool bGotComplete;

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
#ifdef LOG_DEBUG
        // check all status register for debugging
        getAllStatusReg();
#endif
        getDomeAz(dDomeAz);

#ifdef LOG_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete dDomeAz = %3.2f\n", timestamp, dDomeAz);
        fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete m_nCurrentTicks = %d\n", timestamp, m_nCurrentTicks);
        fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete m_goto_find_home = %s\n", timestamp, m_goto_find_home?"true":"false");
        fflush(Logfile);
#endif

        if (m_goto_find_home == true) {
            m_bHomed = false;
            bComplete = false;
            enableBridge();
            gotoAzimuth(m_dHomeAz);
            m_goto_find_home = false; // 1 goto only.
            return SB_OK;
        }
        isGoToComplete(bGotComplete);
        if(!bGotComplete) {
            m_bHomed = false;
            bComplete = false;
            return SB_OK;
        }
        m_bHomed = true;
        bComplete = true;
        m_goto_find_home = true;

        m_nHomingTries = 0;
        enableBridge(); // let's see if this helps.
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
            m_nHomingTries = 1; // dome might still be homing or hasn't statrted to home yet.
            timer.Reset();
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

    if (floor(m_dHomeAz) != floor(dDomeAz)) {
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

#pragma mark - Shutter motions

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

#pragma mark - AMC functions

int CAMCDrive::gainWriteAccess()
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t data[WR_ACCESS_L];

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
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
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
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
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
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



int CAMCDrive::syncTicksPosition(int ticks)
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t data[SYNC_L];
    uint16_t regValue;

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+SET_POSITION_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::syncTicksPosition Sync to ticks : %d\n", timestamp, ticks);
    fflush(Logfile);
#endif

    // set Measured Position Value to new value
    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
    cmdBuf[3] = SET_POSITION_I;
    cmdBuf[4] = SET_POSITION_O;
    cmdBuf[5] = SET_POSITION_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    memcpy(cmdBuf + 8, &ticks, SET_POSITION_L*2);
    nCRC = crc_xmodem((const unsigned char *)&ticks, SET_POSITION_L * 2);

    cmdBuf[8+(SET_POSITION_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(SET_POSITION_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+GOTO_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::syncTicksPosition set Measured Position Value to %d: %s\n", timestamp, ticks, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8 + SET_POSITION_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        printf("nErr = %d\n", nErr);


    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
    cmdBuf[3] = SYNC_I;
    cmdBuf[4] = SYNC_O;
    cmdBuf[5] = SYNC_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    regValue = SYNC_D;
    data[0] = regValue;
    memcpy(cmdBuf + 8, data, SYNC_L*2);
    nCRC = crc_xmodem((const unsigned char *)data, SYNC_L * 2);

    cmdBuf[8+(SYNC_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(SYNC_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+SYNC_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::syncTicksPosition Set Position sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8 + SYNC_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        printf("nErr = %d\n", nErr);

    timer.Reset();
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
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
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
    fprintf(Logfile, "[%s] CAMCDrive::gotoTicksPosition sending data for position %d: %s\n", timestamp, ticks, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8 + GOTO_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        printf("nErr = %d\n", nErr);

    timer.Reset();

    return nErr;
}



int CAMCDrive::getFirmwareVersion(char *szVersion, int nStrMaxLen)
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    size_t nMaxSize;

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_READ | (( m_cSeqNumber++ & 0x0F)<<2);
    cmdBuf[3] = FW_I;
    cmdBuf[4] = FW_O;
    cmdBuf[5] = FW_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    // send command and get response.
    nErr = domeCommand(cmdBuf, 8, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    memset(szVersion, 0, nStrMaxLen);
    if(nStrMaxLen > FW_L)
        nMaxSize = FW_L;
    else
        nMaxSize = nStrMaxLen;

    strncpy(szVersion, (const char *) szResp+8+32, nMaxSize); // firmware name seems to be at offset 32
    return nErr;
}

int CAMCDrive::getFirmwareVersionString(char *szVersion, int nStrMaxLen)
{
    int nErr = SB_OK;
    strncpy(szVersion, m_szFirmwareVersion, nStrMaxLen);
    return nErr;
}

int CAMCDrive::getProductInformation(char *szProdInfo, int nStrMaxLen)
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    size_t nMaxSize;

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_READ | (( m_cSeqNumber++ & 0x0F)<<2);
    cmdBuf[3] = PI_I;
    cmdBuf[4] = PI_O;
    cmdBuf[5] = PI_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    // send command and get response.
    nErr = domeCommand(cmdBuf, 8, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    memset(szProdInfo, 0, nStrMaxLen);
    if(nStrMaxLen > PI_L)
        nMaxSize = PI_L;
    else
        nMaxSize = nStrMaxLen;

    strncpy(szProdInfo, (const char *) szResp+8+2, nMaxSize); // data from 2 to 33 = Control Board Name

    return nErr;
}

int CAMCDrive::getProductInformationString(char *szProdInfo, int nStrMaxLen)
{
    int nErr = SB_OK;
    strncpy(szProdInfo, m_szProdInfo, nStrMaxLen);
    return nErr;
}






int CAMCDrive::abortCurrentCommand()
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t data[STOP_L];
    uint32_t regValue;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    // temp fix
    disableBridge();
    resetEvents();
    // end temp fix

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::abortCurrentCommand \n", timestamp);
    fflush(Logfile);
#endif

    // set STOP bit
    regValue = STOP_D;

    // write register
    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
    cmdBuf[3] = STOP_I;
    cmdBuf[4] = STOP_O;
    cmdBuf[5] = STOP_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    data[0] = regValue;
    
    memcpy(cmdBuf + 8, data, STOP_L*2);

    nCRC = crc_xmodem((const unsigned char *)data, STOP_L * 2);

    cmdBuf[8+(STOP_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(STOP_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+STOP_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::abortCurrentCommand sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8 + STOP_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);

    timer.Reset();
    return nErr;
}

int CAMCDrive::resetEvents()
{
    int nErr = 0;
    uint16_t nCRC;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t data[STOP_L];
    uint32_t regValue;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#ifdef LOG_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CAMCDrive::resetEvents \n", timestamp);
    fflush(Logfile);
#endif

    // set reset events bit
    regValue = RST_EVT_D;

    // write register
    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_WRITE | (( m_cSeqNumber++ & 0x0F)<<2);
    cmdBuf[3] = RST_EVT_I;
    cmdBuf[4] = RST_EVT_O;
    cmdBuf[5] = RST_EVT_L;

    nCRC = crc_xmodem(cmdBuf, 6);
    cmdBuf[6] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[7] = (unsigned char) (nCRC & 0xff);

    data[0] = regValue;

    memcpy(cmdBuf + 8, data, RST_EVT_L*2);

    nCRC = crc_xmodem((const unsigned char *)data, RST_EVT_L * 2);

    cmdBuf[8+(RST_EVT_L*2)] = (unsigned char) ((nCRC>> 8) & 0xff);
    cmdBuf[8+(RST_EVT_L*2)+1] = (unsigned char) (nCRC & 0xff);

#ifdef LOG_DEBUG
    unsigned char cHexBuf[LOG_BUFFER_SIZE];
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(cmdBuf, cHexBuf, 8+RST_EVT_L*2 + 2 , LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CAMCDrive::resetEvents sending : %s\n", timestamp, cHexBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(cmdBuf, 8 + RST_EVT_L*2 + 2, szResp, SERIAL_BUFFER_SIZE);

    timer.Reset();
    return nErr;

}

bool CAMCDrive::isPositionReached()
{
    uint16_t nStatus;
    bool bReached = false;

    nStatus = getStatus(STATUS_2_O);

    if((nStatus & POS_REACHED) == POS_REACHED)
        bReached = true;

    return bReached;
}

uint16_t CAMCDrive::getStatus(unsigned char cStatus)
{
    int nErr = OK;
    unsigned char cmdBuf[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    uint16_t nCRC;
    uint16_t nStatus;

    cmdBuf[0] = SOF;
    cmdBuf[1] = DA;
    cmdBuf[2] = CB_READ | (( m_cSeqNumber++ & 0x0F)<<2);
    cmdBuf[3] = STATUS_I;
    cmdBuf[4] = cStatus; // cStatus
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
    fprintf(Logfile, "[%s] CAMCDrive::getStatus %02x sending : %s\n", timestamp, cStatus, cHexBuf);
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
    fprintf(Logfile, "[%s] CAMCDrive::getStatus nStatus : %04x\n", timestamp, nStatus);
    fflush(Logfile);
#endif

    return nStatus;
}

#ifdef LOG_DEBUG
void CAMCDrive::getAllStatusReg()
{
    int nStatus;

    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    nStatus = getStatus(DRIVE_BRIDGE_STATUS_O);
    fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete DRIVE_BRIDGE_STATUS = %04Xf\n", timestamp, nStatus);

    nStatus = getStatus(DRIVE_PROT_STATUS_O);
    fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete DRIVE_PROT_STATUS = %04Xf\n", timestamp, nStatus);

    nStatus = getStatus(SYS_PROT_STATUS_O);
    fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete SYS_PROT_STATUS = %04Xf\n", timestamp, nStatus);

    nStatus = getStatus(STATUS_1_O);
    fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete STATUS_1 = %04Xf\n", timestamp, nStatus);

    nStatus = getStatus(STATUS_1_O);
    fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete STATUS_1 = %04Xf\n", timestamp, nStatus);

    nStatus = getStatus(STATUS_1_O);
    fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete STATUS_1 = %04Xf\n", timestamp, nStatus);

    nStatus = getStatus(STATUS_2_O);
    fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete STATUS_2 = %04Xf\n", timestamp, nStatus);

    nStatus = getStatus(STATUS_3_O);
    fprintf(Logfile, "[%s] CAMCDrive::isFindHomeComplete STATUS_3 = %04Xf\n", timestamp, nStatus);
    fflush(Logfile);

}
#endif


#pragma mark - helper fucntions

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

#ifdef LOG_DEBUG

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
#endif

