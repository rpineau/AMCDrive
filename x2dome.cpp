#include "x2dome.h"


X2Dome::X2Dome(const char* pszSelection, 
							 const int& nISIndex,
					SerXInterface*						pSerX,
					TheSkyXFacadeForDriversInterface*	pTheSkyXForMounts,
					SleeperInterface*					pSleeper,
					BasicIniUtilInterface*			pIniUtil,
					LoggerInterface*					pLogger,
					MutexInterface*						pIOMutex,
					TickCountInterface*					pTickCount)
{

    m_nPrivateISIndex				= nISIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyXForMounts;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;	
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_bHomingDome = false;
    m_bCalibratingDome = false;
    m_nBattRequest = 0;
    
    m_AMCDrive.setSerxPointer(pSerX);
    m_AMCDrive.setSleeprPinter(pSleeper);
    m_AMCDrive.setLogger(pLogger);

    if (m_pIniUtil)
    {   
        m_AMCDrive.setHomeAz( m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_HOME_AZ, 0) );
        m_AMCDrive.setParkAz( m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_PARK_AZ, 0) );
        m_AMCDrive.setNbTicksPerRev( m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_TICKS_PER_REV, 969840) );
        m_bHasShutterControl = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER_CONTROL, false);
    }

}


X2Dome::~X2Dome()
{
    if(m_bLinked)
        m_AMCDrive.Disconnect();
    
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;

}


int X2Dome::establishLink(void)					
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

    X2MutexLocker ml(GetMutex());
    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);
    nErr = m_AMCDrive.Connect(szPort);
    if(nErr) {
        m_bLinked = false;
        nErr = ERR_COMMOPENING;
    }
    else
        m_bLinked = true;

	return nErr;
}

int X2Dome::terminateLink(void)					
{
    X2MutexLocker ml(GetMutex());
    m_AMCDrive.Disconnect();
	m_bLinked = false;
	return SB_OK;
}

 bool X2Dome::isLinked(void) const				
{
	return m_bLinked;
}


int X2Dome::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    
    return SB_OK;
}

#pragma mark - UI binding

int X2Dome::execModalSettingsDialog()
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    double dHomeAz;
    double dParkAz;
    int nTicksPerRev;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("AMCDrive.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());

    memset(szTmpBuf,0,SERIAL_BUFFER_SIZE);
    // set controls state depending on the connection state
    if(m_bHasShutterControl) {
        dx->setChecked("hasShutterCtrl",true);
    }
    else {
        dx->setChecked("hasShutterCtrl",false);
    }

    if(m_bLinked) {
        dx->setEnabled("pushButton",false); // calibrate  off for now
    }
    else {
        snprintf(szTmpBuf,16,"NA");
        dx->setPropertyString("ticksPerRev","text", szTmpBuf);
        dx->setEnabled("pushButton",false);
        dx->setEnabled("hasShutterCtrl",false);
    }
    nTicksPerRev = m_AMCDrive.getNbTicksPerRev();
    snprintf(szTmpBuf,16,"%d",m_AMCDrive.getNbTicksPerRev());
    dx->setPropertyInt("ticksPerRev","value", nTicksPerRev);
    dx->setPropertyDouble("homePosition","value", m_AMCDrive.getHomeAz());
    dx->setPropertyDouble("parkPosition","value", m_AMCDrive.getParkAz());

    m_bHomingDome = false;
    m_nBattRequest = 0;
    

    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK) {
        dx->propertyDouble("homePosition", "value", dHomeAz);
        dx->propertyDouble("parkPosition", "value", dParkAz);
        dx->propertyInt("ticksPerRev","value", nTicksPerRev);

        m_bHasShutterControl = dx->isChecked("hasShutterCtrl");
        m_AMCDrive.setHomeAz(dHomeAz);
        m_AMCDrive.setParkAz(dParkAz);
        m_AMCDrive.setNbTicksPerRev(nTicksPerRev);

        // save the values to persistent storage
        nErr |= m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_HOME_AZ, dHomeAz);
        nErr |= m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_PARK_AZ, dParkAz);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_TICKS_PER_REV, nTicksPerRev);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_SHUTTER_CONTROL, m_bHasShutterControl);
    }
    return nErr;

}

void X2Dome::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    bool bComplete = false;
    int nErr;
    char szTmpBuf[SERIAL_BUFFER_SIZE];    
    char szErrorMessage[LOG_BUFFER_SIZE];

    if (!strcmp(pszEvent, "on_pushButtonCancel_clicked"))
        m_AMCDrive.abortCurrentCommand();

    if (!strcmp(pszEvent, "on_timer"))
    {
        m_bHasShutterControl = uiex->isChecked("hasShutterCtrl");
        if(m_bLinked) {
            // are we going to Home position to calibrate ?
            if(m_bHomingDome) {
                // are we home ?
                bComplete = false;
                nErr = m_AMCDrive.isFindHomeComplete(bComplete);
                if(nErr) {
                    uiex->setEnabled("pushButton",true);
                    uiex->setEnabled("pushButtonOK",true);
                    snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error homing dome while calibrating dome : Error %d", nErr);
                    uiex->messageBox("AMCDrive Calibrate", szErrorMessage);
                    m_bHomingDome = false;
                    m_bCalibratingDome = false;
                    return;
                }
                if(bComplete) {
                    m_bHomingDome = false;
                    m_bCalibratingDome = true;
                    m_AMCDrive.calibrate();
                    return;
                }
                
            }
            
            if(m_bCalibratingDome) {
                // are we still calibrating ?
                bComplete = false;
                nErr = m_AMCDrive.isCalibratingComplete(bComplete);
                if(nErr) {
                    uiex->setEnabled("pushButton",true);
                    uiex->setEnabled("pushButtonOK",true);
                    snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error calibrating dome : Error %d", nErr);
                    uiex->messageBox("AMCDrive Calibrate", szErrorMessage);
                    m_bHomingDome = false;
                    m_bCalibratingDome = false;
                    return;;
                }
                
                if(!bComplete) {
                    return;
                }
                
                // enable "ok" and "calibrate"
                uiex->setEnabled("pushButton",true);
                uiex->setEnabled("pushButtonOK",true);
                // read step per rev from dome
                snprintf(szTmpBuf,16,"%d",m_AMCDrive.getNbTicksPerRev());
                uiex->setPropertyString("ticksPerRev","text", szTmpBuf);
                m_bCalibratingDome = false;
                
            }
        }
    }

    if (!strcmp(pszEvent, "on_pushButton_clicked"))
    {
        if(m_bLinked) {
            // disable "ok" and "calibrate"
            uiex->setEnabled("pushButton",false);
            uiex->setEnabled("pushButtonOK",false);
            m_AMCDrive.goHome();
            m_bHomingDome = true;
        }
    }
}

//
//HardwareInfoInterface
//
#pragma mark - HardwareInfoInterface

void X2Dome::deviceInfoNameShort(BasicStringInterface& str) const					
{
	str = "A-M-C Drive";
    if(m_bLinked) {
        X2Dome* pMe = (X2Dome*)this;
        char cProdInfo[SERIAL_BUFFER_SIZE];
        pMe->m_AMCDrive.getProductInformationString(cProdInfo, SERIAL_BUFFER_SIZE);
        str = cProdInfo;
    }
}

void X2Dome::deviceInfoNameLong(BasicStringInterface& str) const					
{
    str = "A-M-C Drive";
    if(m_bLinked) {
        X2Dome* pMe = (X2Dome*)this;
        char cProdInfo[SERIAL_BUFFER_SIZE];
        pMe->m_AMCDrive.getProductInformationString(cProdInfo, SERIAL_BUFFER_SIZE);
        str = cProdInfo;
    }
}

void X2Dome::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
    str = "A-M-C Drive";
    if(m_bLinked) {
        X2Dome* pMe = (X2Dome*)this;
        char cProdInfo[SERIAL_BUFFER_SIZE];
        pMe->m_AMCDrive.getProductInformationString(cProdInfo, SERIAL_BUFFER_SIZE);
        str = cProdInfo;
    }
}

 void X2Dome::deviceInfoFirmwareVersion(BasicStringInterface& str)					
{
    if(m_bLinked) {
        char cFirmware[SERIAL_BUFFER_SIZE];
        m_AMCDrive.getFirmwareVersionString(cFirmware, SERIAL_BUFFER_SIZE);
        str = cFirmware;
    }
    else
        str = "";
}

void X2Dome::deviceInfoModel(BasicStringInterface& str)
{
    str = "A-M-C Drive";
    if(m_bLinked) {
        X2Dome* pMe = (X2Dome*)this;
        char cProdInfo[SERIAL_BUFFER_SIZE];
        pMe->m_AMCDrive.getProductInformationString(cProdInfo, SERIAL_BUFFER_SIZE);
        str = cProdInfo;
    }
}

//
//DriverInfoInterface
//
#pragma mark - DriverInfoInterface

 void	X2Dome::driverInfoDetailedInfo(BasicStringInterface& str) const	
{
    str = "A-M-C Drive X2 plugin by Rodolphe Pineau";
}

double	X2Dome::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

//
//DomeDriverInterface
//
#pragma mark - DomeDriverInterface

int X2Dome::dapiGetAzEl(double* pdAz, double* pdEl)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    *pdAz = m_AMCDrive.getCurrentAz();
    *pdEl = m_AMCDrive.getCurrentEl();
    return SB_OK;
}

int X2Dome::dapiGotoAzEl(double dAz, double dEl)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());


    nErr = m_AMCDrive.gotoAzimuth(dAz);
    if(nErr)
        return ERR_CMDFAILED;

    else
        return SB_OK;
}

int X2Dome::dapiAbort(void)
{

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    m_AMCDrive.abortCurrentCommand();

    return SB_OK;
}

int X2Dome::dapiOpen(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());


    if(!m_bHasShutterControl)
        return SB_OK;

    nErr = m_AMCDrive.openShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiClose(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());


    if(!m_bHasShutterControl)
        return SB_OK;

    nErr = m_AMCDrive.closeShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiPark(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    /*
    if(m_bHasShutterControl)
    {
        nErr = AMCDrive.closeShutter();
        if(nErr)
            return ERR_CMDFAILED;
    }
     */
    
    nErr = m_AMCDrive.parkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiUnpark(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    /*
    if(m_bHasShutterControl)
    {
        nErr = AMCDrive.openShutter();
        if(nErr)
            return ERR_CMDFAILED;
    }
     */
    
    nErr = m_AMCDrive.unparkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiFindHome(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_AMCDrive.goHome();
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsGotoComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_AMCDrive.isGoToComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;
    return SB_OK;
}

int X2Dome::dapiIsOpenComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if(!m_bHasShutterControl) {
        *pbComplete = true;
        return SB_OK;
    }

    nErr = m_AMCDrive.isOpenComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int	X2Dome::dapiIsCloseComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if(!m_bHasShutterControl) {
        *pbComplete = true;
        return SB_OK;
    }

    nErr = m_AMCDrive.isCloseComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsParkComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_AMCDrive.isParkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsUnparkComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_AMCDrive.isUnparkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsFindHomeComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_AMCDrive.isFindHomeComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiSync(double dAz, double dEl)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_AMCDrive.syncDome(dAz, dEl);
    if(nErr)
        return ERR_CMDFAILED;
	return SB_OK;
}

//
// SerialPortParams2Interface
//
#pragma mark - SerialPortParams2Interface

void X2Dome::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Dome::setPortName(const char* szPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, szPort);
    
}


void X2Dome::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}



