require asyn,4.27
require axis,10.0.1
require ecmc


## Configure devices
drvAsynECMCPortConfigure("MC_CPU1", 0, 0, 0)
asynOctetSetOutputEos("MC_CPU1", -1, ";\n")
asynOctetSetInputEos("MC_CPU1", -1, ";\n")

#asynSetTraceMask("MC_CPU1", -1, 0xFF)
##asynSetTraceMask("MC_CPU1", -1, 0x48)
asynSetTraceMask("MC_CPU1", -1, 0x41)

asynSetTraceIOMask("MC_CPU1", -1, 2)
asynSetTraceIOMask("MC_CPU1", -1, 6)

asynSetTraceInfoMask("MC_CPU1", -1, 15)

#-----------------------
ecmcCreateController("MCU1", "MC_CPU1", "32", "200", "1000", "")

############################################################

EthercatMCConfigController "MCU1", "Cfg.SetEnableFuncCallDiag(0)"
EthercatMCConfigController "MCU1", "Cfg.EcSetMaster(0)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(0,0x2,0x03fa3052,2,0,0x1a00,0x6000,0x1,1,INPUT_0)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(0,0x2,0x03fa3052,2,0,0x1a01,0x6010,0x1,1,INPUT_1)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(0,0x2,0x03fa3052,2,0,0x1a02,0x6020,0x1,1,INPUT_2)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(0,0x2,0x03fa3052,2,0,0x1a03,0x6030,0x1,1,INPUT_3)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(0,0x2,0x03fa3052,2,0,0x1a04,0x6040,0x1,1,INPUT_4)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(0,0x2,0x03fa3052,2,0,0x1a05,0x6050,0x1,1,INPUT_5)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(0,0x2,0x03fa3052,2,0,0x1a06,0x6060,0x1,1,INPUT_6)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(0,0x2,0x03fa3052,2,0,0x1a07,0x6070,0x1,1,INPUT_7)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(1,0x2,0x0af83052,1,0,0x1600,0x7000,0x1,1,OUTPUT_0)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(1,0x2,0x0af83052,1,0,0x1601,0x7010,0x1,1,OUTPUT_1)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(1,0x2,0x0af83052,1,0,0x1602,0x7020,0x1,1,OUTPUT_2)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(1,0x2,0x0af83052,1,0,0x1603,0x7030,0x1,1,OUTPUT_3)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(1,0x2,0x0af83052,1,0,0x1604,0x7040,0x1,1,OUTPUT_4)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(1,0x2,0x0af83052,1,0,0x1605,0x7050,0x1,1,OUTPUT_5)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(1,0x2,0x0af83052,1,0,0x1606,0x7060,0x1,1,OUTPUT_6)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(1,0x2,0x0af83052,1,0,0x1607,0x7070,0x1,1,OUTPUT_7)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,1,2,0x1600,0x7000,0x01,8,CONTROL)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,1,2,0x1600,0x7000,0x02,16,VALUE)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x00,16,STATUS)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x10,16,POSITION)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x20,16,LATCH_POSITION)"

EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(6,0x2,0x1b7d3052,1,2,0x1602,0x7010,0x1,16,STM_CONTROL)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(6,0x2,0x1b7d3052,1,2,0x1604,0x7010,0x21,16,VELOCITY_SETPOINT)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(6,0x2,0x1b7d3052,2,3,0x1a00,0x6000,0x0,16,ENC_STATUS)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(6,0x2,0x1b7d3052,2,3,0x1a00,0x6000,0x11,16,POSITION)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(6,0x2,0x1b7d3052,2,3,0x1a00,0x6000,0x12,16,LATCH_POSITION)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(6,0x2,0x1b7d3052,2,3,0x1a03,0x6010,0x1,16,STM_STATUS)"
#EthercatMCConfigController "MCU1", "Cfg.EcSlaveConfigDC(6,0x300,1000000,20,1000000,20)"
EthercatMCConfigController "MCU1", "Cfg.EcAddSdo(6,0x8010,0x1,1000,2)"
EthercatMCConfigController "MCU1", "Cfg.EcAddSdo(6,0x8012,0x5,1,1)"

EthercatMCConfigController "MCU1", "Cfg.EcApplyConfig(1)"
EthercatMCConfigController "MCU1", "WriteEcEntry(1,1,1)"
EthercatMCConfigController "MCU1", "WriteEcEntry(1,2,1)"
EthercatMCConfigController "MCU1", "WriteEcEntry(1,5,1)"

EthercatMCConfigController "MCU1", "Cfg.CreateDefaultAxis(1)"
EthercatMCConfigController "MCU1", "Main.M1.fAcceleration=20"
EthercatMCConfigController "MCU1", "Main.M1.fDeceleration=20"
EthercatMCConfigController "MCU1", "Cfg.SetAxisEmergDeceleration(1,200)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisCntrlKp(1,15.0)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisCntrlKi(1,0.02)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisCntrlKd(1,0)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisCntrlKff(1,26.3)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisCntrlOutHL(1,8000.0)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisCntrlOutLL(1,-8000.0)"
EthercatMCConfigController "MCU1", "Cfg.LinkEcEntryToAxisEncoder(2,POSITION,1,0,-1)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisEncScaleDenom(1,2000)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisEncScaleNum(1,-60)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisEncType(1,0)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisEncBits(1,16)"
EthercatMCConfigController "MCU1", "Cfg.LinkEcEntryToAxisDrive(6,STM_CONTROL,1,0,0)"
EthercatMCConfigController "MCU1", "Cfg.LinkEcEntryToAxisDrive(6,VELOCITY_SETPOINT,1,1,-1)"
EthercatMCConfigController "MCU1", "Cfg.LinkEcEntryToAxisDrive(6,STM_STATUS,1,2,1)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisDrvScaleDenom(1,32768.0)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisDrvScaleNum(1,16000.0)"
EthercatMCConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(0,INPUT_1,1,0,0)"
EthercatMCConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(0,INPUT_3,1,1,0)"
EthercatMCConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(0,INPUT_2,1,2,0)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonAtTargetTol(1,0.1)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonAtTargetTime(1,100)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonEnableAtTargetMon(1,1)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonPosLagTol(1,0.5)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonPosLagTime(1,10)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonEnableLagMon(1,0)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonMaxVel(1,100)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonEnableMaxVel(1,1)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonMaxVelDriveILDelay(1,100)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisMonMaxVelTrajILDelay(1,200)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisHomeVelTwordsCam(1,10)"
EthercatMCConfigController "MCU1", "Cfg.SetAxisHomeVelOffCam(1,5)"
EthercatMCConfigController "MCU1", "Cfg.EcSetDiagnostics(0)"
EthercatMCConfigController "MCU1", "Cfg.SetDiagAxisIndex(1)"
EthercatMCConfigController "MCU1", "Cfg.SetDiagAxisFreq(1)"
EthercatMCConfigController "MCU1", "Cfg.EcEnablePrintouts(0)"
EthercatMCConfigController "MCU1", "Cfg.EcSetDomainFailedCyclesLimit(100)"
EthercatMCConfigController "MCU1", "Cfg.SetDiagAxisEnable(0)"


#Go to runtime
EthercatMCConfigController "MCU1", "Cfg.SetAppMode(1)"

# load records
dbLoadTemplate("SolAxis.substitutions")
dbLoadTemplate("SolAxis-extra.substitutions")


#############################################################

#Parameter 3 ecmcCreateAxis
#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_WHEN_HOMING  (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

ecmcCreateAxis("MCU1", "1", "6", "")
