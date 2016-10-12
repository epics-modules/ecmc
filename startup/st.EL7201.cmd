require asyn
require streamdevice
require motor,6.10.3-ESS
require ecmc,anderssandstrom
#require ecmc,1.1.1


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

ecmcConfigController "MCU1", "Cfg.SetEnableFuncCallDiag(0)"
ecmcConfigController "MCU1", "Cfg.EcSetMaster(0)"

ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,1,2,0x1600,0x7000,0x01,8,CONTROL)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,1,2,0x1600,0x7000,0x02,16,VALUE)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x00,16,ENC_STATUS)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x10,16,POSITION)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x20,16,LATCH_POSITION)"
ecmcConfigController "MCU1", "Cfg.EcSlaveConfigDC(2,0x320,1000000,500000,1000000,500000)"


#Test Resolver input
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(9,0x2,0x1c213052,1,2,0x1600,0x7010,0x01,16,CONTROL)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(9,0x2,0x1c213052,1,2,0x1601,0x7010,0x06,32,VELOCITY_SETPOINT)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(9,0x2,0x1c213052,2,3,0x1a00,0x6000,0x11,32,POSITION)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(9,0x2,0x1c213052,2,3,0x1a01,0x6010,0x01,16,STATUS)"
ecmcConfigController "MCU1", "Cfg.EcSlaveConfigDC(9,0x700,1000000,500000,1000000,500000)"
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x8011,0x11,1000,4)"

#Sync mode 
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x1C32,0x1,3,2)"
#Cycle time
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x1C32,0x2,1000000,4)"
#Sync mode 
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x1C33,0x1,3,2)"
#Cycle time
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x1C33,0x2,1000000,4)"


#end

ecmcConfigController "MCU1", "Cfg.EcSelectReferenceDC(0,9)"

ecmcConfigController "MCU1", "Cfg.EcSetDomainFailedCyclesLimit(10)"
ecmcConfigController "MCU1", "Cfg.EcApplyConfig(1)"
ecmcConfigController "MCU1", "Cfg.EcSetDiagnostics(1)"
ecmcConfigController "MCU1", "Cfg.EcEnablePrintouts(0)"
ecmcConfigController "MCU1", "Cfg.SetDiagAxisEnable(0)"

ecmcConfigController "MCU1", "Cfg.SetEnableTimeDiag(1)"

#Write to outputs in order to power switches
#ecmcConfigController "MCU1", "WriteEcEntry(1,0,1)"


ecmcConfigController "MCU1", "Cfg.CreateAxis(1,3)"
ecmcConfigController "MCU1", "Cfg.SetAxisTrajStartPos(1,0)"
ecmcConfigController "MCU1", "Main.M1.fAcceleration=20"
ecmcConfigController "MCU1", "Main.M1.fDeceleration=40"
ecmcConfigController "MCU1", "Cfg.SetAxisEmergDeceleration(1,1000)"
ecmcConfigController "MCU1", "Main.M1.fVelocity=20"
ecmcConfigController "MCU1", "Main.M1.nCommand=3"
ecmcConfigController "MCU1", "Cfg.SetAxisJogVel(1,100.0)"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#5001,16#D,8,5=-10000"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#5001,16#E,8,5=10000"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#5001,16#B,2,2=0"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#5001,16#C,2,2=0"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(-1,ONE,1,0,0)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(-1,ONE,1,1,0)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(-1,ONE,1,2,0)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonAtTargetTol(1,0.05)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonAtTargetTime(1,100)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonEnableAtTargetMon(1,0)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonPosLagTol(1,1)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonPosLagTime(1,10)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonEnableLagMon(1,0)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonMaxVel(1,1000)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonEnableMaxVel(1,1)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonMaxVelTrajILDelay(1,200)"

#Diagnostics
ecmcConfigController "MCU1", "Cfg.SetEnableFuncCallDiag(0)"
#Go to runtime
ecmcConfigController "MCU1", "Cfg.SetAppMode(1)"


#############################################################



  #General 
  dbLoadTemplate("ecmcGeneral.substitutions")


#var streamDebug 1

