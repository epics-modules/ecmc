require asyn
require streamdevice
require motor,6.10.3-ESS
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
#
# Notes: The drive needs special startup sequence. 
# Control word needs to be set to:
# 1. 0hex
# 2. 80hex (Fault reset)
# 3. 6hex  (Shutdown)
# 4. 7hex  (Switch on)
# 5. Fhex  (Enable operation)
#
#
ecmcConfigController "MCU1", "Cfg.SetEnableFuncCallDiag(0)"
ecmcConfigController "MCU1", "Cfg.EcSetMaster(0)"

ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,1,2,0x1600,0x7000,0x01,8,CONTROL)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,1,2,0x1600,0x7000,0x02,16,VALUE)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x00,16,ENC_STATUS)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x10,16,POSITION)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x20,16,LATCH_POSITION)"
ecmcConfigController "MCU1", "Cfg.EcSlaveConfigDC(2,0x320,1000000,500000,1000000,500000)"

ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(7,0x2,0x1b7d3052,1,2,0x1602,0x7010,0x1,16,STM_CONTROL)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(7,0x2,0x1b7d3052,1,2,0x1604,0x7010,0x21,16,VELOCITY_SETPOINT)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(7,0x2,0x1b7d3052,2,3,0x1a00,0x6000,0x0,16,ENC_STATUS)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(7,0x2,0x1b7d3052,2,3,0x1a00,0x6000,0x11,16,POSITION)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(7,0x2,0x1b7d3052,2,3,0x1a00,0x6000,0x12,16,LATCH_POSITION)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(7,0x2,0x1b7d3052,2,3,0x1a03,0x6010,0x1,16,STM_STATUS)"
ecmcConfigController "MCU1", "Cfg.EcSlaveConfigDC(7,0x300,1000000,20,1000000,20)"
ecmcConfigController "MCU1", "Cfg.EcAddSdo(7,0x8010,0x1,1000,2)"
ecmcConfigController "MCU1", "Cfg.EcAddSdo(7,0x8012,0x5,1,1)"
#Sync mode 8010:19
ecmcConfigController "MCU1", "Cfg.EcAddSdo(7,0x1C32,0x1,2,2)"
#Cycle time
#ecmcConfigController "MCU1", "Cfg.EcAddSdo(7,0x1C32,0x2,1000000,4)"
#Sync mode 
#ecmcConfigController "MCU1", "Cfg.EcAddSdo(7,0x1C33,0x1,2,2)"
#Cycle time
#ecmcConfigController "MCU1", "Cfg.EcAddSdo(7,0x1C33,0x2,1000000,4)"






#Test Resolver input
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(9,0x2,0x1c213052,1,2,0x1600,0x7010,0x01,16,CONTROL)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(9,0x2,0x1c213052,1,2,0x1601,0x7010,0x06,32,VELOCITY_SETPOINT)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(9,0x2,0x1c213052,2,3,0x1a00,0x6000,0x11,32,POSITION)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(9,0x2,0x1c213052,2,3,0x1a01,0x6010,0x01,16,STATUS)"
#NOTE important the sync 1 period needs to be 0!
ecmcConfigController "MCU1", "Cfg.EcSlaveConfigDC(9,0x700,1000000,500000,0,0)"
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x8011,0x11,1000,4)"
ecmcConfigController "MCU1", "Cfg.EcSlaveConfigWatchDog(9,1000,10000)"

#*************** Distributed clocks config EL7201
#Sync mode 
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x1C32,0x1,3,2)"
#Cycle time
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x1C32,0x2,1000000,4)"
#Sync mode 
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x1C33,0x1,3,2)"
#Cycle time
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x1C33,0x2,1000000,4)"

#Does not work. DC-patch needed...
ecmcConfigController "MCU1", "Cfg.EcSelectReferenceDC(0,7)"

#***************Parmetrization of EL7201
#Nominal voltage = 48V
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x8010,0x19,48000,4)"

#Motor max current = 6A
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x8011,0x11,6000,4)"

#Motor rated current = 4A
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x8011,0x12,4000,4)"

#Motor pole pairs = 3
ecmcConfigController "MCU1", "Cfg.EcAddSdo(9,0x8011,0x13,3,1)"





ecmcConfigController "MCU1", "Cfg.EcApplyConfig(1)"


#Diagnostics
ecmcConfigController "MCU1", "Cfg.SetEnableFuncCallDiag(0)"
ecmcConfigController "MCU1", "Cfg.EcSetDomainFailedCyclesLimit(10)"
ecmcConfigController "MCU1", "Cfg.EcSetDiagnostics(1)"
ecmcConfigController "MCU1", "Cfg.EcEnablePrintouts(0)"
ecmcConfigController "MCU1", "Cfg.SetDiagAxisEnable(1)"
ecmcConfigController "MCU1", "Cfg.SetEnableTimeDiag(0)"

#Enable dirve (except voltage)
#ecmcConfigController "MCU1", "Cfg.WriteEcEntryIDString(9,CONTROL,7)"

#Go to runtime
ecmcConfigController "MCU1", "Cfg.SetAppMode(1)"


#############################################################
#Stream device
epicsEnvSet "P" "$(P=I:)" 
epicsEnvSet "R" "$(R=Test)" 

  #General 
  dbLoadTemplate("ecmcGeneral.substitutions")
  dbLoadTemplate("el7201.substitutions")

#var streamDebug 1

