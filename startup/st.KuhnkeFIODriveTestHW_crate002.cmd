require asyn
require streamdevice
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
EthercatMCCreateController("MCU1", "MC_CPU1", "32", "200", "1000", "")

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
EthercatMCConfigController "MCU1", "Cfg.SetEnableFuncCallDiag(0)"
EthercatMCConfigController "MCU1", "Cfg.EcSetMaster(0)"

#Kuhnke drive at slave position 11
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1600,0x6040,0x0,16,STM_CONTROL)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1600,0x607A,0x0,32,TARGET_POSITION)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1600,0x3202,0x0,32,SUB_MODE_SELECT)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1600,0x6060,0x0,8,OP_MODE_CMD)"

EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1603,0x6043,0x0,16,VELOCITY_SETPOINT)"

EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,2,3,0x1a00,0x6041,0x0,16,STM_STATUS)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,2,3,0x1a00,0x6064,0x0,32,POSITION)"
EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,2,3,0x1a00,0x6061,0x0,8,OP_MODE_ACT)"

#Config open loop 
EthercatMCConfigController "MCU1", "Cfg.EcAddSdo(11,0x320A,0x3,-1,4)"

EthercatMCConfigController "MCU1", "Cfg.EcAddSdo(11,0x320A,0x4,-1,4)"

EthercatMCConfigController "MCU1", "Cfg.EcApplyConfig(1)"

#Velocity mode
EthercatMCConfigController "MCU1", "Cfg.WriteEcEntryIDString(11,OP_MODE_CMD,2)"
EthercatMCConfigController "MCU1", "Cfg.WriteEcEntryIDString(11,STM_CONTROL,128)"
EthercatMCConfigController "MCU1", "Cfg.WriteEcEntryIDString(11,SUB_MODE_SELECT,0)"

#Diagnostics
EthercatMCConfigController "MCU1", "Cfg.SetEnableFuncCallDiag(0)"
EthercatMCConfigController "MCU1", "Cfg.EcSetDomainFailedCyclesLimit(10)"
EthercatMCConfigController "MCU1", "Cfg.EcSetDiagnostics(1)"
EthercatMCConfigController "MCU1", "Cfg.EcEnablePrintouts(0)"
EthercatMCConfigController "MCU1", "Cfg.SetDiagAxisEnable(1)"
EthercatMCConfigController "MCU1", "Cfg.SetDiagAxisFreq(1)"
EthercatMCConfigController "MCU1", "Cfg.SetEnableTimeDiag(0)"

#Enable dirve (except voltage)
#EthercatMCConfigController "MCU1", "Cfg.WriteEcEntryIDString(9,CONTROL,7)"

#Go to runtime
EthercatMCConfigController "MCU1", "Cfg.SetAppMode(1)"


#############################################################
#Stream device
epicsEnvSet "P" "$(P=I:)" 
epicsEnvSet "R" "$(R=Test)" 

  #General 
  dbLoadTemplate("ecmcGeneral.substitutions")


  dbLoadRecords("ethercat.db", "S=11,E=STM_STATUS,PORT=MC_CPU1")
  dbLoadRecords("ethercat.db", "S=11,E=STM_CONTROL,PORT=MC_CPU1")

  dbLoadRecords("ethercat.db", "S=11,E=OP_MODE_ACT,PORT=MC_CPU1")
  dbLoadRecords("ethercat.db", "S=11,E=OP_MODE_CMD,PORT=MC_CPU1")

  dbLoadRecords("ethercat.db", "S=11,E=POSITION,PORT=MC_CPU1")
  dbLoadRecords("ethercat.db", "S=11,E=VELOCITY_SETPOINT,PORT=MC_CPU1")


