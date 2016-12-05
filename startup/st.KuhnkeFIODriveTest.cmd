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

#Kuhnke drive at slave position 11
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1600,0x6040,0x0,16,STM_CONTROL)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1600,0x607A,0x0,32,TARGET_POSITION)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1600,0x3202,0x0,32,SUB_MODE_SELECT)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1600,0x6060,0x0,8,OP_MODE_CMD)"

ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,1,2,0x1603,0x6043,0x0,16,VELOCITY_SETPOINT)"

ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,2,3,0x1a00,0x6041,0x0,16,STM_STATUS)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,2,3,0x1a00,0x6064,0x0,32,POSITION)"
ecmcConfigController "MCU1", "Cfg.EcAddEntryComplete(11,0x0048554b,0x0002ba67,2,3,0x1a00,0x6061,0x0,8,OP_MODE_ACT)"

#Config open loop (expose counter to ethercat bus "Display")
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x320A,0x3,-1,4)"
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x320A,0x4,-1,4)"

#Open loops steper encoder resolution (51200)
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x608F,0x1,51200,4)"
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x608F,0x2,1,4)"

#Motor Pole pair count (50)
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x2030,0x0,50,4)"

#Maximal current (3000mA)
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x2031,0x0,3000,4)"

#Over voltage limit (51000mV)
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x2034,0x0,51000,4)"

#Under voltage limit (45000mV)
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x2035,0x0,45000,4)"

#Open loop current reduction at standstill (reduce by 70%=>-70)
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x2037,0x0,-70,4)"

#Open loop current reduction at standstill idele time (100ms at stanstill then reduction)
ecmcConfigController "MCU1", "Cfg.EcAddSdo(11,0x2036,0x0,100,4)"



ecmcConfigController "MCU1", "Cfg.EcApplyConfig(1)"

#Velocity mode
ecmcConfigController "MCU1", "Cfg.WriteEcEntryIDString(11,OP_MODE_CMD,2)"
ecmcConfigController "MCU1", "Cfg.WriteEcEntryIDString(11,STM_CONTROL,128)"
ecmcConfigController "MCU1", "Cfg.WriteEcEntryIDString(11,SUB_MODE_SELECT,0)"

ecmcConfigController "MCU1", "Cfg.CreateDefaultAxis(1)"
#Set DS402 drive type (must be done right after Create Axis otherwise settings can be lost since new drive object will be created)
ecmcConfigController "MCU1", "Cfg.SetAxisDrvType(1,1)"

ecmcConfigController "MCU1", "Main.M1.fAcceleration=1000"
ecmcConfigController "MCU1", "Main.M1.fDeceleration=1000"
ecmcConfigController "MCU1", "Cfg.SetAxisEmergDeceleration(1,200)"
ecmcConfigController "MCU1", "Main.M1.fVelocity=100"
ecmcConfigController "MCU1", "Main.M1.nCommand=1"
ecmcConfigController "MCU1", "Cfg.SetAxisJogVel(1,100.0)"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#5001,16#D,8,5=-5000"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#5001,16#E,8,5=5000"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#5001,16#B,2,2=0"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#5001,16#C,2,2=0"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#4001,16#6,8,5=10"
ecmcConfigController "MCU1", "ADSPORT=501/.ADR.16#4001,16#7,8,5=5"
ecmcConfigController "MCU1", "Cfg.SetAxisCntrlKp(1,0.8)"
ecmcConfigController "MCU1", "Cfg.SetAxisCntrlKi(1,0.001)"
ecmcConfigController "MCU1", "Cfg.SetAxisCntrlKd(1,0)"
ecmcConfigController "MCU1", "Cfg.SetAxisCntrlKff(1,1)"
ecmcConfigController "MCU1", "Cfg.SetAxisCntrlOutHL(1,8000.0)"
ecmcConfigController "MCU1", "Cfg.SetAxisCntrlOutLL(1,-8000.0)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisEncoder(11,POSITION,1,0,-1)"
ecmcConfigController "MCU1", "Cfg.SetAxisEncScaleDenom(1,51200)"
ecmcConfigController "MCU1", "Cfg.SetAxisEncScaleNum(1,360)"
ecmcConfigController "MCU1", "Cfg.SetAxisEncType(1,0)"
ecmcConfigController "MCU1", "Cfg.SetAxisEncBits(1,32)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisDrive(11,STM_CONTROL,1,0,-1)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisDrive(11,VELOCITY_SETPOINT,1,1,-1)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisDrive(11,STM_STATUS,1,2,-1)"
ecmcConfigController "MCU1", "Cfg.SetAxisDrvScaleDenom(1,1.0)"
ecmcConfigController "MCU1", "Cfg.SetAxisDrvScaleNum(1,1.0)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(-1,ONE,1,0,0)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(-1,ONE,1,1,0)"
ecmcConfigController "MCU1", "Cfg.LinkEcEntryToAxisMonitor(-1,ONE,1,2,0)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonAtTargetTol(1,0.1)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonAtTargetTime(1,100)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonEnableAtTargetMon(1,1)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonPosLagTol(1,0.5)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonPosLagTime(1,10)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonEnableLagMon(1,0)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonMaxVel(1,8000)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonEnableMaxVel(1,1)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonMaxVelDriveILDelay(1,100)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonMaxVelTrajILDelay(1,200)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonEnableCntrlOutHLMon(1,0)"
ecmcConfigController "MCU1", "Cfg.SetAxisMonCntrlOutHL(1,7500)"

#Diagnostics
ecmcConfigController "MCU1", "Cfg.SetEnableFuncCallDiag(0)"
ecmcConfigController "MCU1", "Cfg.EcSetDomainFailedCyclesLimit(10)"
ecmcConfigController "MCU1", "Cfg.EcSetDiagnostics(1)"
ecmcConfigController "MCU1", "Cfg.EcEnablePrintouts(0)"
ecmcConfigController "MCU1", "Cfg.SetDiagAxisEnable(0)"
ecmcConfigController "MCU1", "Cfg.SetDiagAxisFreq(1)"
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

  #First motion axis: Virtual slit position	
  dbLoadRecords("DUT_AxisStatus_v0_01.db", "P=$(P),R=$(R),PORT=MC_CPU1,A=0,P=,VP=Main.M,VI=1,VN=stAxisStatus")
  dbLoadRecords("FB_DriveVirtual_v1_01.db", "P=$(P),R=$(R),PORT=MC_CPU1,A=0,P=,VP=Main.M,VI=1,VN=")
  dbLoadRecords("expression.db", "P=$(P),R=$(R),PORT=MC_CPU1,A=0,Index=1")

  dbLoadRecords("ethercat.db", "S=11,E=STM_STATUS,PORT=MC_CPU1")
  dbLoadRecords("ethercat.db", "S=11,E=STM_CONTROL,PORT=MC_CPU1")

  dbLoadRecords("ethercat.db", "S=11,E=OP_MODE_ACT,PORT=MC_CPU1")
  dbLoadRecords("ethercat.db", "S=11,E=OP_MODE_CMD,PORT=MC_CPU1")

#var streamDebug 1

