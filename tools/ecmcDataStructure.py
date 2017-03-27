from datetime import datetime
class dataPoint:
  def __init__(self,variableName):
    self.value_=0
    self.timestamp_=0
    self.valid_=0
    self.completeString_=""
    self.variableName_=variableName
    self.timestampString_="No timestamp"
    self.lineNumber=0
  def setValue(self,*args):
    if len(args)==1:
      self.value_=args[0]
      self.timestamp_=NULL
      self.valid_=1
      self.completeString_=""
    if len(args)==2:
      self.value_=args[0]
      self.timestampString_=args[1].strip()
      self.timeStamp_ = datetime.strptime(self.timestampString_,"%Y/%m/%d %H:%M:%S.%f")
      self.valid_=1
      self.completeString_=""
    if len(args)==3:
      self.value_=args[0]
      self.timestampString_=args[1].strip()
      self.timeStamp_ = datetime.strptime(self.timestampString_,"%Y/%m/%d %H:%M:%S.%f")
      self.valid_=1
      self.lineNumber=args[2]
      #self.completeString_=args[2]

  @property
  def valid(self):
    return self.valid_
  def __repr__(self):
    valueStr=str(self.value_) + ";"
    if self.valid_:
      validStr="Valid;".ljust(0)
    else:
      validStr="Not Valid;".ljust(0)   
    return (self.variableName_ + '=' + valueStr).ljust(50) + validStr.ljust(12) + (self.timestampString_ + ';').rjust(30) + str(self.lineNumber).rjust(10) + '\n'
  
class monitorData:
  def __init__(self):
    self.baseString="monitor.".rjust(20)
    self.error = dataPoint('error')
    self.enable=dataPoint('enable')
    self.atTargetMonEnable=dataPoint('atTargetMonEnable')
    self.atTargetTolerance=dataPoint('atTargetTolerance')
    self.atTargetTime=dataPoint('atTargetTime')
    self.atTarget=dataPoint('atTarget')
    self.posLagMonEnable=dataPoint('posLagMonEnable')
    self.posLagTol=dataPoint('posLagTol')
    self.posLagTime=dataPoint('posLagTime')
    self.maxVelMonEnable=dataPoint('maxVelMonEnable')
    self.maxVel=dataPoint('maxVel')
    self.maxVelDriveTime=dataPoint('maxVelDriveTime')
    self.maxVelTrajTime=dataPoint('maxVelTrajTime')
    self.velDiffMonEnable=dataPoint('velDiffMonEnable')
    self.velDiffMax=dataPoint('velDiffMax')
    self.velDiffTimeTraj=dataPoint('velDiffTimeTraj')
    self.velDiffTimeDrive=dataPoint('velDiffTimeDrive')
    self.cntrlHLMonEnable=dataPoint('cntrlHLMonEnable')
    self.cntrlOutputHL=dataPoint('cntrlOutputHL')
    self.limitBwd=dataPoint('limitBwd')
    self.limitFwd=dataPoint('limitFwd')
    self.enableAlarmAtHardlimitBwd=dataPoint('enableAlarmAtHardlimitBwd')
    self.enableAlarmAtHardlimitFwd=dataPoint('enableAlarmAtHardlimitFwd')
    self.enableSoftLimitBwd=dataPoint('enableSoftLimitBwd')
    self.enableSoftLimitFwd=dataPoint('enableSoftLimitFwd')
    self.softLimitBwd=dataPoint('softLimitBwd')
    self.softLimitFwd=dataPoint('softLimitFwd')
    self.homeSwitch=dataPoint('homeSwitch')
    self.interlockStatus=dataPoint('interlockStatus')
    self.enableHardwareInterlock=dataPoint('enableHardwareInterlock')
    self.reset=dataPoint('reset')
    self.mySelf=dataPoint('self') 

  def setValue(self,*args):
    self.mySelf.setValue(*args)     

  def __repr__(self):
    stringToReturn=""
    stringToReturn=stringToReturn + self.baseString + self.error.__repr__() 
    stringToReturn=stringToReturn + self.baseString + self.enable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.atTargetMonEnable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.atTargetTolerance.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.atTargetTime.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.atTarget.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.posLagMonEnable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.posLagTol.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.posLagTime.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.maxVelMonEnable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.maxVel.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.maxVelTrajTime.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.maxVelDriveTime.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.velDiffMonEnable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.velDiffMax.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.velDiffTimeTraj.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.velDiffTimeDrive.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.cntrlHLMonEnable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.cntrlOutputHL.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.limitBwd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.limitFwd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.enableAlarmAtHardlimitBwd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.enableAlarmAtHardlimitFwd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.enableSoftLimitBwd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.enableSoftLimitFwd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.softLimitBwd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.softLimitFwd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.homeSwitch.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.interlockStatus.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.enableHardwareInterlock.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.reset.__repr__()
    return stringToReturn

class trajectoryData:
  def __init__(self):
    self.baseString="trajectory.".rjust(20)
    self.error = dataPoint('error')
    self.motionMode=dataPoint('motionMode')
    self.targetPosition=dataPoint('targetPosition')
    self.targetVelocity=dataPoint('targetVelocity')
    self.currentPositionSetpoint=dataPoint('currentPositionSetpoint')
    self.acceleration=dataPoint('acceleration')
    self.deceleration=dataPoint('deceleration')
    self.jerk=dataPoint('jerk')
    self.emergencyDeceleration=dataPoint('emergencyDeceleration')
    self.stepACC=dataPoint('stepACC')
    self.stepDEC=dataPoint('stepDEC')
    self.stepNOM=dataPoint('stepNOM')
    self.stepDECEmerg=dataPoint('stepDECEmerg')
    self.mySelf=dataPoint('self') 

  def setValue(self,*args):
    self.mySelf.setValue(*args)     

  def __repr__(self):
    stringToReturn=""
    stringToReturn=stringToReturn + self.baseString + self.error.__repr__() 
    stringToReturn=stringToReturn + self.baseString + self.motionMode.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.targetPosition.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.targetVelocity.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.currentPositionSetpoint.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.acceleration.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.deceleration.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.jerk.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.emergencyDeceleration.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.stepACC.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.stepDEC.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.stepNOM.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.stepDECEmerg.__repr__()
    return stringToReturn

class encoderData:
  def __init__(self):
    self.baseString="encoder.".rjust(20)
    self.error = dataPoint('error')
    self.enable=dataPoint('enable')
    self.scaleNum=dataPoint('scaleNum')
    self.scaleDenom=dataPoint('scaleDenom')
    self.bits=dataPoint('bits')
    self.actPos=dataPoint('actPos')
    self.homed=dataPoint('homed')
    self.mySelf=dataPoint('self') 

  def setValue(self,*args):
    self.mySelf.setValue(*args)     

  def __repr__(self):
    stringToReturn=""
    stringToReturn=stringToReturn + self.baseString + self.error.__repr__() 
    stringToReturn=stringToReturn + self.baseString + self.enable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.scaleNum.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.scaleDenom.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.bits.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.actPos.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.homed.__repr__()
    return stringToReturn

class controllerData:
  def __init__(self):
    self.baseString="controller.".rjust(20)
    self.error = dataPoint('error')
    self.kp=dataPoint('kp')
    self.ki=dataPoint('ki')
    self.kd=dataPoint('kd')
    self.kff=dataPoint('kff')
    self.outputMin=dataPoint('outputMin')
    self.outputMax=dataPoint('outputMax')
    self.mySelf=dataPoint('self') 

  def setValue(self,*args):
    self.mySelf.setValue(*args)     

  def __repr__(self):
    stringToReturn=""
    stringToReturn=stringToReturn + self.baseString + self.error.__repr__() 
    stringToReturn=stringToReturn + self.baseString + self.kp.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.ki.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.kd.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.kff.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.outputMin.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.outputMax.__repr__()
    return stringToReturn


class sequencerData:
  def __init__(self):
    self.baseString="sequencer.".rjust(20)
    self.error = dataPoint('error')
    self.enable=dataPoint('enable')
    self.execute=dataPoint('execute')
    self.command=dataPoint('command')
    self.cmdData=dataPoint('cmdData')
    self.positionTarget=dataPoint('positionTarget')
    self.velocityTarget=dataPoint('velocityTarget')
    self.inProgress=dataPoint('inProgress')
    self.state=dataPoint('state')
    self.homeVelOffCam=dataPoint('homeVelOffCam')
    self.homeVelTwordsCam=dataPoint('homeVelTwordsCam')
    self.homePosition=dataPoint('homePosition')
    self.mySelf=dataPoint('self') 

  def setValue(self,*args):
    self.mySelf.setValue(*args)     

  def __repr__(self):
    stringToReturn=""
    stringToReturn=stringToReturn + self.baseString + self.error.__repr__() 
    stringToReturn=stringToReturn + self.baseString + self.enable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.execute.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.command.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.cmdData.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.positionTarget.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.velocityTarget.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.inProgress.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.state.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.homeVelOffCam.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.homeVelTwordsCam.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.homePosition.__repr__()
    return stringToReturn

class masterSlaveIFData:
  def __init__(self):
    self.baseString="masterSlaveIF.".rjust(20)
    self.error = dataPoint('error')
    self.interfaceType=dataPoint('interfaceType')
    self.mySelf=dataPoint('self') 

  def setValue(self,*args):
    self.mySelf.setValue(*args)     

  def __repr__(self):
    stringToReturn=""
    stringToReturn=stringToReturn + self.baseString + self.error.__repr__() 
    stringToReturn=stringToReturn + self.baseString + self.interfaceType.__repr__()
    return stringToReturn

class driveData:
  def __init__(self):
    self.baseString="drive.".rjust(20)
    self.error = dataPoint('error')
    self.type=dataPoint('type')
    self.scaleNum=dataPoint('scaleNum')
    self.scaleDenom=dataPoint('scaleDenom')
    self.enabled=dataPoint('enabled')        
    self.mySelf=dataPoint('self') 

  def setValue(self,*args):
    self.mySelf.setValue(*args)     

  def __repr__(self):
    stringToReturn=""
    stringToReturn=stringToReturn + self.baseString + self.error.__repr__() 
    stringToReturn=stringToReturn + self.baseString + self.type.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.scaleNum.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.scaleDenom.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.enabled.__repr__()
    return stringToReturn

class axisData:
  def __init__(self,index):
    self.index_ = index
    self.error = dataPoint('error')
    self.sampleTime = dataPoint('sampleTime')
    self.type = dataPoint('type')
    self.state = dataPoint('state')
    self.enable = dataPoint('enable')
    self.enabled = dataPoint('enabled')
    self.busy = dataPoint('busy')
    self.moving = dataPoint('moving')
    self.driveType = dataPoint('driveType')
    self.inStartupPhase = dataPoint('inStartupPhase')
    self.inRealtime = dataPoint('inRealtime')
    self.reset = dataPoint('reset')
    self.monitor=monitorData()
    self.trajectory=trajectoryData()
    self.encoder=encoderData()
    self.sequencer=sequencerData()
    self.controller=controllerData()
    self.drive=driveData()
    self.masterSlaveIF=masterSlaveIFData()
    self.baseString="axis" + '['+str(self.index_) + ']:' + '\n'
    self.mySelf=dataPoint('self') 

  def setValue(self,*args):
    self.mySelf.setValue(*args)     

  def __repr__(self):
    stringToReturn=self.baseString
    stringToReturn=stringToReturn + self.error.__repr__() 
    stringToReturn=stringToReturn + self.sampleTime.__repr__() 
    stringToReturn=stringToReturn + self.type.__repr__() 
    stringToReturn=stringToReturn + self.state.__repr__() 
    stringToReturn=stringToReturn + self.enable.__repr__() 
    stringToReturn=stringToReturn + self.enabled.__repr__() 
    stringToReturn=stringToReturn + self.busy.__repr__() 
    stringToReturn=stringToReturn + self.moving.__repr__() 
    stringToReturn=stringToReturn + self.driveType.__repr__()
    stringToReturn=stringToReturn + self.inStartupPhase.__repr__()  
    stringToReturn=stringToReturn + self.inRealtime.__repr__()  
    stringToReturn=stringToReturn + self.reset.__repr__()  
    stringToReturn=stringToReturn + self.monitor.__repr__() 
    stringToReturn=stringToReturn + self.trajectory.__repr__() 
    stringToReturn=stringToReturn + self.encoder.__repr__()
    stringToReturn=stringToReturn + self.controller.__repr__()
    stringToReturn=stringToReturn + self.drive.__repr__()
    stringToReturn=stringToReturn + self.sequencer.__repr__()  
    stringToReturn=stringToReturn + self.masterSlaveIF.__repr__() 
    return stringToReturn

  
