from datetime import datetime
class dataPoint:
  def __init__(self,variableName):
    self.value_=0
    self.timestamp_=0
    self.valid_=0
    self.completeString_=""
    self.variableName_=variableName
    self.timestampString_="No timestamp"
  def setValue(self,*args):
    if len(args)==1:
      self.value_=args[0]
      self.timestamp_=NULL
      self.valid_=1
      self.completeString_=""
    if len(args)==2:
      self.value_=args[0]
      self.timestampstring_=args[1]
      self.timeStamp = datetime.strptime(self.timestampString_, '%y/%m/%d %H:%M:%S.%f')
      self.valid_=1
      self.completeString_=""
    if len(args)==3:
      self.value_=args[0]
      self.timestampString_=args[1]
      self.timeStamp = datetime.strptime(self.timestampString_, '%y/%m/%d %H:%M:%S.%f')
      self.valid_=1
      self.completeString_=args[2]
  @property
  def valid(self):
    return self.valid_
  def __repr__(self):
    valueStr=str(self.value_) + ";"
    if self.valid_:
      validStr="Valid;".ljust(12)
    else:
      validStr="Not Valid;".ljust(12)   
    return (self.variableName_ + '=' + valueStr).ljust(30) + validStr + (self.timestampString_ + ';').ljust(25) + '\n'
  
class monitorData:
  def __init__(self):
    self.atTargetMonEnable=dataPoint('atTargetMonEnable')
    self.posLagMonEnable=dataPoint('posLagMonEnable')
    self.maxVelMonEnable=dataPoint('maxVelMonEnable')
    self.baseString="monitor.".rjust(15)
  def __repr__(self):
    stringToReturn=""
    stringToReturn=stringToReturn + self.baseString + self.atTargetMonEnable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.posLagMonEnable.__repr__()
    stringToReturn=stringToReturn + self.baseString + self.maxVelMonEnable.__repr__()
    return stringToReturn

class axisData:
  def __init__(self,index):
    self.index_ = index
    self.monitor=monitorData()
    self.baseString="axis" + '['+str(self.index_) + ']:' + '\n'
  def __repr__(self):
    return self.baseString + self.monitor.__repr__()

  
