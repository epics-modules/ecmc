from datetime import datetime
import math

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

  def getVariableName(self):
    return self.variableName_

  def valid(self):
    return self.valid_

  def __repr__(self):
    valueStr=str(self.value_) + ";"
    if self.valid_:
      validStr="Valid;".ljust(0)
    else:
      validStr="Not Valid;".ljust(0)   
    return (self.variableName_ + '=' + valueStr).ljust(50) + validStr.ljust(12) + (self.timestampString_ + ';').rjust(30) + str(self.lineNumber).rjust(10) + '\n'
  

class dataTreeItem:
  def __init__(self,*args):
    self.isRoot_=1
    self.name_="root"
    if len(args)==1: #parent
      self.isRoot_=0
      self.parent_==args[0] 
    if len(args)==2: #parent,name
      self.isRoot_=0
      self.parent_==args[0] 
      self.name_=args[1]
    
    self.datapoints_=[]  #For dataItems
    self.children_=[]    #For dataTree children

  #Variable structure for exec access..
  def addVariable(self,name):
    if not (hasattr(self, name)):
      exec("self."+name+"=dataPoint(name)")  
      self.datapoints_.append(name);
      valid_=1
   
  def addChild(self,child):
    for c in children:
       if c.name_ == child_.name:
          return c      
    self.children_.append(child)
    return child

  def getVariables(self):
    return self.datapoints_

  def getChildren(self):
    return self.children_
  
  def getParent(self):
    if self.isRoot:
      return 0
    return self.parent_
