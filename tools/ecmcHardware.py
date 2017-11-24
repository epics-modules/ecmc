#!/usr/bin/python
# coding: utf-8
import sys
from os import listdir
from os.path import isfile,join
     

################## ecmcCommand
class ecmcCommand:
  def __init__(self,command):
    self.command=command

  def printCommand(self):
    print self.command

################## ecmcComponent
class ecmcComponent:
  def __init__(self,path,filename):
    self.path=path
    self.filename=filename
    self.ecmcCommands=[]
    f=open(join(self.path,self.filename),'r')
    for line in f:
      line=line.strip()
      if len(line)>0:
        self.ecmcCommands.append(ecmcCommand(line))
    f.close()

  def printEcmcCommands(self):
    for c in self.ecmcCommands:
      c.printCommand()
    return 

  def printInfo(self):
    print "    Path:          " + join(self.path,self.filename)
    print "    Command count: " + str(len(self.ecmcCommands))


################## ecmcEcEntry
class ecmcEcEntry:
  def __init__(self,strEcAddEntryComplete):
    #EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(${ECMC_EC_SLAVE_NUM},0x000001a3,0x019f418d,1,2,0x1600,0x6040,0x0,16,STM_CONTROL)"
    #int ecAddEntryComplete(
    #    uint16_t slaveBusPosition,
    #    uint32_t vendorId,
    #    uint32_t productCode,
    #    int direction,
    #    uint8_t syncMangerIndex,
    #    uint16_t pdoIndex,
    #    uint16_t entryIndex,
    #    uint8_t  entrySubIndex,
    #    uint8_t bits,
    #    char *entryIDString);    
    self.vendorId=""
    self.productId=""
    self.direction=""
    self.smIndex=""
    self.pdoIndex=""
    self.entryIndex=""
    self.entrySubIndex=""
    self.bits=""
    self.alias=""
    self.strEcAddEntryComplete=strEcAddEntryComplete;
    print "Entry added: "+ strEcAddEntryComplete

  def parseEcmcCommandString(self):
    splitCmd=self.strEcAddEntryComplete.split(',');
    if len(splitCmd) < 11:
      return 0
    self.vendorId=splitCmd[2]
    self.productId=splitCmd[3]
    self.direction=splitCmd[4]
    self.smIndex=splitCmd[5]
    self.pdoIndex=splitCmd[6]
    self.entryIndex=splitCmd[7]
    self.entrySubIndex=splitCmd[8]
    self.bits=splitCmd[9]
    self.alias=splitCmd[10]
    return 1

  def getEcmcCommandString(self):
    #EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(${ECMC_EC_SLAVE_NUM},0x000001a3,0x019f418d,1,2,0x1600,0x6040,0x0,16,STM_CONTROL)"
    retStr='EthercatMCConfigController "MCU1", "Cfg.EcAddEntryComplete(${ECMC_EC_SLAVE_NUM},'
    retStr=retStr+ self.vendorId + ',' 
    retStr=retStr+ self.productId + ',' 
    retStr=retStr+ self.direction + ','
    retStr=retStr+ self.smIndex + ',' 
    retStr=retStr+ self.pdoIndex + ',' 
    retStr=retStr+ self.entryIndex + ',' 
    retStr=retStr+ self.entrySubIndex + ',' 
    retStr=retStr+ self.bits + ','
    retStr=retStr+ self.alias + ')"'    
    return retStr


################## ecmcEcSlave
class ecmcEcSlave:
  def __init__(self,path,filename):
    self.ecmcCommands=[]
    self.path=path
    self.filename=filename
    self.components=[]
    self.productId=""
    self.vendorId=""
    self.entries=[]
    print "Added slave: " + filename
    self.readSlaveInfo()
   
  def addComponent(self,compFilename):    
    self.components.append(ecmcComponent(self.path,compFilename))
    print "Slave " + self.filename + ", component added: "+ compFilename

  def getComponetsList(self):
    return self.components

  def readSlaveInfo(self):
    print "reading slave data: " + self.filename
    foundSlaveData=0
    f=open(join(self.path,self.filename),'r')

    for line in f:
      line=line.strip()
      #Cfg.EcAddEntryComplete(${ECMC_EC_SLAVE_NUM},0x000001a3,0x019f418d,1,2,0x1600,0x6040,0x0,16,STM_CONTROL)"
      if line.find('EcAddEntryComplete(')>=0:
        splitLine=line.split(',')
        if len(splitLine)==11:
          if not foundSlaveData:
            self.vendorId=splitLine[2]
            self.productId=splitLine[3]
            foundSlaveData=1

        self.entries.append(ecmcEcEntry(line))

        if not self.entries[-1].parseEcmcCommandString():
          print "Failed parsing EcAddEntryComplete command string."

      #Cfg.EcAddSlave(0,${ECMC_EC_SLAVE_NUM},0x2,0x044c2c52)     
      if line.find('EcAddSlave(')>=0:
        splitLine=line.split(',')
        if len(splitLine)==5:          
          if not foundSlaveData:
            self.vendorId=splitLine[3]
            self.productId=splitLine[4]
            foundSlaveData=1

      if len(line)>0:
        self.ecmcCommands.append(ecmcCommand(line))

    f.close()

  def getComponetsCount(self):
    return len(self.components)

  def getEcmcCommandString(self):
    #EthercatMCConfigController ${ECMC_MOTOR_PORT}, "Cfg.EcAddSlave(0,${ECMC_EC_SLAVE_NUM},0x2,0x044c2c52)"
    retStr='EthercatMCConfigController ${ECMC_MOTOR_PORT}, "Cfg.EcAddSlave(0,${ECMC_EC_SLAVE_NUM},'
    retStr=retStr+ self.vendorId + ',' 
    retStr=retStr+ self.productId + ')"' 
    return retStr

  def printEcmcCommands(self):
    for c in self.ecmcCommands:
      c.printCommand()
    return 

  def printInfo(self):
    print "Path:            " + join(self.path,self.filename)
    print "Product Id:      " + str(self.productId)
    print "Vendor Id:       " + str(self.vendorId)
    print "Entry count:     " + str(len(self.entries))
    print "Command count:   " + str(len(self.ecmcCommands))
    print "Component count: " + str(self.getComponetsCount())
    for c in self.components:
      c.printInfo()
    return


################## ecmcHardware
class ecmcHardware:
  def __init__(self,path):
    self.validEcSlaveConfigFiles=[]
    self.slaveObjects=[]
    self.path=path
    self.checkValidConfigFiles()
    return

  def isEcmcConfigFile(self,filename):     
    if filename.find('.cmd')>=0:
      return 0
    if filename.find('Start')>=0:
      return 0
    if filename.find('Test')>=0:
      return 0
    if filename.find('~')>=0:
      return 0
    if filename.find('ecmc')<0:
      return 0
    return 1

  def isComponent(self,slaveName,compName):
    if(compName.count('-')<=1):
      return 0
    if len(slaveName)==len(compName):
      return 0
    basename=slaveName.split("-")
    if len(basename)<2:
      return 0
    basename=basename[0].strip()
    if compName.find(basename)==-1:
      return 0
    if compName.find('records')>=0:
      return 0
    return 1

  def checkValidConfigFiles(self):
    allFiles=[]
    for f in listdir(self.path):
      if isfile(join(self.path,f)):
        allFiles.append(f)
    allFiles.sort()

    #filter files
    for filename in allFiles:
      if not self.isEcmcConfigFile(filename): 
        continue
      if(filename.count('-')>1):
        continue
      if(filename.count('-')==0):
        continue
      self.validEcSlaveConfigFiles.append(filename)

    self.validEcSlaveConfigFiles.sort()
    
    #Find component files to each hardware
    for slaveName in self.validEcSlaveConfigFiles:
      self.slaveObjects.append(ecmcEcSlave(self.path,slaveName))
      #Find components
      for compName in allFiles:
        if not self.isEcmcConfigFile(compName): 
          continue
        if self.isComponent(slaveName,compName):          
          self.slaveObjects[-1].addComponent(compName)

  def getSlave(self,index):    
    return self.slaveObjects[index]

  def getSlaves(self):    
    return self.slaveObjects
                     
if __name__ == "__main__":
 # Check args
  if len(sys.argv)!=2:
    print "python ecmcHardware.py <path to ecmctraining hardware files>"
    print "example: python ecmcHardware.py ~/projects/ecmctraining/V2/startup/hardware"
    sys.exit()

  dirname=sys.argv[1]
  availableHw=ecmcHardware(dirname)

  #for slave in availableHw.getSlaves():
  # slave.printEcmcCommands()

  for slave in availableHw.getSlaves():
   print "###############################"
   slave.printInfo()

 
    


  

