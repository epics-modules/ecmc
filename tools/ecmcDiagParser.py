#!/usr/bin/python
# coding: utf-8
from ecmcDataStructure import axisData
from datetime import datetime
import sys
import re

def is_number(s):
  try:
    complex(s) # for int, long, float and complex
  except ValueError:
    return False

  return True

class ecmcDiagParser:
  def __init__(self,maxNumAxis):
    self.toTime=0
    self.missingVariableList=[]
    self.axisIndex=0
    self.filename=""
    self.maxNumAxis=maxNumAxis+1
    self.axis=[]

    for i in range(self.maxNumAxis):
      self.axis.append(axisData(i))

  def setFilename(self,dataFileName):
    self.filename=dataFileName 

  def parse(self):
    dataFile=open(self.filename,'r')
    #self.axis=axisData(self.axisIndex)

    print "Parsing log-file:"
    lineNumber=0;
    
    for line in dataFile.readlines():
      lineNumber=lineNumber+1
      #remove all axisrecord printouts
      n=line.count('[')
      if n>1:
        #print "To many [: " + str(n)
        continue
      colonPos=line.rfind(':');
      if colonPos<0:
        continue
      semiColonPos=line.rfind(';')
      if semiColonPos<0:
        continue
      equalPos=line.rfind('=')
      if equalPos<0:
        continue
      dataString=line[colonPos+1:semiColonPos].strip()
      equalPos=dataString.rfind('=')
      if equalPos<0:
        continue
      spacePos=line[:colonPos].rfind(' ')
      if spacePos<0:
        continue
      dateTimeString=line[:spacePos]

      #Sanity check of time string %y/%m/%d %H:%M:%S.%f
      if len(dateTimeString)!= len("2017/03/21 16:07:50.588"):
        print "Not valid time (length) " + dateTimeString
        continue
      n=dateTimeString.count('/')
      if n!=2:
        print "Not valid time (/)" + str(n) + " " + dateTimeString
        continue
      n=dateTimeString.count(':')
      if n!=2:
        print "Not valid time (:)" + str(n) + " " + dateTimeString
        continue
      n=dateTimeString.count('.')
      if n!=1:
        print "Not valid time (.)" + str(n) + " " + dateTimeString
        continue
      print "DateTimeString: " + dateTimeString
      if self.toTime!=0:
        currTimeStamp = datetime.strptime(dateTimeString,"%Y/%m/%d %H:%M:%S.%f") 
        if currTimeStamp>toTime:
          #exit loop
          break;  
      variableName=dataString[:equalPos].strip();
      print "Variable Name: " + variableName
      n=variableName.find('axis[')
      if n!=0:
        continue
      #get axis number
      axisIndexInVarName=re.findall(r'\d+',variableName)
      print "axisIndexInVarName" + str(axisIndexInVarName)
      if len(axisIndexInVarName)<1:
        print "No numbers in variable name"
        continue
      currAxisIndex=int(axisIndexInVarName[0])
      if currAxisIndex>=self.maxNumAxis or currAxisIndex<0:  
        print "Not data for the axis we are looking for"
        continue
      n=variableName.find(']')  
      if n<0:
        continue
     
      #variableName='self.axis'+ +variableName[n+1:]
      variableName='self.'+variableName
      
      print "Variable Name: " + variableName
  
      #Exclude certain variables
      if variableName.find('filter')>=0:
        continue
     
      valueString=dataString[equalPos+1:].strip()
      print "valueString: " + valueString
      if is_number(valueString):
        exeString=variableName + '.setValue(' + valueString + ',dateTimeString,'+ str(lineNumber)+')'
      else:
        exeString=variableName + '.setValue(valueString,dateTimeString,'+ str(lineNumber)+')'
      print "Execute string: " + exeString
      #Execute expression
      try:
        exec(exeString.strip())
      except SyntaxError:
        print "Failed to execute: " + exeString
      except AttributeError:
        print "Failed to execute: " + exeString
        self.missingVariableList.append(variableName)

    #print''
    #print "Printing summary of data for axis[" +str(self.axisIndex) + "]:"  
    #print self.axis
    return self.axis

    if len(self.missingVariableList)>0:
      print "Printing missing variables in python structures:"  
      #for ax in axis[axisIndex]:
      print self.missingVariableList

    dataFile.close()
