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

###############################START###############################
toTime=0
missingVariableList=[];

# Check args
if len(sys.argv)==1 or len(sys.argv)>4:
  print "python ecmcDiagnostics.py <filename> <axisIndex> <toTime>"
  print 'example: python ecmcDiagnostics.py xx.txt 2 "2017/03/21 16:07:50.721"'
  sys.exit() 
if len(sys.argv)==2:
  axisIndex=1
  filename=sys.argv[1]
if len(sys.argv)==3:
  axisIndex=int(sys.argv[2])
  filename=sys.argv[1]
if len(sys.argv)==4:
  axisIndex=int(sys.argv[2])
  filename=sys.argv[1]
  toTime=datetime.strptime(sys.argv[3],"%Y/%m/%d %H:%M:%S.%f")

print "Welcome to ECMC Diagnostocs Tool. Should never be needed :-).."

dataFile=open(filename,'r')
axis=axisData(axisIndex)
#not so nice
numberOfAxes=axisIndex+1
print "Adding axis objects (" +str(range(1, numberOfAxes)) + ")."

#for ax in range(0, numberOfAxes): 
#  axis.append(axisData(ax))

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
  if toTime!=0:
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
  if axisIndex!=int(axisIndexInVarName[0]):  
    print "Not data for the axis we are looking for"
    continue
  n=variableName.find(']')  
  if n<0:
    continue
  variableName='axis'+variableName[n+1:]
  print "Variable Name: " + variableName
  
  #Exclude certain variables
  if variableName.find('filter')>=0:
    continue
  valueString=dataString   

  #Exclude certain variables
  if variableName.find('filter')>=0:
    continue
  valueString=dataString[equalPos+1:].strip()
  print "valueString: " + valueString
  if is_number(valueString):
    #exeString=dataString[:equalPos]+'.setValue(' + valueString + ',dateTimeString,line[:semiColonPos+1])'
    #exeString=dataString[:equalPos]+'.setValue(' + valueString + ',dateTimeString)'
    exeString=variableName + '.setValue(' + valueString + ',dateTimeString,'+ str(lineNumber)+')'
  else:
    #exeString=dataString[:equalPos]+'.setValue(stringVariable,dateTimeString,line[:semiColonPos+1])'
    #exeString=dataString[:equalPos]+'.setValue(valueString,dateTimeString)'
    exeString=variableName + '.setValue(valueString,dateTimeString,'+ str(lineNumber)+')'
  print "Execute string: " + exeString
  #Execute expression
  try:
    exec(exeString.strip())
  except SyntaxError:
    print "Failed to execute: " + exeString
  except AttributeError:
    print "Failed to execute: " + exeString
    missingVariableList.append(variableName)

print''
print "Printing summary of data for axis[" +str(axisIndex) + "]:"  
#for ax in axis[axisIndex]:
print axis

if len(missingVariableList)>0:
  print "Printing missing variables in python structures:"  
  #for ax in axis[axisIndex]:
  print missingVariableList


dataFile.close()
