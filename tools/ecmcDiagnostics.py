#!/usr/bin/python
# coding: utf-8
from ecmcDataStructure import axisData
from datetime import datetime

numberOfAxes=10
filename='test.txt'
print "Welcome to ECMC Diagnostocs Tool. Should never be needed :-).."

dataFile=open(filename,'r')
axis=[]

print "Adding axis objects (" +str(numberOfAxes) + ")."
for ax in range(0, numberOfAxes): 
  axis.append(axisData(ax))

print "Parsing log-file:"
for line in dataFile.readlines():
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
  print "DateTimeString: " + dateTimeString
  variableName=dataString[:equalPos];
  print "Variable Name: " + variableName
  exeString=dataString[:equalPos]+'.setValue('+dataString[equalPos+1:]+",dateTimeString,line[:semiColonPos+1])"
  print "Execute string: " + exeString

  #Execute expression
  exec(exeString)

print''
print "Printing summary of all read data:"  
for ax in axis:
  print ax

dataFile.close()
