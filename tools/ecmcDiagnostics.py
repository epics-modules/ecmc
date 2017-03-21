#!/usr/bin/python
# coding: utf-8
from ecmcDataStructure import axisData
from datetime import datetime
import sys

def is_number(s):
  try:
      complex(s) # for int, long, float and complex
  except ValueError:
      return False

  return True

################START############
toTime=0
if len(sys.argv)==1 or len(sys.argv)>4:
  print "python ecmcDiagnostics.py <filename> <axisIndex> <toTime>"
  sys.exit() 
if len(sys.argv)==2:
  axisNumber=1
  filename=sys.argv[1]
if len(sys.argv)==3:
  axisNumber=int(sys.argv[2])
  filename=sys.argv[1]
if len(sys.argv)==4:
  axisNumber=int(sys.argv[2])
  filename=sys.argv[1]
  toTime=datetime.strptime(sys.argv[3],"%Y/%m/%d %H:%M:%S.%f")



print "Welcome to ECMC Diagnostocs Tool. Should never be needed :-).."

dataFile=open(filename,'r')
axis=[]
#not so nice
numberOfAxes=3
print "Adding axis objects (" +str(range(1, numberOfAxes)) + ")."
for ax in range(0, numberOfAxes): 
  axis.append(axisData(ax))

print "Parsing log-file:"
for line in dataFile.readlines():
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
  variableName=dataString[:equalPos];
  print "Variable Name: " + variableName

  #Exclude certain variables
  if variableName.find('filter')>=0:
    continue
  valueString=dataString[equalPos+1:].strip()
  print "valueString: " + valueString
  if is_number(valueString):
    #exeString=dataString[:equalPos]+'.setValue(' + valueString + ',dateTimeString,line[:semiColonPos+1])'
    exeString=dataString[:equalPos]+'.setValue(' + valueString + ',dateTimeString)'
  else:
    #exeString=dataString[:equalPos]+'.setValue(stringVariable,dateTimeString,line[:semiColonPos+1])'
    exeString=dataString[:equalPos]+'.setValue(valueString,dateTimeString)'
  print "Execute string: " + exeString
  #Execute expression
  exec(exeString.strip())

print''
print "Printing summary of all read data:"  
for ax in axis:
  print ax

dataFile.close()
