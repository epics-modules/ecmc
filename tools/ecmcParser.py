#!/usr/bin/python
# coding: utf-8
from datetime import datetime
import sys
import re

def is_number(s):
  try:
    complex(s) # for int, long, float and complex
  except ValueError:
    return False

  return True

class ecmcParser:
  def __init__(self):
    return

  def lineValid(self, line):
    localLine=line[:] 
    pos1=localLine.lastIndexOf(':')
    pos2=localLine.lastIndexOf('=')
    pos3=localLine.lastIndexOf(';')
    if pos1>0 and pos2>0 and pos3>0 and pos3>pos2 and pos2>pos1:
      return 1
    else:
      return 0

  def getPathSectionList(self,line):
    localLine=line[:]
    pos=localLine.lastIndexOf(':')
    if(pos==0):
      return 0 , "" 

    localLine=localLine[pos+1:].trimmed()
    pos=localLine.lastIndexOf('=')
    if(pos==0):
      number += 1
      return 0 , "" 

    localLine=localLine[:pos]
    print "Path: " + localLine
    if len(localLine)>0:                
      lineSections=localLine.split('.')
      return 1, lineSections

    return 0 , "" 

  def getValue(self,line):
    localLine=line[:]
    pos1=localLine.lastIndexOf('=')
    pos2=localLine.lastIndexOf(';')
    if pos1==0 or pos2==0:
      return 0 , "" 
    return 1, localLine[pos1+1:pos2]     
         
  def getTimestampString(self,line):
    localLine=line[:]

    spacePos=localLine.indexOf(' ')
    if spacePos<0:
      return 0 , "" 

    spacePos=localLine.indexOf(' ',spacePos+1)
    if spacePos<0:
      return 0 , "" 

    localLine=localLine[:spacePos]
    #Sanity check of time string %y/%m/%d %H:%M:%S.%f
    if len(localLine)!= len("2017/03/21 16:07:50.588"):
      print "Not valid time (length) " + localLine
      return 0 , "" 

    n=localLine.count('/')
    if n!=2:
      print "Not valid time (/)" + str(n) + " " + localLine
      return 0 , "" 
    n=localLine.count(':')
    if n!=2:
      print "Not valid time (:)" + str(n) + " " + localLine
      return 0 , "" 
    n=localLine.count('.')
    if n!=1:
      print "Not valid time (.)" + str(n) + " " + localLine
      return 0 , "" 
    print "DateTimeString: " + localLine
    return 1 , localLine 

    
  def getTimeFromString(self,line):
      return datetime.strptime(line,"%Y/%m/%d %H:%M:%S.%f") 


 

