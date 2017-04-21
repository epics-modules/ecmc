#!/usr/bin/env python

from PyQt4 import QtCore, QtGui
from ecmcParser import *

class TreeItem(object):
    def __init__(self, data, parent=None):
        self.parentItem = parent
        self.itemData = data
        self.childItems = []

    def appendChild(self, item):
        self.childItems.append(item)

    def child(self, row):
        return self.childItems[row]

    def childCount(self):
        return len(self.childItems)

    def columnCount(self):
        return len(self.itemData)

    def data(self, column):
        try:
            return self.itemData[column]
        except IndexError:
            return None

    def parent(self):
        return self.parentItem

    def row(self):
        if self.parentItem:
            return self.parentItem.childItems.index(self)

        return 0

    def setData(self, data):
        self.itemData = data


class TreeModel(QtCore.QAbstractItemModel):
    def __init__(self, data, parent=None):
        super(TreeModel, self).__init__(parent)

        self.rootItem = TreeItem(("Title", "Value", "Timestamp"))
        self.setupModelData(data.split('\n'), self.rootItem)

    def columnCount(self, parent):
        if parent.isValid():
            return 3 #parent.internalPointer().columnCount()
        else:
            return 3 #self.rootItem.columnCount()

    def data(self, index, role):
        if not index.isValid():
            return None

        if role != QtCore.Qt.DisplayRole:
            return None

        item = index.internalPointer()

        return item.data(index.column())

    def flags(self, index):
        if not index.isValid():
            return QtCore.Qt.NoItemFlags

        return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable

    def headerData(self, section, orientation, role):
        if orientation == QtCore.Qt.Horizontal and role == QtCore.Qt.DisplayRole:
            return self.rootItem.data(section)

        return None

    def index(self, row, column, parent):
        if not self.hasIndex(row, column, parent):
            return QtCore.QModelIndex()

        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        childItem = parentItem.child(row)
        if childItem:
            return self.createIndex(row, column, childItem)
        else:
            return QtCore.QModelIndex()

    def parent(self, index):
        if not index.isValid():
            return QtCore.QModelIndex()

        childItem = index.internalPointer()
        parentItem = childItem.parent()

        if parentItem == self.rootItem:
            return QtCore.QModelIndex()

        return self.createIndex(parentItem.row(), 0, parentItem)

    def rowCount(self, parent):
        if parent.column() > 0:
            return 0

        if not parent.isValid():        
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        return parentItem.childCount()


    def setupModelData(self, lines, parent=None):
        self.beginResetModel()
        localBaseParent=self.rootItem
        if parent:
          localBaseParent = parent
        indentations = [0]
        self.beginInsertRows
        number = 0
        parser=ecmcParser() 
        while number < len(lines):            
            lineData = lines[number].trimmed()
            print "" 
            print "Parsing line: "+ lineData 

            isValid =parser.lineValid(lineData)
            if not isValid:
              number += 1
              print "InValid (lineValid)"
              continue              
            
            isValid,lineSections=parser.getPathSectionList(lineData)
            if not isValid:
              number += 1
              print "InValid (getPathSectionList)"
              continue              
            
            isValid,value=parser.getValue(lineData)
            if not isValid:
              number += 1
              print "InValid (getValue)"
              continue        

            isValid,timestamp=parser.getTimestampString(lineData)
            if not isValid:
              number += 1
              print "InValid (getTimestampString)"
              continue        
                
            numberLevels=len(lineSections)              
            actLevel=0
            localParent=localBaseParent
            for section in lineSections:                    
              #print "Adding: " + section 
              columnData=[]
              columnData.append(section)
              #print  "numberLevels==actLevel: " + str(numberLevels-1) + "==" + str(actLevel)
              if numberLevels-1==(actLevel):
                 columnData.append(value)
                 columnData.append(timestamp)
              else:
                 columnData.append("")                    

              for col in columnData:
                print "ColumnData: " + str(col)
                    
              #print "Checking if " + section + " already exists!"
              alreadyInTree=0
              childCount=localParent.childCount()
              for row in range(0,childCount):
                if localParent.child(row).data(0)==section:
                  localChild=localParent.child(row);
                  alreadyInTree=1
                  break 

              if alreadyInTree:
                #update data
                if actLevel==numberLevels-1:
                  localChild.setData(columnData)

                actLevel+=1;
		
                localParent=localChild
                continue 
              #print "Adding: " + section 
              localChild=TreeItem(columnData, localParent)
              localParent.appendChild(localChild)
              localParent=localChild
              actLevel+=1;

            number += 1
        print "Reset model!!!!"
        self.endResetModel()


