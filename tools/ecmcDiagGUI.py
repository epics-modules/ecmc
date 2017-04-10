#!/usr/bin/python
# coding: utf-8
import sys
from ecmcDiagParser2 import ecmcDiagParser
from LineTextWidget import LineTextWidget
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import Qt

class Main(QtGui.QMainWindow):

    def __init__(self, parent = None):
        QtGui.QMainWindow.__init__(self,parent)
        
        self.filename = ""
        self.initUI()
        #self.diagParser=ecmcDiagParser(10)

    def initToolbar(self):

        self.toolbar = self.addToolBar("Options")
        self.openAction = QtGui.QAction(QtGui.QIcon("icons/open.png"),"Open file",self)
        self.openAction.setStatusTip("Open existing document")
        self.openAction.setShortcut("Ctrl+O")
        self.openAction.triggered.connect(self.open)

        self.toolbar.addAction(self.openAction)
        # Makes the next toolbar appear underneath this one
        self.addToolBarBreak()

    def initMenubar(self):

        menubar = self.menuBar()
        file = menubar.addMenu("File")
        file.addAction(self.openAction)


  
    def initUI(self):

        #self.text = QtGui.QTextEdit(self)
        self.text = LineTextWidget(self)



        #self.setCentralWidget(self.text)

        self.text.getTextEdit().setReadOnly(1)
        self.initToolbar()
        self.initMenubar()

        # Initialize a statusbar for the window
        self.statusbar = self.statusBar()
        # x and y coordinates on the sQWidgetcreen, width, height
        self.setGeometry(100,100,1400,800)

        self.setWindowTitle("ECMC Diagnostics")

        self.treeView = QtGui.QTreeView()
        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        
        self.model = QtGui.QStandardItemModel()
        #self.addItems(self.model, data)
        self.treeView.setModel(self.model)
        
        self.model.setHorizontalHeaderLabels([self.tr("Object")])

	wid = QtGui.QWidget(self)
        self.setCentralWidget(wid)
        layout = QtGui.QHBoxLayout()
        layout.addWidget(self.text)
        layout.addWidget(self.treeView)
        wid.setLayout(layout)

        #Test tree widget
        self.root=QtGui.QStandardItem("root")
        self.model.appendRow(self.root)      
	item=QtGui.QStandardItem("axis[1]")
        self.root.appendRow(item)
	item=QtGui.QStandardItem("event[1]")
        self.root.appendRow(item)
	item=QtGui.QStandardItem("dataRecorder[1]")
        self.root.appendRow(item)
	item=QtGui.QStandardItem("dataStorage[1]")
        self.root.appendRow(item)

        self.model.setColumnCount(2)     
   
        print "Columns: " + str(self.model.columnCount())
        i=self.checkAndAddLeaf("axis[1].monitor.atTargetMonEnable=1")
        i=self.checkAndAddLeaf("axis[1].monitor.atTargetMonEnable2=2")
        i=self.checkAndAddLeaf("axis[1].monitor2.atTargetMonEnable2=3")
        i=self.checkAndAddLeaf("axis[10].monitor2.atTargetMonEnable2=4")
        i=self.checkAndAddLeaf("axis[10].monitor.atTargetMonEnable2=5")
        i=self.checkAndAddLeaf("axis[10].monitor2.atTargetMonEnable2=6")
        i=self.checkAndAddLeaf("axis[10].monitor2.atTargetMonEnable5=7")
        print "Columns: " + str(self.model.columnCount())
     
        #item = QtGui.QStandardItem("Monitor")
        #self.rootItem.appendRow(item)        
        #item = QtGui.QStandardItem("Trajectory")
        #self.rootItem.appendRow(item)        
        #item = QtGui.QStandardItem("Drive")
        #self.rootItem.appendRow(item)        


    def checkAndAddLeaf(self,pathAndValue):        
        parentName="root"
        temp=pathAndValue.split('=')
        if(len(temp)!=2):
          return -1
        value=temp[1] 
        path=temp[0].split('.')
        if(len(path)<2):
          return -2
        column=0
        recursiveItem=self.root
        foundItems=[]
        for i in range(0,len(path)):
          subPath=path[i]
          print "Trying to find: "+ subPath + " in model column: " +str(column) +'.'
          foundItems=self.model.findItems(subPath,Qt.MatchFixedString | Qt.MatchRecursive)
          print foundItems
          found=0
          for matchingItem in foundItems:
            if(parentName==matchingItem.parent().text()):
              recursiveItem=foundItems[0]  
              found=1
              break;
          if(len(foundItems)==0) or not found: 
            print subPath + " not found in model."
            rows=[]            
            row=QtGui.QStandardItem(subPath)
            rows.append(row)    
            if(i==len(path)-1):
              row=QtGui.QStandardItem(value)
              rows.append(row)    
            recursiveItem.appendRow(rows)
            recursiveItem=rows[0] 
          else:
            parentName=recursiveItem.text()  



        #for row in self.model.rows:
        #  print child.text
        #return 1

    def add_item(self,parent,text):
        
        if len(text) > 0: 
          item = QtGui.QStandardItem(text)
          parent.appendRow(item)
          return item
        return NULL
         

    def open(self):

        # Get filename and show only .writer files
        self.filename = QtGui.QFileDialog.getOpenFileName(self, 'Open File',".","(*)")

        if self.filename:
            with open(self.filename,"rt") as file:
                self.text.getTextEdit().setText(file.read())

        #self.diagParser.setFilename(self.filename)
        #parsedData=self.diagParser.parse()

        self.model.setColumnCount(2)        
        if(len(parsedData)<=0):
          return
        root=QtGui.QStandardItem('Axes')
        self.model.appendRow(root)          

        #for axis in parsedData:
        #  parent=root
        #  if(axis.valid()):
        #    axis.addVariable('blaffs')
        #    print "Blaffs valid:" + str(axis.blaffs.valid_)
        #    parent=self.add_item(parent,'axis['+ str(axis.index_) + ']')               
        #    row=[]
        #    for element in axis.getDataList():
        #      item = QtGui.QStandardItem(element.getVariableName())
        #      row.append(item)
        #      parent.appendRow(row)
          

def main():

    app = QtGui.QApplication(sys.argv)

    main = Main()
    main.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
