#!/usr/bin/python
# coding: utf-8
import sys
#from ecmcDiagParser2 import ecmcDiagParser
from LineTextWidget import LineTextWidget
from ecmcTreeModel import *
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import Qt

class Main(QtGui.QMainWindow):

    def __init__(self, parent = None):
        QtGui.QMainWindow.__init__(self,parent)
        self.data=[]        
        self.filename = ""
        self.initUI()

        #self.diagParser=ecmcDiagParser(10)

    def initToolbar(self):

        self.toolbar = self.addToolBar("Options")
        #self.openAction = QtGui.QAction(QtGui.QIcon("icons/open.png"),"Open file",self)
        self.openAction = QtGui.QAction(QtGui.QIcon.fromTheme("open"),"Open file",self)
        self.openAction.setStatusTip("Open existing document")
        self.openAction.setShortcut("Ctrl+O")
        self.openAction.triggered.connect(self.open)
        self.toolbar.addAction(self.openAction)

        self.refreshAction = QtGui.QAction(QtGui.QIcon.fromTheme("refresh"),"Refersh",self)
        self.refreshAction.setStatusTip("Refresh tree view")
        self.refreshAction.setShortcut("Ctrl+R")
        self.refreshAction.triggered.connect(self.refreshTreeviewCallback)
        self.toolbar.addAction(self.refreshAction)


        # Makes the next toolbar appear underneath this one
        self.addToolBarBreak()

    def initMenubar(self):

        menubar = self.menuBar()
        file = menubar.addMenu("File")
        file.addAction(self.openAction)
        file.addAction(self.refreshAction)

    def initUI(self):

        #self.text = QtGui.QTextEdit(self)
        self.text = LineTextWidget(self)

        #self.setCentralWidget(self.text)
        self.text.getTextEdit().selectionChanged.connect(self.textSelectionUpdated)
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

 	#f = QtCore.QFile('./default_3.txt')
        #f.open(QtCore.QIODevice.ReadOnly)
        self.model = TreeModel(QtCore.QByteArray(""))
        #f.close()
        
        #self.model = QtGui.QStandardItemModel()
        #self.addItems(self.model, data)
        self.treeView.setModel(self.model)
        #self.treeView.setColumnCount(3)          
        #self.model.setHorizontalHeaderLabels([self.tr("Object")])

	wid = QtGui.QWidget(self)
        self.setCentralWidget(wid)
        layout = QtGui.QHBoxLayout()
        layout.addWidget(self.text)
        layout.addWidget(self.treeView)
        wid.setLayout(layout)

    def textSelectionUpdated(self):
        cursor = self.text.getTextEdit().textCursor();
        #print "Text selection updated: " + str(cursor.blockNumber() + 1)

    def add_item(self,parent,text):
        
        if len(text) > 0: 
          item = QtGui.QStandardItem(text)
          parent.appendRow(item)
          return item
        return NULL

    def refreshTreeviewCallback(self):         
        self.refreshTreeview(self.text.getTextEdit().textCursor().blockNumber()+1)         

    def refreshTreeview(self,numLines=None):
	self.model.setupModelData(self.data.split('\n'),None,numLines)

    def open(self):

        # Get filename and show only .writer files
        self.filename = QtGui.QFileDialog.getOpenFileName(self, 'Open File',".","(*)")

        if self.filename:
          f = QtCore.QFile(self.filename)
          f.open(QtCore.QIODevice.ReadOnly)
          self.data=f.readAll()
          self.refreshTreeview()
	  #self.model.setupModelData(self.data.split('\n'))

          #self.model = TreeModel(data)
          #self.treeView.setModel(self.model)
          self.text.getTextEdit().setText(QtCore.QString(self.data))
          f.close()       

def main():

    app = QtGui.QApplication(sys.argv)

    main = Main()
    main.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
