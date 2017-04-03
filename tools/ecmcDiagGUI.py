#!/usr/bin/python
# coding: utf-8
import sys
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import Qt
data = [
     ("Alice", [
         ("Keys", []),
         ("Purse", [
             ("Cellphone", [])
             ])
         ]),
     ("Bob", [
         ("Wallet", [
             ("Credit card", []),
             ("Money", [])
             ])
         ])
    ]
class Main(QtGui.QMainWindow):

    def __init__(self, parent = None):
        QtGui.QMainWindow.__init__(self,parent)
        
        self.filename = ""
        self.initUI()

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

        self.text = QtGui.QTextEdit(self)
        #self.setCentralWidget(self.text)

        self.text.setReadOnly(1)
        self.initToolbar()
        self.initMenubar()

        # Initialize a statusbar for the window
        self.statusbar = self.statusBar()


        # x and y coordinates on the sQWidgetcreen, width, height
        self.setGeometry(100,100,1400,800)

        #textEditY1=self.toolbar.y()+self.toolbar.height()+10
        #textEditHeight=self.height()-textEditY1-10
        #self.text.setGeometry(10,textEditY1,900,textEditHeight)
        self.setWindowTitle("ECMC Diagnostics")

        self.treeView = QtGui.QTreeView()
        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        #self.treeView.customContextMenuRequested.connect(self.openMenu)
        
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

        ##################################################################
        ### Models
	##################################################################
	#self.model = QtGui.QStandardItemModel(0, 1, self)
	#self.model.setHeaderData(0, QtCore.Qt.Horizontal, QtCore.QVariant("Topic"))
	#self.proxyModel = QtGui.QSortFilterProxyModel(self)
	#self.proxyModel.setSourceModel( self.model )

	#################################################################
	### Tree
	##################################################################
	#self.tree = QtGui.QTreeView()
	#self.tree.setRootIsDecorated(False)
	#self.tree.setAlternatingRowColors(True)
	#self.tree.setSortingEnabled(True)

        #treeEditX1=self.text.x()+self.text.width()+10
        #treeEditWidth=self.width()-treeEditX1-10        
        #self.tree.setGeometry(treeEditX1,textEditY1,treeEditWidth,textEditHeight)
        #print  "treeEditX1: " + str(treeEditX1)+ ", treeEditWidth: " + str(textEditHeight)
	#self.tree.setModel(self.proxyModel)
	#layout.addWidget(self.tree)
	#self.connect(self.tree, QtCore.SIGNAL("doubleClicked(const QModelIndex&)"), self.on_tree_double_clicked)


    def open(self):

        # Get filename and show only .writer files
        self.filename = QtGui.QFileDialog.getOpenFileName(self, 'Open File',".","(*)")

        if self.filename:
            with open(self.filename,"rt") as file:
                self.text.setText(file.read())


def main():

    app = QtGui.QApplication(sys.argv)

    main = Main()
    main.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
