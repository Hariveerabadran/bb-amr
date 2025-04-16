#! /usr/bin/env python3

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer,QThread, pyqtSignal
import time

import sys
import rclpy
import threading
from rclpy.node import Node
from cafe_interfaces.srv import Order

class ROS2Client(Node):
    def __init__(self):
        super().__init__('GUI_order')
        self.client = self.create_client(Order, 'order_gui')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_command(self, table, order:bool, cancel:bool, mode:str):
        request = Order.Request()
        request.tables = table
        request.order = order
        request.cancel = cancel
        request.mode = mode

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

class Ui_CafeOrderApp(object):
    def setupUi(self, CafeOrderApp):
        CafeOrderApp.setObjectName("CafeOrderApp")
        CafeOrderApp.resize(850, 450)
        self.mainLayout = QtWidgets.QHBoxLayout(CafeOrderApp)
        self.mainLayout.setObjectName("mainLayout")
        self.tableLayout = QtWidgets.QVBoxLayout()
        self.tableLayout.setObjectName("tableLayout")
        self.groupBoxTable1 = QtWidgets.QGroupBox(CafeOrderApp)
        self.groupBoxTable1.setObjectName("groupBoxTable1")
        self.verticalLayout1 = QtWidgets.QVBoxLayout(self.groupBoxTable1)
        self.verticalLayout1.setObjectName("verticalLayout1")
        self.orderBtn1 = QtWidgets.QPushButton(self.groupBoxTable1)
        self.orderBtn1.setObjectName("orderBtn1")
        self.verticalLayout1.addWidget(self.orderBtn1)
        self.cancelBtn1 = QtWidgets.QPushButton(self.groupBoxTable1)
        self.cancelBtn1.setObjectName("cancelBtn1")
        self.verticalLayout1.addWidget(self.cancelBtn1)
        self.receivedBtn1 = QtWidgets.QPushButton(self.groupBoxTable1)
        self.receivedBtn1.setObjectName("receivedBtn1")
        self.verticalLayout1.addWidget(self.receivedBtn1)
        self.tableLayout.addWidget(self.groupBoxTable1)
        self.groupBoxTable2 = QtWidgets.QGroupBox(CafeOrderApp)
        self.groupBoxTable2.setObjectName("groupBoxTable2")
        self.verticalLayout2 = QtWidgets.QVBoxLayout(self.groupBoxTable2)
        self.verticalLayout2.setObjectName("verticalLayout2")
        self.orderBtn2 = QtWidgets.QPushButton(self.groupBoxTable2)
        self.orderBtn2.setObjectName("orderBtn2")
        self.verticalLayout2.addWidget(self.orderBtn2)
        self.cancelBtn2 = QtWidgets.QPushButton(self.groupBoxTable2)
        self.cancelBtn2.setObjectName("cancelBtn2")
        self.verticalLayout2.addWidget(self.cancelBtn2)
        self.receivedBtn2 = QtWidgets.QPushButton(self.groupBoxTable2)
        self.receivedBtn2.setObjectName("receivedBtn2")
        self.verticalLayout2.addWidget(self.receivedBtn2)
        self.tableLayout.addWidget(self.groupBoxTable2)
        self.groupBoxTable3 = QtWidgets.QGroupBox(CafeOrderApp)
        self.groupBoxTable3.setObjectName("groupBoxTable3")
        self.verticalLayout3 = QtWidgets.QVBoxLayout(self.groupBoxTable3)
        self.verticalLayout3.setObjectName("verticalLayout3")
        self.orderBtn3 = QtWidgets.QPushButton(self.groupBoxTable3)
        self.orderBtn3.setObjectName("orderBtn3")
        self.verticalLayout3.addWidget(self.orderBtn3)
        self.cancelBtn3 = QtWidgets.QPushButton(self.groupBoxTable3)
        self.cancelBtn3.setObjectName("cancelBtn3")
        self.verticalLayout3.addWidget(self.cancelBtn3)
        self.receivedBtn3 = QtWidgets.QPushButton(self.groupBoxTable3)
        self.receivedBtn3.setObjectName("receivedBtn3")
        self.verticalLayout3.addWidget(self.receivedBtn3)
        self.tableLayout.addWidget(self.groupBoxTable3)
        self.mainLayout.addLayout(self.tableLayout)
        self.kitchenGroup = QtWidgets.QGroupBox(CafeOrderApp)
        self.kitchenGroup.setObjectName("kitchenGroup")
        self.kitchenLayout = QtWidgets.QVBoxLayout(self.kitchenGroup)
        self.kitchenLayout.setObjectName("kitchenLayout")
        self.kitchenDisplay = QtWidgets.QTextEdit(self.kitchenGroup)
        self.kitchenDisplay.setReadOnly(True)
        self.kitchenDisplay.setObjectName("kitchenDisplay")
        self.kitchenLayout.addWidget(self.kitchenDisplay)
        self.kitchenTakeBtn = QtWidgets.QPushButton(self.kitchenGroup)
        self.kitchenTakeBtn.setObjectName("kitchenTakeBtn")
        self.kitchenLayout.addWidget(self.kitchenTakeBtn)
        self.clear_layout = QtWidgets.QHBoxLayout()
        self.clear_layout.setObjectName("clear_layout")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.clear_layout.addItem(spacerItem)
        self.displayclear = QtWidgets.QPushButton(self.kitchenGroup)
        self.displayclear.setObjectName("displayclear")
        self.clear_layout.addWidget(self.displayclear)
        self.kitchenLayout.addLayout(self.clear_layout)
        self.mainLayout.addWidget(self.kitchenGroup)

        self.retranslateUi(CafeOrderApp)
        QtCore.QMetaObject.connectSlotsByName(CafeOrderApp)

    def retranslateUi(self, CafeOrderApp):
        _translate = QtCore.QCoreApplication.translate
        CafeOrderApp.setWindowTitle(_translate("CafeOrderApp", "☕ Café Ordering System"))
        self.groupBoxTable1.setTitle(_translate("CafeOrderApp", "Table 1"))
        self.orderBtn1.setText(_translate("CafeOrderApp", "📝 Order"))
        self.cancelBtn1.setText(_translate("CafeOrderApp", "❌ Cancel"))
        self.receivedBtn1.setText(_translate("CafeOrderApp", "📦 Received"))
        self.groupBoxTable2.setTitle(_translate("CafeOrderApp", "Table 2"))
        self.orderBtn2.setText(_translate("CafeOrderApp", "📝 Order"))
        self.cancelBtn2.setText(_translate("CafeOrderApp", "❌ Cancel"))
        self.receivedBtn2.setText(_translate("CafeOrderApp", "📦 Received"))
        self.groupBoxTable3.setTitle(_translate("CafeOrderApp", "Table 3"))
        self.orderBtn3.setText(_translate("CafeOrderApp", "📝 Order"))
        self.cancelBtn3.setText(_translate("CafeOrderApp", "❌ Cancel"))
        self.receivedBtn3.setText(_translate("CafeOrderApp", "📦 Received"))
        self.kitchenGroup.setTitle(_translate("CafeOrderApp", "Kitchen"))
        self.kitchenTakeBtn.setText(_translate("CafeOrderApp", "✅ Order Taken"))
        self.displayclear.setText(_translate("CafeOrderApp", "clear"))

    def process(self):
        rclpy.init(args=None)
        self.node = ROS2Client()

        self.orderBtn1.clicked.connect(self.t1)
        self.orderBtn2.clicked.connect(self.t2)
        self.orderBtn3.clicked.connect(self.t3)

        self.cancelBtn1.clicked.connect(self.t1_)
        self.cancelBtn2.clicked.connect(self.t2_)
        self.cancelBtn3.clicked.connect(self.t3_)

        self.receivedBtn1.clicked.connect(self.food1)
        self.receivedBtn2.clicked.connect(self.food2)
        self.receivedBtn3.clicked.connect(self.food3)

        self.kitchenTakeBtn.clicked.connect(self.take)

        self.displayclear.clicked.connect(self.kitchenDisplay.clear)

    def t1(self):        
        threading.Thread(target= self._send_command_thread('1',order=True), daemon=True).start()   
    def t2(self):        
        threading.Thread(target= self._send_command_thread('2',order=True), daemon=True).start() 
    def t3(self):        
        threading.Thread(target= self._send_command_thread('3',order=True), daemon=True).start()   

    def t1_(self):        
        threading.Thread(target= self._send_command_thread('1',cancel=True), daemon=True).start()   
    def t2_(self):        
        threading.Thread(target= self._send_command_thread('2',cancel=True), daemon=True).start() 
    def t3_(self):        
        threading.Thread(target= self._send_command_thread('3',cancel=True), daemon=True).start()

    def food1(self):        
        threading.Thread(target= self._send_command_thread('1',mode='T1'), daemon=True).start()   
    def food2(self):        
        threading.Thread(target= self._send_command_thread('2',mode='T2'), daemon=True).start() 
    def food3(self):        
        threading.Thread(target= self._send_command_thread('3',mode='T3'), daemon=True).start() 

    def take(self):
        threading.Thread(target= self._send_command_thread(mode='Kc'), daemon=True).start() 
        
    def _send_command_thread(self, num='\0', order=False, cancel=False, mode='\0'):
        self.node.get_logger().info(f"Sending service request: {num}")
        response = self.node.send_command(table=num,order=order, cancel=cancel,mode=mode)

        self.kitchenDisplay.append(response.msg)

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()
    

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    CafeOrderApp = QtWidgets.QWidget()
    ui = Ui_CafeOrderApp()
    ui.setupUi(CafeOrderApp)
    CafeOrderApp.show()
    ui.process()
    sys.exit(app.exec_())