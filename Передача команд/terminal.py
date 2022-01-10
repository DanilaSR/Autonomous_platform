from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice
from struct import *
import functools

app = QtWidgets.QApplication([])
ui = uic.loadUi("C:/Users/Данила/Autonomous_platform/PyQt5/terminal.ui")
ui.setWindowTitle("SerialGUI")

serial: QSerialPort = QSerialPort()
serial.setBaudRate(115200)
portList = []
ports = QSerialPortInfo().availablePorts()
for port in ports:
    portList.append(port.portName())
ui.comL.addItems(portList)
index_out_mes = 0

def onRead():
    rx = serial.readLine()
    #b = unpack("H", rx[:2])
    #rxs =  str(rx, 'utf-8').strip
    # data = rxs.split(',')
    #res = functools.reduce(lambda sub, ele: sub * 10 + ele, b)
    print(rx)

def onWrite():
    rx = serial.read(9)
    if rx[0] == 126 and rx[8] == 126:
        command = None
        data = None
        if rx[1] == 15:
            command, data = unpack('<cf', rx[2:7])
        if rx[1] == 22:
            command, data = unpack('<cI', rx[2:7])
        if rx[1] == 160:
            command, data = unpack('<ci', rx[2:7])
        if command == b'\xB0':
            ui.L1.setNum(data)
            return
        if command == b'\xB1':
            ui.L2.setNum(data)
            return
        if command == b'\xB2':
            ui.L3.setNum(data)
            return
        if command == b'\xB3':
            ui.L4.setNum(data)
            return
        
        # Command from PC
        if command == b'\xDA':
            ui.L43.setNum(data)
            return
        # State CAR
        if command == b'\xDB':
            ui.L44.setNum(data)
            return


def onOpen():
    serial.setPortName(ui.comL.currentText())
    serial.open(QIODevice.ReadWrite)


def onClose():
    serial.close()

def turn_on_green():
    serial.write( b'\xB0')

def turn_on_blue():
    serial.write( b'\xB1')

def turn_on_red():
    serial.write( b'\xB2')        
 

serial.readyRead.connect(onRead)

ui.RB1.clicked.connect(turn_on_green)
ui.RB2.clicked.connect(turn_on_blue)
ui.RB3.clicked.connect(turn_on_red)


ui.OpenB.clicked.connect(onOpen)
ui.CloseB.clicked.connect(onClose)


ui.show()
app.exec()
