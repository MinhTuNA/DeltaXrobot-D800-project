from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QTimer, QThread, QObject, QIODevice,pyqtSignal as Signal
from PyQt5.QtWidgets import QApplication


import sys
import time
import math

class Device(QObject):
    receivedMsg = Signal(str)
    connectDevice = Signal(bool)

    def __init__(self, COM='auto', baudrate=115200, default_cmd='', rev_msg='Ok', device_name='device'):
        super().__init__()
        self.serial_device = QSerialPort()

        self.data = ''
        self.COM = COM
        self.baudrate = baudrate
        self.default_cmd = default_cmd
        self.rev_msg = rev_msg
        self.device_name = device_name
        #test
        self.is_connected = False
        
        self.find_port_timer = QTimer()
        self.find_port_timer.timeout.connect(lambda: self.check_connection())
        # self.find_port_timer.setInterval(1000)
        #test
        # self.timer.start(1000)

    def find_device_port(self):
        if self.COM.lower() == 'auto' and self.default_cmd != '':
            for port_info in QSerialPortInfo.availablePorts():
                self.serial_device.setPortName(port_info.portName())
                self.serial_device.setBaudRate(self.baudrate)
                if self.serial_device.open(QIODevice.OpenModeFlag.ReadWrite):
                    self.serial_device.write((self.default_cmd + '\n').encode())
                    self.serial_device.waitForReadyRead(500)
                    response = self.serial_device.readAll().data().decode('utf-8')
                    print(response)
                    if response.find(self.rev_msg) > -1:
                        print(self.device_name + ' is connected at ' +
                              port_info.portName())
                        self.serial_device.errorOccurred.connect(self.handle_connection_error)
                        self.serial_device.readyRead.connect(self._read)
                        self.connectDevice.emit(True)
                        self.is_connected = True
                        self.find_port_timer.stop()
                        break
                    else:
                        self.serial_device.close()
        elif self.COM.lower() != 'auto':
            self.connect(port=self.COM)

    def connect(self, port='COM1', baudrate=115200, autoRead=False):
        self.serial_device.setPortName(port)
        self.serial_device.setBaudRate(baudrate)

        if self.serial_device.open(QIODevice.OpenModeFlag.ReadWrite):
            self.serial_device.errorOccurred.connect(self.handle_connection_error)
            self.serial_device.readyRead.connect(self._read)
            self.connectDevice.emit(True)
            self.is_connected = True

    def _read_line(self):
        start = time.time()
        while time.time() - start < 2000:
            self.serial_device.waitForReadyRead(100)
            if self.serial_device.canReadLine() == True:
                break
        return self._read()

    def _read(self):
        if self.serial_device.canReadLine():
            self.data = self.serial_device.readLine().data().decode()
            msg = self.data.strip('\n')
            print(self.device_name + " << " + msg)
            self.receivedMsg.emit(self.data)
            return self.data

    def handle_connection_error(self):
        self.is_connected = False
        self.serial_device.close()
        self.serial_device.errorOccurred.disconnect(self.handle_connection_error)
        self.serial_device.readyRead.disconnect(self._read)
        self.connectDevice.emit(False)

    def send_data(self, data=''):
        if self.serial_device.isOpen():
            print(self.device_name + " >> " + data)
            if not data.endswith('\n'): 
                data += '\n'
            self.serial_device.write(data.encode())
            print(data)
            return True
        else:
            print(self.device_name + " >>? " + data)
            return False

    def send_data_for_check_encoder(self, data='',timeout = 5):
        if self.serial_device.isOpen():
            print(self.device_name + " >> " + data)
            if not data.endswith('\n'): 
                data += '\n'
            self.serial_device.write(data.encode())
            print(data)
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if self.serial_device.canReadLine():
                    data_p = self.serial_device.readLine().data().decode()
                    if data_p.startswith("P0:"):
                        position = float(data[3:])
                        print(f"position: {position}")
                        return position
                    else: 
                        print("sai định dạng dữ liệu")
                        return False
            print(self.device_name + " >>? Không nhận được phản hồi trong thời gian chờ.")
            return False
        else:
            print(self.device_name + " >>? " + data)
            return False

    def check_connection(self):
        if self.is_connected == False:
            print(f'finding {self.device_name} port . . .')
            self.find_device_port()

class Robot(Device):
    gcode_done = Signal()
    received_position = Signal(str)

    def __init__(self, COM='None', baudrate=115200, defaultCmd="IsDelta", revMsg="YesDelta", deviceName="Delta"):
        super().__init__(COM=COM, baudrate=baudrate, default_cmd=defaultCmd,
                         rev_msg=revMsg, device_name=deviceName)

        self.receivedMsg.connect(self.get_response)
        self.X, self.Y, self.Z, self.W, self.U, self.V, self.F, self.A, self.S, self.E, self.J = 0.0, 0.0, 0.0, 0, 0, 0, 500, 8000, 30, 40, 255000

        self.done_msg = 'Ok'

        self.path_type = 'line'
        self.path_vel = 100
        self.path_angle = 0
        self.path_rad_angle = 0
        self.is_sync = False
        self.encoder = None

        self.gcode_queue = []
        self.gcode_queue_length = 0
        self.last_gcode = ''

    def get_response(self, response=""):
        if response.count(self.done_msg) > 0:
            self.gcode_done.emit()

        if self.last_gcode.count('Position') > 0:
            if response.count(',') > 2:
                self.received_position.emit()

    def send_gcode(self, gcode="G28", time_out=10000):

        if not gcode.endswith('\n'):
            gcode = gcode + '\n'

        if gcode.find('G04') < 0:
            print(gcode)

        if self.serial_device.isOpen():
            self.serial_device.write(gcode.encode())
        else:
            print('Robot is not connected')
            self.thread().sleep(1)
            return

        self.last_gcode = gcode
        isMovingGcode = self.get_para(gcode)

        return ''

    def get_para(self, gcode):
        paras = gcode.split()

        if paras[0].find('G01') < 0 and paras[0].find('G1') < 0:
            return False

        self.old_X, self.old_Y, self.old_Z = self.X, self.Y, self.Z

        for para in paras:
            if para[0] == 'X':
                self.X = float(para[1:])
            if para[0] == 'Y':
                self.Y = float(para[1:])
            if para[0] == 'Z':
                self.Z = float(para[1:])
            if para[0] == 'W':
                self.W = float(para[1:])
            if para[0] == 'U':
                self.U = float(para[1:])
            if para[0] == 'V':
                self.V = float(para[1:])
            if para[0] == 'F':
                self.F = float(para[1:])
            if para[0] == 'A':
                self.A = float(para[1:])
            if para[0] == 'S':
                self.S = float(para[1:])
            if para[0] == 'E':
                self.E = float(para[1:])
            if para[0] == 'J':
                self.J = float(para[1:])

        return True

    def __cal_move_time(self):
        xy = math.sqrt(math.pow(self.X - self.old_X, 2) +
                       math.pow(self.Y - self.old_Y, 2))
        xyz = math.sqrt(math.pow(xy, 2) + math.pow(self.Z - self.old_Z, 2))

    def input(self, pin):
        gcode = 'M07 I{0}'.format(pin)
        state = self.send_gcode(gcode)
        paras = state.split()
        for para in paras:
            if para[0] == 'V':
                return para[1:]

    def output(self, pin, state):
        if state != 0:
            gcode = 'M03 D{0}'.format(pin)
        else:
            gcode = 'M05 D{0}'.format(pin)

        return self.send_gcode(gcode)

    def G01(self, X=None, Y=None, Z=None, W=None, U=None, V=None, F=None, A=None, S=None, E=None, J=None):
        gcode = 'G01'

        self.old_X, self.old_Y, self.old_Z = self.X, self.Y, self.Z

        if X != None:
            self.X = X
            gcode = gcode + ' X' + str(X)
        if Y != None:
            self.Y = Y
            gcode = gcode + ' Y' + str(Y)
        if Z != None:
            self.Z = Z
            gcode = gcode + ' Z' + str(Z)
        if W != None:
            self.W = W
            gcode = gcode + ' W' + str(W)
        if U != None:
            self.U = U
            gcode = gcode + ' U' + str(U)
        if V != None:
            self.V = V
            gcode = gcode + ' V' + str(V)
        if F != None:
            self.F = F
            gcode = gcode + ' F' + str(F)
        if A != None:
            self.A = A
            gcode = gcode + ' A' + str(A)
        if S != None:
            self.S = S
            gcode = gcode + ' S' + str(S)
        if E != None:
            self.E = E
            gcode = gcode + ' E' + str(E)
        if J != None:
            self.J = J
            gcode = gcode + ' J' + str(J)

        return gcode

    def move_point(self, p):
        self.G01(X=p[0], Y=p[1], Z=p[2])

    def go_home(self):
        self.send_gcode('G28')

    def set_sync_path(self, path='line', con_vel=100, con_angle=0, encoder=None):
        self.path_type = path
        self.path_vel = con_vel
        self.path_angle = con_angle
        self.path_rad_angle = math.radians(con_angle)
        self.is_sync = True
        self.encoder = encoder

    def stop_sync(self):
        self.is_sync = False


class Conveyor(Device):
    def __init__(self, COM='None', baudrate=115200, defaultCmd="M310 1", revMsg="Ok", deviceName="Conveyor"):
        super().__init__(COM=COM, baudrate=baudrate, default_cmd=defaultCmd,
                         rev_msg=revMsg, device_name=deviceName)
        self.speed = 0

class Encoder(Device):
    def __init__(self, COM='None', baudrate=115200, defaultCmd="IsXEncoder", revMsg="YesXEncoder", deviceName="Encoder"):
        super().__init__(COM=COM, baudrate=baudrate, default_cmd=defaultCmd,
                         rev_msg=revMsg, device_name=deviceName)
        self.speed = 0
    
        

# if __name__ == '__main__':
#     app=QApplication(sys.argv)
#     robot = Robot('auto')
#     slider = Slider('auto')
#     rtable = RotateTable('auto')
#     sys.exit(app.exec_())