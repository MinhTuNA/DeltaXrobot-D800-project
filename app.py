import sys
import random

from devices import *
from constants import *
from scurve_interpolator import *

from PyQt5.QtCore import QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout
from PyQt5.QtWidgets import QLabel, QLineEdit
from PyQt5.QtCore import Qt, QSettings, QPoint, QTimer
from PyQt5.QtGui import QCursor, QMouseEvent
from PyQt5.QtTest import QTest
from PyQt5 import QtWidgets
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import Qt, QTimer, QThread, QObject, QIODevice, pyqtSignal as Signal
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QTabWidget,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSpacerItem,
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from widget import Ui_Mainwindows


class MainWindow(QMainWindow, Ui_Mainwindows):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # tạo giao diện
        self.DeltaRobot = Robot(
            "auto", defaultCmd="IsDelta", revMsg="YesDelta", deviceName="D800"
        )
        self.Encoder = Encoder(
            "auto", defaultCmd="IsXEncoder", revMsg="YesXEncoder", deviceName="Enco"
        )
        self.scurve = Scurve_Interpolator()

        self.checkConnectionTimer = QTimer()
        self.checkConnectionTimer.setInterval(2000)
        self.checkConnectionTimer.timeout.connect(self.autoConnect)
        self.status = STATUS.WAIT_CONNECTION

        self.F = FEED_RATE
        self.F2 = FEED_RATE * 4
        self.A = ACCEL
        self.J = JERK
        self.VS = VS
        self.VE = VE

        self.scurve.max_acc = self.A
        self.scurve.max_jer = self.J
        self.scurve.max_vel = self.F
        self.scurve.vel_start = self.VS
        self.scurve.vel_end = self.VE

        # Tạo biểu đồ và thêm vào giao diện
        self.create_plot()
        self.ax = self.figure.add_subplot(111)
        self.swExecute.clicked.connect(self.execute)
        self.swEnableConveyor.clicked.connect(self.onEnableConveyor)
        self.onSensor.clicked.connect(self.enSensor)
        self.count_board = 0 # số board hoàn thành
        self.count = 0 # số board chưa hoàn thành
        self.position_count = [0]*100 # thời điểm cảm biến phát hiện board 
        self.init()
        self.count_point = 0
        self.is_getting_position = False
        self.is_executing = False
        self.moving_to_first_point_time = 0
        self.last_cycle_time = -CYCLE_TIME
        self.last_check_encoder_time = time.time()
        self.last_check_encoder_position = 0.0
        self.first_check_encoder = 0.0
        
        
        self.scurve.max_vel = self.F
        self.scurve.set_moving_distance(
            self.cal_2point_dis(
                self.ready_point,
                [
                    board_pos[0] + point_offsets[0][0],
                    board_pos[1] + point_offsets[0][1],
                    z_safe,
                ],
            )
        )
        self.scurve.start()
        self.move_to_first_point_time = self.scurve.t_target
        print(self.scurve.t_target)
        self.DeltaRobot.send_data("M8 I3 B1 P0")
        self.ready = "G1 X" + str(self.ready_point[0] ) + " Y" + str(self.ready_point[1])+ " Z" + str(self.ready_point[2])
        print(self.ready)
        self.DeltaRobot.send_data(self.ready)
        self.plot_data(0)
        
    def init(self):
        # self.gcodeQueue.clear()
        # self.initGcodeQueue.clear()
        self.ready_point = ready_point.copy()
        self.conveyor_vel = conveyor_speed
        self.sensor_distance = sensor_distance
        self.startTimers()

    def onEnableConveyor(self):
        if self.swEnableConveyor.isChecked():
            self.swEnableConveyor.setStyleSheet("background-color : green")
            self.swEnableConveyor.setText("CONV: STOP")
            self.Encoder.send_data("M310 2")
            self.Encoder.send_data(f"M311 {abs(conveyor_speed)}")
        else:
            self.swEnableConveyor.setStyleSheet("background-color : grey")
            self.swEnableConveyor.setText("CONV: START")
            self.Encoder.send_data("M310 1")
            self.Encoder.send_data("M317 T0")

    def execute(self):
        self.send_gcode("M316 0")
        self.count_point += 1
        self.plot_data(self.count_point)
        
        
    def enSensor(self):
        print("co tin hieu cam bien")
        self.ax.clear()
        if self.is_executing == False:
            self.count += 1
            self.position_count[self.count] = 0
            self.startExecute()
            print("bat dau thuc thi")
        elif self.is_executing == True:
            self.position_count[self.count] = 10
            self.count += 1
            print("them vao hang doi")

    def startExecute(self):
        
        while self.count >= 1:
            
            self.is_executing = True
            firt_check_encoder = self.Encoder.send_data_for_check_encoder("M317")
            #firt_check_encoder = 200
            distance = self.position_count[self.count] - firt_check_encoder
            #distance = 50
            gcodes, points3d = self.create_gcode_for_object()
            #last_check_encoder = 210
            for pointf in points3d:
                print(pointf)
            for point_f in points3d:
                x, y, z, f = point_f
                # last_check_encoder += 5
                x += self.sensor_distance - distance
                last_check_encoder = self.Encoder.send_data_for_check_encoder("M317")
                if f < 500:
                    x = previousx + self.conveyor_vel * point_delay / 1000 * math.cos(conveyor_angle)
                else: 
                    x -= last_check_encoder - firt_check_encoder
                previousx = x
                gcode = "G1 X" + str(x) + " Y" + str(y) + " Z" + str(z) + " F" + str(f)
                print(gcode)
                self.DeltaRobot.send_data(gcode)
                if f == 20:
                    self.count_point += 1
                    self.plot_data(self.count_point)
            self.count_board += 1
            self.count_point = 0
            self.plot_data(7)
            self.lcdNumber.display(self.count_board)
            self.count -= 1
        self.is_executing = False
    def create_plot(self):
        self.plot_widget = self.findChild(QtWidgets.QWidget, "plotWidget")
        if self.plot_widget is None:
            raise ValueError("plotWidget không được tìm thấy trong giao diện.")

        # Tạo Figure và Canvas từ matplotlib
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setParent(self.plot_widget)

        # Tạo một layout để chứa Canvas
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.plot_widget.setLayout(layout)
        # self.plot_data(0)
        # Vẽ biểu đồ
        

    def plot_data(self,count = 0):
        point_offsets = [(40, 35), (35, 10), (20, 38), (15, 10), (5, 25)]
        lines_offsets = [[(50, 39), (50, 25), (28, 25)], [(50, 10), (30, 10)]]
        
        # Vẽ các điểm
        if count == 0:
            # Vẽ trục tọa độ
            self.ax.set_xlim(0, 60)  # Thiết lập giới hạn trục X
            self.ax.set_ylim(0, 50)  # Thiết lập giới hạn trục Y
            self.ax.axhline(0, color='black',linewidth=0.5)  # Vẽ trục X
            self.ax.axvline(0, color='black',linewidth=0.5)  # Vẽ trục Y
        elif 0 < count <= 5:
            x,y = point_offsets[count-1]
            print(f"x = {x}, y = {y}")
            self.ax.scatter(x, y, color="red", marker="o", zorder=2)
        # vẽ các đường
        elif count > 5:
            if count == 6:
                x_coords, y_coords = zip(*lines_offsets[0])  # Tách các tọa độ
                self.ax.plot(x_coords,y_coords,color="blue",marker="o",linestyle="-",zorder = 1)
            elif count == 7:  
                x_coords, y_coords = zip(*lines_offsets[1])  # Tách các tọa độ
                self.ax.plot(x_coords,y_coords,color="blue",marker="o",linestyle="-",zorder = 1)
        self.ax.legend()
        # Cập nhật canvas để hiển thị biểu đồ
        self.canvas.draw()

    def autoConnect(self):
        if self.status == STATUS.WAIT_CONNECTION:
            if self.DeltaRobot.is_connected and self.Encoder.is_connected:
                self.DeltaRobot.find_port_timer.stop()
                self.Encoder.find_port_timer.stop()

                self.DeltaRobot.receivedMsg.connect(self.receivedData)
                self.Encoder.receivedMsg.connect(self.receivedData)

                self.status = STATUS.INIT_DEVICES
                print("kết nối thành công")
                self.DeltaRobot.send_data("M8 I3 B1 P0")
                self.ready = "G1 X" + str(self.ready_point[0] ) + " Y" + str(self.ready_point[1])+ " Z" + str(self.ready_point[2])
                print(self.ready)
                self.DeltaRobot.send_data(self.ready)
                self.execute()
        elif self.status != STATUS.INIT_DEVICES:
            # if self.D800.is_connected == False or self.Encoder.is_connected == False or self.Conveyor.is_connected == False:
            if self.DeltaRobot.is_connected == False:
                self.status = STATUS.WAIT_CONNECTION
                self.DeltaRobot.find_port_timer.start()
                print("D800 connection failed")
            if self.Encoder.is_connected == False:
                self.status = STATUS.WAIT_CONNECTION
                self.Encoder.find_port_timer.start()
                print("Encoder connection failed")

    def startTimers(self):
        self.DeltaRobot.find_port_timer.start(1000)
        self.Encoder.find_port_timer.start(1000)
        self.checkConnectionTimer.start(1000)

    def send_gcode(self, gcode=""):
        if gcode.startswith("M31"):
            self.Encoder.send_data(gcode)
        else:
            self.DeltaRobot.send_data(gcode)

    def receivedData(self, data=""):
        if data.startswith("P0:"):
            current_time = time.time()
            current_position = float(data[3:])
            self.conveyor_vel = -abs(
                current_position - self.last_check_encoder_position
            ) / (current_time - self.last_check_encoder_time)
            self.last_check_encoder_time = current_time
            self.last_check_encoder_position = current_position
            print(f"speed conv: {self.conveyor_vel}")
        elif data.find("I3 V0") > -1:
            if self.is_executing == False:
                self.count += 1
                self.position_count[self.count] = self.Encoder.send_data_for_check_encoder("M317")
                self.startExecute()
            elif self.is_executing == True:
                self.count += 1
                self.position_count[self.count] = self.Encoder.send_data_for_check_encoder("M317")
                
                
        # for gcode in self.initGcodeQueue:
        #     print(gcode)

        # print(self.ready_point)

    def points_after_recognize(self, t=start_delay):
        points = []
        for off in point_offsets:
            (x, y) = off
            # tính tọa độ x = x + khoảng cách vật đã di chuyển sau thời gian di chuyển cánh tay
            # từ vị trí hiện tại đến vị trí đích
            x += self.conveyor_vel * t / 1000
            points.append([board_pos[0] + x, board_pos[1] - y])
        lines = []
        for line_off in lines_offsets:
            line = []
            # duyệt mảng 2 chiều
            for p in line_off:
                (x, y) = p
                # tính tọa độ x
                x += self.conveyor_vel * t / 1000
                line.append([board_pos[0] + x, board_pos[1] - y])
            lines.append(line)
        print("pointsafrec: ", points, lines)
        return points, lines  # trả về mảng sau khi tính tọa độ

    def create_gcode_for_object(self):
        points_2d, lines_2d = self.points_after_recognize(self.scurve.t_target)
        points_3d = []
        gcodes = []
        points_3d_f = []
        point_num = points_2d.__len__()

        for point in points_2d:
            (x, y) = point
            points_3d.append([x, y, z_safe])
            points_3d.append([x, y, z_exe])
            points_3d.append([x, y, z_safe])

        self.ready_point = points_3d[0]
        # print("points_3d: ",points_3d)

        for line in lines_2d:
            for i in range(line.__len__()):
                x, y = line[i][0], line[i][1]
                if i == 0:
                    points_3d.append([x, y, z_safe])
                    points_3d.append([x, y, z_exe])
                elif i == line.__len__() - 1:
                    points_3d.append([x, y, z_exe])
                    points_3d.append([x, y, z_safe])
                else:
                    points_3d.append([x, y, z_exe])
        point_execute_time = 0
        current_f = FEED_RATE

        for id, p in zip(range(points_3d.__len__()), points_3d):
            x, y, z = p
            gcodes.append(
                f"G1 X{points_3d[id][0]} Y{points_3d[id][1]} Z{points_3d[id][2]} F{current_f}"
            )
            points_3d_f.append(
                [points_3d[id][0], points_3d[id][1], points_3d[id][2], current_f]
            )
            if z == z_exe and point_num >= 0:
                point_num -= 1
                x = points_3d[id][0]
                y = points_3d[id][1]
                x = round(x, 3)
                y = round(y, 3)
                gcodes.append(f"G1 X{x} Y{y} Z{z} F{abs(self.conveyor_vel)}")
                points_3d_f.append(
                    [
                        x,
                        y,
                        points_3d[id][2],
                        abs(self.conveyor_vel),
                    ]
                )
                point_execute_time += point_delay

        gcodes.append(f"G1 X{self.ready_point[0]} Y{self.ready_point[1]} Z{z_safe}")

        return gcodes, points_3d_f

    def cal_2point_dis(self, p1=[1, 2, 3], p2=[4, 5, 6]):
        return math.sqrt(
            (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
