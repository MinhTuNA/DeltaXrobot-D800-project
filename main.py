import copy
import math
import time

from devices import *
from constants import *
from scurve_interpolator import *

from PyQt5.QtCore import QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout
from PyQt5.QtWidgets import QLabel, QLineEdit
from PyQt5.QtCore import Qt, QSettings, QPoint, QTimer
from PyQt5.QtGui import QCursor, QMouseEvent
from PyQt5.QtTest import QTest

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

from PyQt5.QtGui import QPixmap, QImage, QPalette, QColor


class StickyManager(QMainWindow):

    def __init__(self):
        super().__init__()
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

        self.original_gcode_queue = []
        self.handle_object_timer = QTimer()
        self.handle_object_timer.timeout.connect(self.handle_object)

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

        # self.robots = [self.DeltaRobot]
        self.initGcodeQueue = ["G28"]
        self.gcodeQueue = []
        self.subGcodeQueue = []

        self.paths = []
        self.currentPointIndex = -1
        self.waitThreeOk = False
        self.okCounter = 0

        self.init()
        self.initUI()

        self.btExecute.clicked.connect(self.startExecute)
        self.swEnableConveyor.clicked.connect(self.onEnableConveyor)
        self.EnableSensor.clicked.connect(self.enSensor)
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
        # calculate time moving to 1st point

    def init(self):
        self.gcodeQueue.clear()
        self.initGcodeQueue.clear()
        self.ready_point = ready_point.copy()
        self.conveyor_vel = conveyor_speed
        self.startTimers()

    def initUI(self):
        self.setWindowTitle("DeltaX - Applying Adhesive")
        self.setGeometry(100, 100, 300, 200)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.btExecute = QPushButton()
        self.btExecute.setText("Execute")
        self.btExecute.setFixedHeight(50)
        self.swEnableConveyor = QPushButton()
        self.swEnableConveyor.setText("CONV: START")
        self.swEnableConveyor.setCheckable(True)
        self.swEnableConveyor.setStyleSheet("background-color : lightgrey")
        self.swEnableConveyor.setFixedHeight(50)

        self.EnableSensor = QPushButton()
        self.EnableSensor.setText("Enable Sensor")
        self.EnableSensor.setCheckable(True)
        self.EnableSensor.setStyleSheet("background-color : lightgrey")
        self.EnableSensor.setFixedHeight(50)

        self.lbTerminal = QLabel()
        self.lbTerminal.setFixedHeight(30)
        self.lbTerminal.setStyleSheet("color: green;")

        vBox = QVBoxLayout()
        vBox.addWidget(self.btExecute)
        vBox.addWidget(self.swEnableConveyor)
        vBox.addWidget(self.EnableSensor)
        vBox.addWidget(self.lbTerminal)
        main_layout.addLayout(vBox)

    def onEnableConveyor(self):
        if self.swEnableConveyor.isChecked():
            self.swEnableConveyor.setStyleSheet("background-color : green")
            self.swEnableConveyor.setText("CONV: STOP")
            self.Encoder.send_data("M310 2")
            self.Encoder.send_data("M311 -50")
        else:
            self.swEnableConveyor.setStyleSheet("background-color : grey")
            self.swEnableConveyor.setText("CONV: START")
            self.Encoder.send_data("M310 1")
            self.Encoder.send_data("M317 T0")

    def startExecute(self):
        # firt_check_encoder = self.Encoder.send_data_for_check_encoder("M317")
        firt_check_encoder = 200
        gcodes, points3d = self.create_gcode_for_object()
        last_check_encoder = 210
        for point_f in points3d:
            x, y, z, f = point_f
            # last_check_encoder = self.Encoder.send_data_for_check_encoder("M317")
            last_check_encoder += 10
            x += last_check_encoder - firt_check_encoder
            gcode = "G1 X" + str(x) + " Y" + str(y) + " Z" + str(z) + " F" + str(f)
            print(gcode)

    def enSensor(self):
        self.send_gcode("M8 I3 B1 P0")

    def execute(self):
        self.send_gcode("M316 0")
        self.send_gcode("M317 T500")

    def autoConnect(self):
        if self.status == STATUS.WAIT_CONNECTION:
            if self.DeltaRobot.is_connected and self.Encoder.is_connected:
                self.DeltaRobot.find_port_timer.stop()
                self.Encoder.find_port_timer.stop()

                self.DeltaRobot.receivedMsg.connect(self.receivedData)
                self.Encoder.receivedMsg.connect(self.receivedData)

                self.status = STATUS.INIT_DEVICES
                print("kết nối thành công")
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

        # for gcode in self.initGcodeQueue:
        #     print(gcode)

        # print(self.ready_point)

    def handle_object(self):
        self.handle_object_timer.stop()
        self.lbTerminal.setText("Object Processing..")
        self.is_executing = True
        if self.gcodeQueue.__len__() == 0:
            self.status = STATUS.EXECUTING
            self.gcodeQueue = self.create_gcode_for_object()
            delay_time = int(abs(sensor_distance / self.conveyor_vel) * 1000)
            print(f"exe after {delay_time}")
            temp_timer = QTimer()
            temp_timer.singleShot(delay_time, self.execute)
        else:
            self.subGcodeQueue.append(self.create_gcode_for_object())

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
        is_first_point_in_line = True

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
                x = points_3d[id][
                    0
                ] + self.conveyor_vel * point_delay / 1000 * math.cos(conveyor_angle)
                y = points_3d[id][
                    1
                ] + self.conveyor_vel * point_delay / 1000 * math.sin(conveyor_angle)
                x = round(x, 3)
                y = round(y, 3)
                points_3d[id][0] = x
                points_3d[id][1] = y
                gcodes.append(f"G1 X{x} Y{y} Z{z} F{abs(self.conveyor_vel)}")
                points_3d_f.append(
                    [
                        points_3d[id][0],
                        points_3d[id][1],
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

    ins = StickyManager()
    ins.show()

    sys.exit(app.exec_())
