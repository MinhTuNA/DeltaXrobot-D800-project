from enum import Enum

class STATUS(Enum):
    WAIT_CONNECTION = -1
    INIT_DEVICES = 0
    FREE = 1
    EXECUTING = 2
    DONE = 3

#movement consts
FEED_RATE = 500.0
ACCEL = 8000.0
JERK = 500000.0
VS = 50.0
VE = 50.0

CYCLE_TIME = 35 #(s) [>= object's length(mm) / conveyor vel(mm/s)]

#D800 PICK AND PLACE
PP_WIDTH = 200
PP_HIGH = -850
PP_LOW = -950
PP_Y = -100

# CUP PICKING

class SEGMENT(Enum):
    INIT_DEVICES = 0
    FREE = 1
    GETTING_CUP = 2
    MOVING_TO_PICK_POS = 3
    EXECUTE_POURING = 4
    DONE = 5

Z_SAFE = -890
PICKING_POS = [100, -150, -680]
POURING_POS = [-100, -100, -660, 0, -135]

AIR_DELAY = 400
GET_CUP_POS = [100, -150, -710]
PLACE_POS = [100, -150, -690]

#demo dan keo
z_safe = -890  #Era antes -820
z_exe = -920   #Era antes -846.5
conveyor_angle = 0.0  #degree

board_pos = [0,0]         #where to start acting
board_dim_xy = [56, 45] #board dimensions
point_offsets = [ (40,35), (35,10),(20,38),(15,10),(5,25)] #point positions on board
lines_offsets = [[(50,39), (50,25), (28,25)],[(50,10),(30,10)]] #lines positions on board

point_delay = 300 #delay in (s) each point
conveyor_speed = -20.0 #constant value of the conv speed
start_delay = 0.0 

ready_point = [board_pos[0]+point_offsets[0][0], board_pos[1]-point_offsets[0][1],z_safe]
line_speed = 100.0

sensor_distance = 150.0 #distance from sensor to ready_point