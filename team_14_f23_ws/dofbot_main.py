#!/usr/bin/env python3
import time
from Arm_Lib import Arm_Device
import inv_kin as inv
import numpy as np
from aruco_helper import Blocks, updateBlockPositions

def reset(duration):
	Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, duration)
	time.sleep(2)

def place_block_simple(start, finish):
	R = np.eye(3)
	
	q_s = inv.inv_kin(R,start)
	q_f = inv.inv_kin(R,finish)

	Arm.Arm_serial_servo_write6(q_s[0], q_s[1], q_s[2], q_s[3], 90, 0, 5000)
	time.sleep(6)
	
	Arm.Arm_serial_servo_write6(q_s[0], q_s[1], q_s[2], q_s[3], 90, 140, 2000)
	time.sleep(3)
	Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 140, 5000)
	time.sleep(6)
	Arm.Arm_serial_servo_write6(q_f[0], q_f[1], q_f[2], q_f[3], 90, 140, 5000)
	time.sleep(6)
	Arm.Arm_serial_servo_write6(q_f[0], q_f[1], q_f[2], q_f[3], 90, 0, 2000)
	time.sleep(3)
	reset(5000)

def lerp(a,b,t):
	if t < 0.0:
		t = 0.0
	elif t > 1.0:
		t = 1.0
	return (1-t)*a + t*b

def linear_path(a,b, time, steps):
	step = float(1/steps)
	times = []
	values = []
	for i in range(steps + 1):
		values.append(lerp(a,b, i*step))
		times.append(time*step*i)
	return values, time*step
		
def follow_path(positions, Time):
	R = np.eye(3)
	q_s = []

	for i in range(len(positions)):
		q = inv.inv_kin(R,positions[i])
		q_s.append(q)
	for i in range(len(positions)):
		q = q_s[i]

		Arm.Arm_serial_servo_write6(q[0], q[1], q[2], q[3], 90, 0, int(Time))
		time.sleep(Time/1000)		


if __name__ == "__main__":
    
    """ Create Robot Object (speeds up code)"""
    
    # Define all the joint lengths [m]
    l0 = 61 * 10**-3
    l1 = 43.5 * 10**-3
    l2 = 82.85 * 10**-3
    l3 = 82.85 * 10**-3
    l4 = 73.85 * 10**-3
    l5 = 54.57 * 10**-3

    # define the unit vectors
    ex = np.array([1, 0, 0])
    ey = np.array([0, 1, 0])
    ez = np.array([0, 0, 1])

    # Define the position vectors from i-1 -> i
    P01 = (l0 + l1) * ez
    P12 = np.zeros(3)
    P23 = l2 * ex
    P34 = -1*l3 * ez
    P45 = np.zeros(3)
    P5T = -1*(l4 + l5) * ex

    # define the class inputs: rotation axes (H), position vectors (P), and joint_type
    H = np.array([ez, -1*ey, -1*ey, -1*ey, -1*ex]).T
    P = np.array([P01, P12, P23, P34, P45, P5T]).T
    joint_type = [0, 0, 0, 0, 0]

    # define the Robot class
    robot = rox.Robot(H, P, joint_type)
    
    """ Create Block Objects """
    
    block1 = Block({0:[ ez,  ey],
                    1:[-ez, -ex],
                    2:[ ex,  ey],
                    3:[-ey,  ex],
                    4:[-ex, -ey],
                    5:[ ey, -ex]})

    block2 = Block({6:[ ez,  ey],
                    7:[-ez, -ex],
                    8:[ ex,  ey],
                    9:[-ey,  ex],
                   10:[-ex, -ey],
                   11:[ ey, -ex]})

    block3 = Block({12:[ ez,  ey],
                    13:[-ez, -ex],
                    14:[ ex,  ey],
                    15:[-ey,  ex],
                    16:[-ex, -ey],
                    17:[ ey, -ex]})

    block4 = Block({18:[ ez,  ey],
                    19:[-ez, -ex],
                    20:[ ex,  ey],
                    21:[-ey,  ex],
                    22:[-ex, -ey],
                    23:[ ey, -ex]})

    blocks = [block1, block2, block3, block4]
    
    """ Find Camera Port """
    for port in range(10):
        camera = cv2.VideoCapture(port)
        ret, image = camera.read()
        if ret:
            break
    
    
    """ Make Robot Perform Tasks """
    
	Arm = Arm_Device()
	time.sleep(0.1)

	reset(1000)
	R = np.eye(3)

	P = np.array([[0], [0.3], [0.2]])
	P1 = np.array([[0.3], [0], [0.2]])
	q = inv.inv_kin(R,P)

	#Arm.Arm_serial_servo_write6(q[0], q[1], q[2], q[3], 90, 0, 5000)
	#time.sleep(6)

	#path, Time = linear_path(P,P1, 1000,100)
	#print(Time)
	#follow_path(path,Time)

	#Pf = np.array([[0], [0.17], [0.081]])
	#Ps = np.array([[-0.11], [0.23], [0.06]])
	
		

	place_block_simple(Ps, Pf)
