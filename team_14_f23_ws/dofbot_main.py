#!/usr/bin/env python3
import time
from Arm_Lib import Arm_Device
import inv_kin as inv
import numpy as np

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
	

	


	

