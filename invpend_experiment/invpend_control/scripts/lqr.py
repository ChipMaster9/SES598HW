#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import control as ct
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
import math

MASS_CART = 20
MASS_POLE = 2
LENGTH_POLE = 0.5
RADIUS_POLE = 0.025
INERTIA = 0.005
g = 9.81

den = INERTIA*(MASS_CART + MASS_POLE) + MASS_CART * MASS_POLE * LENGTH_POLE**2
A = np.matrix([[0,1,0,0],
               [0, 0, -(MASS_POLE**2 * g * LENGTH_POLE**2)/den, 0],
               [0, 0, 0, 1],
               [0, 0, MASS_POLE * g * LENGTH_POLE * (MASS_CART + MASS_POLE)/den, 0]
               ])

#A = np.matrix( [ [0, 1, 0, 0], 
#                 [0, 0, 0, 0],
#                 [0, 0, 0, 1],
#                 [0, 0, 0, 0]
#                 ]);
                                   
               
B = np.matrix([0, (INERTIA + MASS_POLE * LENGTH_POLE**2)/den, 0, -MASS_POLE * LENGTH_POLE/den]).T
C = np.matrix([[1,0,0,0], [0,0,1,0]])
D = np.matrix([0,0]).T

Q = np.diag([1, 1, 1, 1])
R = np.diag([0.1])

K, S, E = ct.lqr(A, B, Q, R)

class CartPoleLQR:
    def __init__(self):
        self.cartPos = 0
        self.cartvel = 0
        self.polePos = 0
        self.poleVel = 0
        self.ss = None
        self.state = 0
        self.reset_dur = 1
        self.freq = 50
        self.desired_state = np.matrix([[1],[0],[0],[0]])
        self.currentStateSub = rospy.Subscriber('/invpend/joint_states', JointState, self.stateSubCallback)
        self.pub_velocity = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)
    
    def stateSubCallback(self, data):
        self.polePos = data.position[0]
        self.poleVel = data.velocity[0]
        self.cartPos = data.position[1]
        self.cartvel = data.velocity[1]
        self.state = np.matrix([[self.cartPos], [self.cartvel], [self.polePos], [self.poleVel]])
    
    def controlLoop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            U = -1 * np.matmul(K, (self.state - self.desired_state))
            cmd_vel = float(U)
            self.pub_velocity.publish(cmd_vel)
            rate.sleep()
    
    def check_controllability(self):
        ss = ct.ss(A,B,C,D)
        _ctrb = ct.ctrb(ss.A, ss.B)
        _rank = np.linalg.matrix_rank(_ctrb)
        print("The rank of the system is ",_rank)
        if _rank == 4:
            print("Therefore, it is a Controllable System")

if __name__ == '__main__':
    print("hello")
    rospy.init_node('CartController', anonymous=True)
    cartpole = CartPoleLQR()
    cartpole.check_controllability()
    cartpole.controlLoop()
    rospy.spin()
Footer

