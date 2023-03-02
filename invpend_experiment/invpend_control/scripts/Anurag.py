
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import control as ct
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
import math
from matplotlib import pyplot as plt


g = 9.81

gcartPos = [];
gpoleVel = [];
gpolePos = [];
gcartvel = [];
gtime = [];


class invpend_LQR:
    def __init__(self):
        self.cartPos = 0
        self.cartvel = 0
        self.polePos = 0
        self.poleVel = 0
        self.ss = None
        self.state = 0
        self.reset_dur = 1
        self.freq = 50
        self.input_goal = np.matrix([[1],[0],[0],[0]])
        self.time = 0;

        m1 = 20;
        m2 = 2;
        l = 0.5;
        I = m1*l*l/12;


        den = m1*m2*l*l + 4*m1*I + 4*m2*I;
        
        A23 = -m2*m2*l*l*g/den;
        A43 = 2*l*m2*(g*m1+g*m2)/den;
        
        B2 = (4*I+m2*l*l)/den;
        B4 = -2*l*m2/den;

        A = np.matrix([[0,1,0,0],
               [0, 0, A23, 0],
               [0, 0, 0, 1],
               [0, 0, A43, 0]
               ])
        B = np.matrix([0, B2, 0, B4]).T
        
        C = np.matrix([[1,0,0,0], [0,0,1,0]])
        D = np.matrix([0,0]).T
        
        Q = np.diag([1, 1, 10, 100])
        R = np.diag([0.1])

        K, S, E = ct.lqr(A, B, Q, R)
        
        self.gain = K;

        self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.callback)
        self.pub_velocity = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)
        self._pub_set_pole = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
        
                    
    
    def callback(self, data):
        self.polePos = data.position[0]
        self.poleVel = data.velocity[0]
        self.cartPos = data.position[1]
        self.cartvel = data.velocity[1]
        self.state = np.matrix([[self.cartPos], [self.cartvel], [self.polePos], [self.poleVel]])
        stamp = data.header.stamp
        self.time = stamp.secs + stamp.nsecs * 1e-9
        
    
    def controlLoop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            Xd = -1 * np.matmul(self.gain, (self.state - self.input_goal))
            cmd_vel = float(Xd)
            self.pub_velocity.publish(cmd_vel)
            rate.sleep()
        
        

if __name__ == '__main__':
    rospy.init_node('invpend_lqr', anonymous=True)
    cartpole = invpend_LQR()
    cartpole.controlLoop()
    plt.ion()
    plt.show()
    rospy.spin()