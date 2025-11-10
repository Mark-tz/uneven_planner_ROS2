#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
import math
import time

    # Wheel spacing: 0.18 
    # Axle length: 0.13*2

class CmdVel2Gazebo(Node):

    def __init__(self):
        super().__init__('cmdvel2gazebo')
        self.subscription = self.create_subscription(
            Twist, '/racebot/cmd_vel', self.callback, 1)

        # 使用 Float64MultiArray 并且话题名改为 /commands（Group 控制器接口）
        self.pub_steerL = self.create_publisher(Float64MultiArray, '/racebot/left_front_steering_position_controller/commands', 1)
        self.pub_steerR = self.create_publisher(Float64MultiArray, '/racebot/right_front_steering_position_controller/commands', 1)
        self.pub_rearL = self.create_publisher(Float64MultiArray, '/racebot/left_rear_velocity_controller/commands', 1)
        self.pub_rearR = self.create_publisher(Float64MultiArray, '/racebot/right_rear_velocity_controller/commands', 1)
        self.pub_frontR = self.create_publisher(Float64MultiArray, '/racebot/right_front_velocity_controller/commands', 1)
        self.pub_frontL = self.create_publisher(Float64MultiArray, '/racebot/left_front_velocity_controller/commands', 1)

        # initial velocity and tire angle are 0
        self.x = 0.0
        self.z = 0.0

        # car Wheelbase (in m)
        self.L = 0.26

        # car Tread
        self.T_front = 0.18
        self.T_rear = 0.18 
        
        # car max vel(m/s)
        self.maxvel = 1.5

        # how many seconds delay for the dead man's switch
        self.timeout = 0.2  # seconds
        self.lastMsg = self.get_clock().now()

        # maximum steer angle of the "inside" tire
        self.maxsteerInside=1.5
        # self.maxsteerInside=0.6

        # turning radius for maximum steer angle just with the inside tire
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMax = self.L/math.tan(self.maxsteerInside)

        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        rIdeal = rMax+(self.T_front/2.0)

        # maximum steering angle for ideal middle tire
        # tan(angle) = wheelbase/radius
        self.maxsteer=math.atan2(self.L,rIdeal)

        # Create timer for publishing at 100Hz
        self.timer = self.create_timer(0.01, self.publish)  # 0.01s = 100Hz
        

    def callback(self,data):
        # w = v / r
        self.x = min(self.maxvel, self.x)
        self.x = float(data.linear.x / 0.05)
        # constrain the ideal steering angle such that the ackermann steering is maxed out
        self.z = float(max(-self.maxsteer,min(self.maxsteer,data.angular.z)))
        self.lastMsg = self.get_clock().now()

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # NOTE: we only set self.x to be 0 after 200ms of timeout
        current_time = self.get_clock().now()
        delta_last_msg_time = (current_time - self.lastMsg).nanoseconds / 1e9  # convert to seconds
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            # 发布零命令（单元素数组）
            msgRear = Float64MultiArray(); msgRear.data = [0.0]
            self.pub_rearL.publish(msgRear); self.pub_rearR.publish(msgRear)
            self.pub_frontR.publish(msgRear); self.pub_frontL.publish(msgRear)
            msgSteer = Float64MultiArray(); msgSteer.data = [0.0]
            self.pub_steerL.publish(msgSteer); self.pub_steerR.publish(msgSteer)
            return

        if self.z != 0:
            T_rear = self.T_rear
            T_front = self.T_front
            L=self.L
            # self.v is the linear *velocity*
            r = L/math.fabs(math.tan(self.z))

            rL_rear = r-(math.copysign(1,self.z)*(T_rear/2.0))
            rR_rear = r+(math.copysign(1,self.z)*(T_rear/2.0))
            rL_front = r-(math.copysign(1,self.z)*(T_front/2.0))
            rR_front = r+(math.copysign(1,self.z)*(T_front/2.0))
            msgRearR = Float64MultiArray(); msgRearR.data = [self.x*rR_rear/r]
            msgRearL = Float64MultiArray(); msgRearL.data = [self.x*rL_rear/r]
            self.pub_rearL.publish(msgRearL); self.pub_rearR.publish(msgRearR)

            msgSteerL = Float64MultiArray(); msgSteerL.data = [math.atan2(L,rL_front)*math.copysign(1,self.z)]
            msgSteerR = Float64MultiArray(); msgSteerR.data = [math.atan2(L,rR_front)*math.copysign(1,self.z)]
            self.pub_steerL.publish(msgSteerL); self.pub_steerR.publish(msgSteerR)

            msgFrontL = Float64MultiArray(); msgFrontL.data = [math.sqrt(rR_front*rR_front + L*L)*self.x/r]
            msgFrontR = Float64MultiArray(); msgFrontR.data = [math.sqrt(rL_front*rL_front + L*L)*self.x/r]
            self.pub_frontL.publish(msgFrontL); self.pub_frontR.publish(msgFrontR)
        else:
            msgRear = Float64MultiArray(); msgRear.data = [float(self.x)]
            msgFront = Float64MultiArray(); msgFront.data = [float(self.x)]
            self.pub_frontR.publish(msgFront); self.pub_frontL.publish(msgFront)
            self.pub_rearL.publish(msgRear); self.pub_rearR.publish(msgRear)

            msgSteer = Float64MultiArray(); msgSteer.data = [float(self.z)]
            self.pub_steerL.publish(msgSteer); self.pub_steerR.publish(msgSteer)


if __name__ == '__main__':
    time.sleep(5) #sleep for 5 seconds to wait for spawning model
    rclpy.init()
    try:
        node = CmdVel2Gazebo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
