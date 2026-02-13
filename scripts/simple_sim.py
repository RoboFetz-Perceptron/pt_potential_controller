#!/usr/bin/env python3

import yaml

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Twist, Quaternion



class Simple_Sim(Node):

    def __init__(self):
        super().__init__('simple_sim')

        self.declare_parameter(name='scenario_path', value="")
        self.declare_parameter(name='own_id', value="perceptron")
        self.declare_parameter(name='enemy_id', value="enemy")
        self.declare_parameter(name='freq', value=0.01)
        self.scenario_path = self.get_parameter('scenario_path').get_parameter_value().string_value
        self.own_id = self.get_parameter('own_id').get_parameter_value().string_value
        self.enemy_id = self.get_parameter('enemy_id').get_parameter_value().string_value
        self.freq = self.get_parameter('freq').get_parameter_value().double_value


        if self.scenario_path != "":
            yaml_file = open(self.scenario_path, 'r')
            yaml_str = yaml_file.read()
            parsed = yaml.safe_load(yaml_str)
            print("Loading yaml file at " + self.scenario_path)
            for anchor in parsed["anchors"]:
                if parsed["anchors"][anchor]["id"] == self.own_id:
                    self.own_x = parsed["anchors"][anchor]["pos_x"]
                    self.own_y = parsed["anchors"][anchor]["pos_y"]
                    self.own_t = parsed["anchors"][anchor]["pos_t"]
                if parsed["anchors"][anchor]["id"] == self.enemy_id:
                    self.enemy_x = parsed["anchors"][anchor]["pos_x"]
                    self.enemy_y = parsed["anchors"][anchor]["pos_y"]
        else:
            self.own_x = 0.0
            self.own_y = 0.0
            self.own_t = 0.0
            self.enemy_x = 0.0
            self.enemy_y = 0.0
        self.enemy_t = 0.0 # keep enemy pose, but only publish point
        self.enemy_moved = False

        self.own_twist = Twist()
        self.own_twist_sub = self.create_subscription(Twist, 'own_twist', self.callback_own_twist, 10)
        self.own_pose_pub = self.create_publisher(Pose, 'own_pose', 10)

        self.enemy_twist = Twist()
        self.enemy_twist_sub = self.create_subscription(Twist, 'enemy_twist', self.callback_enemy_twist, 10)
        self.enemy_pose_pub = self.create_publisher(Point, 'enemy_pose', 10)

        self.timer = self.create_timer(self.freq, self.on_timer)
        
    
    def callback_own_twist(self, msg: Twist):
        self.own_twist = msg


    def callback_enemy_twist(self, msg: Twist):
        self.enemy_twist = msg
        self.enemy_moved = True


    def quat_from_euler(self, roll, pitch, yaw):
        ret = Quaternion()
        ret.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        ret.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        ret.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        ret.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return ret


    def on_timer(self):
        print("own twist:", self.own_twist)
        print("enemy twist:", self.enemy_twist)
        dx = self.own_twist.linear.x*np.cos(self.own_t) + self.own_twist.linear.y*np.cos(self.own_t + np.pi/2)
        dy = self.own_twist.linear.x*np.sin(self.own_t) + self.own_twist.linear.y*np.sin(self.own_t + np.pi/2)
        print(f"absolute x/y/w change: {dx}/{dy}/{self.own_twist.angular.z}")
        self.own_x += dx*self.freq
        self.own_y += dy*self.freq
        self.own_t += (self.own_twist.angular.z)*self.freq
        msg = Pose()
        msg.position.x = self.own_x
        msg.position.y = self.own_y
        msg.orientation = self.quat_from_euler(0.0, 0.0, self.own_t)
        self.own_pose_pub.publish(msg)

        if self.enemy_moved: # avoid changing position of anchor with attached potential for performance reasons
            dx = self.enemy_twist.linear.x*np.cos(self.enemy_t) + self.enemy_twist.linear.y*np.cos(self.enemy_t + np.pi/2)
            dy = self.enemy_twist.linear.x*np.sin(self.enemy_t) + self.enemy_twist.linear.y*np.sin(self.enemy_t + np.pi/2)
            self.enemy_x += dx*self.freq
            self.enemy_y += dy*self.freq
            self.enemy_t += (self.enemy_twist.angular.z)*self.freq
            msg = Point()
            msg.x = self.enemy_x
            msg.y = self.enemy_y
            self.enemy_pose_pub.publish(msg)

            
    
def main(args=None):
    rclpy.init(args=args)
    simple_sim_node = Simple_Sim()
    rclpy.spin(simple_sim_node)
    simple_sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
