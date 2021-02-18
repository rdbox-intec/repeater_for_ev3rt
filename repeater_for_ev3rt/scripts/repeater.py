#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Quaternion, PoseStamped, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path


class RelayNode():
    def __init__(self):
        #######################
        # Publish By EV3
        #######################
        # color_sensor_color
        self.sub_robo_color = rospy.Subscriber('color_sensor_color', String, self.callback_robo_color)
        self.pub_robo_color = rospy.Publisher('ev3rt/color_sensor_color', String, queue_size=1)
        # color_sensor_reflect
        self.sub_robo_reflect = rospy.Subscriber('color_sensor_reflect', String, self.callback_robo_reflect)
        self.pub_robo_reflect = rospy.Publisher('ev3rt/color_sensor_reflect', String, queue_size=1)
        # motor_counts
        self.sub_motor_counts = rospy.Subscriber('motor_counts', String, self.callback_motor_counts)
        self.pub_motor_counts = rospy.Publisher('ev3rt/motor_counts', String, queue_size=1)
        # odom
        self.sub_mc_for_odom = rospy.Subscriber('motor_counts', String, self.callback_mc_for_odom)
        self.pub_odom = rospy.Publisher('ev3rt/odom', Odometry, queue_size=1)
        # path
        self.pub_path = rospy.Publisher('ev3rt/path', Path, queue_size=1)
        # cmd_vel
        self.sub_cmd_vel = rospy.Subscriber('ev3rt/cmd_vel', Twist, self.callback_cmd_vel)
        #######################
        # Subscribe By EV3
        #######################
        # motor_steer
        self.sub_motor_steer = rospy.Subscriber('ev3rt/motor_steer', String, self.callback_motor_steer)
        self.pub_motor_steer = rospy.Publisher('motor_steer', String, queue_size=1)
        #######################
        # Param
        #######################
        self.path_msg = Path()
        self.odom_msg = Odometry()
        self.start_flg = False
        self.x = 0
        self.y = 0
        self.th = 0
        self.last_time = rospy.Time.now()
        self.last_enc0_l = 0
        self.last_enc1_r = 0
        self.last_plx = 0
        self.last_paz = 0


    # color_sensor_color
    def callback_robo_color(self, data):
        self.pub_robo_color.publish(data)

    # color_sensor_reflect
    def callback_robo_reflect(self, data):
        self.pub_robo_reflect.publish(data)

    # motor_counts
    def callback_motor_counts(self, data):
        self.pub_motor_counts.publish(data)

    # motor_steer
    def callback_motor_steer(self, data):
        if self.start_flg is False:
            self.start_flg = True
        self.pub_motor_steer.publish(data)

    # cmd_vel
    def callback_cmd_vel(self, data):
        if self.start_flg is False:
            self.start_flg = True
        plx = int((data.linear.x - self.odom_msg.twist.twist.linear.x) * 50 + self.last_plx)
        paz = int((data.angular.z - self.odom_msg.twist.twist.angular.z) * 10 + self.last_paz)
        self.pub_motor_steer.publish('v:{},v:{}'.format(plx, -1*paz))
        self.last_plx = plx
        self.last_paz = paz

    # odom
    def callback_mc_for_odom(self, data):
        if self.start_flg is False:
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            self.pub_odom.publish(self.odom_msg)
            self.last_time = rospy.Time.now()
            return
        current_time = rospy.Time.now()
        dt = current_time.to_sec() - self.last_time.to_sec()
        count_list = [int(count.split(':')[-1]) for count in data.data.split(',')]
        if count_list[0] > 0:
            current_enc0_l = count_list[0] / 360.0
        else:
            current_enc0_l = 0
        if count_list[1] > 0:
            current_enc1_r = count_list[1] / 360.0
        else:
            current_enc1_r = 0
        delta_enc0_l = current_enc0_l - self.last_enc0_l
        delta_enc1_r = current_enc1_r - self.last_enc1_r
        delta_ll = 0.1 * 3.1415926535897931 * delta_enc0_l
        delta_lr = 0.1 * 3.1415926535897931 * delta_enc1_r
        delta_l = (delta_lr + delta_ll) / 2.0
        delta_th = (delta_lr - delta_ll) / (2.0 * 0.0818)
        delta_x = delta_l * math.cos(self.th + (delta_th / 2.0))
        delta_y = delta_l * math.sin(self.th + (delta_th / 2.0))
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        odom_quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.header.seq = self.path_msg.header.seq + 1
        self.path_msg.header.frame_id = "odom"
        self.path_msg.header.stamp = current_time
        pose.header.stamp = current_time
        self.path_msg.poses.append(pose)
        self.odom_msg = Odometry()
        self.odom_msg.header.stamp = current_time
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation = odom_quat
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.twist.twist.linear.x = delta_x / dt
        self.odom_msg.twist.twist.linear.y = delta_y / dt
        self.odom_msg.twist.twist.angular.z = delta_th / dt
        self.odom_msg.pose.covariance[0] = 20.0
        self.odom_msg.pose.covariance[7] = 20.0
        self.odom_msg.pose.covariance[14] = 1000000000000000
        self.odom_msg.pose.covariance[21] = 1000000000000000
        self.odom_msg.pose.covariance[28] = 1000000000000000
        self.odom_msg.pose.covariance[35] = 50.0
        self.odom_msg.twist.covariance[0] = 0.1
        self.odom_msg.twist.covariance[7] = 0.1
        self.odom_msg.twist.covariance[14] = 1000000000
        self.odom_msg.twist.covariance[21] = 1000000000
        self.odom_msg.twist.covariance[28] = 1000000000
        self.odom_msg.twist.covariance[35] = 0.1
        ########################
        self.pub_odom.publish(self.odom_msg)
        self.pub_path.publish(self.path_msg)
        ########################
        self.last_enc0_l = current_enc0_l
        self.last_enc1_r = current_enc1_r
        self.last_time = current_time


if __name__ == '__main__':
    rospy.init_node('relay', xmlrpc_port=22422, tcpros_port=22522)

    node = RelayNode()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
