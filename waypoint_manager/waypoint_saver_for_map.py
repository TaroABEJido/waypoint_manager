import rclpy
from rclpy.node import Node
from pynput import keyboard
import csv
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import threading

import numpy as np
import quaternion

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')

        # Span settings
        self.wp_dist_span = 3.0 # [m]
        self.wp_angle_span = np.pi/180.0 * 10 #[deg] 

        self.poses = []
        self.current_pose = None
        self.pre_save_pose = None
        self.pose_id = 0
        self.xy_goal_tol_ = 2.0
        self.des_lin_vel_ = 0.4
        self.stop_flag_ = 0
        self.skip_flag_ = 1
        self.gps_pose_enable_ = 0  # Set to 0 as per the new requirement
        self.map_pose_enable_ = 1  # Set to 1 as per the new requirement
        self.init_pose_pub_ = 0

        # Subscribe to /current_pose instead of /odom
        # self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 10)
        # self.wp_publisher = self.create_publisher(PoseStamped, '/waypoint_manager/waypoint', 10) 
        print('Press "q" to quit and save to csv.')

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.start()

    def quaternion_dot(self, a, b):
        return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        
        # Save pose at initializing 
        if self.pre_save_pose == None:
            self.pre_save_pose = msg.pose

        vec_current_pose =  np.array ([self.current_pose.position.x,  self.current_pose.position.y,  self.current_pose.position.z])
        vec_save_pose =     np.array ([self.pre_save_pose.position.x, self.pre_save_pose.position.y, self.pre_save_pose.position.z])
        quat_current_pose =     np.quaternion(
                                    self.current_pose.orientation.w, 
                                    self.current_pose.orientation.x, 
                                    self.current_pose.orientation.y, 
                                    self.current_pose.orientation.z
                                )
        quat_current_pose =  quat_current_pose.normalized()
        quat_pre_save_pose =    np.quaternion(
                                    self.pre_save_pose.orientation.w, 
                                    self.pre_save_pose.orientation.x, 
                                    self.pre_save_pose.orientation.y, 
                                    self.pre_save_pose.orientation.z
                                )
        quat_pre_save_pose =  quat_pre_save_pose.normalized()


        vec_current_pose - vec_save_pose
        distance = np.linalg.norm (vec_current_pose - vec_save_pose)
        # print (quat_current_pose)
        # print (quat_pre_save_pose)

        dot_product = self.quaternion_dot(quat_current_pose, quat_pre_save_pose)
        if dot_product < 0 :
            dot_product = self.quaternion_dot(quat_current_pose, -quat_pre_save_pose)

        angle_diff = 2 * np.arccos(np.abs(dot_product))

        if distance > self.wp_dist_span or angle_diff > self.wp_angle_span :

            # Save the pose whenever it is received from /current_position
            self.save_current_pose()

            # Update pre_save_pose 
            self.pre_save_pose = self.current_pose

    def keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_key_press) as listener:
            listener.join()

    def on_key_press(self, key):
        try:
            if key.char == 'q':
                self.save_poses_to_csv()
                self.destroy_node()
                rclpy.shutdown()
        except AttributeError:
            pass

    def save_current_pose(self):
        if self.current_pose is not None:
            pose_data = [
                str(self.pose_id),
                str(self.current_pose.position.x),
                str(self.current_pose.position.y),
                str(self.current_pose.position.z),
                str(self.current_pose.orientation.x),
                str(self.current_pose.orientation.y),
                str(self.current_pose.orientation.z),
                str(self.current_pose.orientation.w),
                str(self.xy_goal_tol_),
                str(self.des_lin_vel_),
                str(self.stop_flag_),
                str(self.skip_flag_),
                # str(self.gps_pose_enable_),
                # str(self.map_pose_enable_),
                # str(self.init_pose_pub_)
            ]
            self.poses.append(pose_data)
            print('Pose saved! ID: ' + str(self.pose_id))
            self.pose_id += 1


        else:
            print('Warning: No pose received yet.')

    def save_poses_to_csv(self):
        filename = datetime.now().strftime('%Y%m%d%H%M%S') + '_waypoints.csv'
        path = os.path.join(os.path.curdir, 'waypoints', filename)  

        os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # writer.writerow(['id', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w', 
            #                  'xy_goal_tol', 'des_lin_vel', 'stop_flag', 'skip_flag', 'gps_pose_enable', 'map_pose_enable', 'init_pose_pub'])
            writer.writerow(['id', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w', 
                             'xy_goal_tol', 'des_lin_vel', 'stop_flag', 'skip_flag'])
            writer.writerows(self.poses)

        print(f'Data saved to {path}!')

def main(args=None):
    rclpy.init(args=args)
    pose_recorder = PoseRecorder()

    try:
        rclpy.spin(pose_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

