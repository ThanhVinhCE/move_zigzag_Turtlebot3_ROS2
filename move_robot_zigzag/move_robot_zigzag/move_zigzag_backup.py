import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped

class SubscriberClass(Node):

    def __init__(self):
        super().__init__('sub_node')
        rclpy.logging.set_logger_level('sub_node', rclpy.logging.LoggingSeverity.DEBUG)
        
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = MutuallyExclusiveCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=self.group1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE), callback_group=self.group2)
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10, callback_group=self.group4)

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.group3)
        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.vel_msg = Twist()
        self.initialpose_msg = PoseWithCovarianceStamped()
        self.publisher = PublisherClass()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # self.publisher = PublisherClass()

        self.safe_distance = 0.25

        # self.first_call = True
        self.first_call = True

        # self.rotate_part1_completed = False
        self.rotate_part1_completed = False
        # self.rotating = False
        self.rotating = False

        self.rotate_counter_clk_completed = False
        self.rotate_clk_completed = False

        self.move_straight_flag = False
        self.move_backward_flag = False
        # self.moving_backward = False
        # self.rotate_clk_safe = False
        # self.rotate_counter_clk_safe = False
        self.move_zigzag_completed = False

        # self.target_angle = 0.0
        # self.target_angle1 = 999.0
        # self.target_angle2 = 999.0

        # self.target_angle = 0.0
        # self.target_angle1 = 999.0
        # self.target_angle2 = 999.0

        self.target_angle = 0.0
        self.target_angle1 = 999.0
        self.target_angle2 = 999.0

        # self.first_read = True

        # self.count_rotate_clk_time = 0
        # self.count_rotate_counter_clk_time = 0
        # self.move_straight_1s_done = False
        # self.move_straight_period_done = False

        # self.count_time = 0
        # self.move_straight_period = 3

        self.initial_pose_is_set = False

        self.not_rotating_clk_yet = True
        self.not_rotating_counter_clk_yet = True

        self.can_move_straight = False
        self.can_move_backward = False
        self.can_rotate_clk = False
        self.can_rotate_counter_clk = False

        self.rotated_counter_clk_first_time = False
        self.rotated_counter_clk_second_time = False
        self.rotated_clk_first_time = False
        self.rotated_clk_second_time = False

        self.move_backward_finished = False
        self.is_moving_backward = False
        self.move_straight_in_a_period_time_finished = False
        self.pausing_move_straight = False
        self.start_moving_backward_1s = False

        self.state = 'STOPPED'

    def initialpose_callback(self, msg):
        self.initialpose_msg = msg
        if self.initialpose_msg.pose.pose.position.x != -999:
            self.initial_pose_is_set = True

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion (orientation_list)

    def scan_callback(self, msg):
        self.laser_msg = msg

    def stop_robot(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.publisher_.publish(self.vel_msg)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        """"
        reference from: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
        """

        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def rotate_counter_clockwise(self, target_angle):
        self.rotating = True

        while self.first_call:
            if self.yaw < 0:
                if abs(self.yaw) > 0.01:
                    self.publisher.stop_robot()
                    self.target_angle = target_angle*(np.pi)/180.0 + self.yaw
                    if abs(self.target_angle + (np.pi)) > 0.02 and self.target_angle > 0.0:
                        self.target_angle1 = self.yaw
                        self.target_angle2 = target_angle*(np.pi)/180.0 + self.target_angle1
                    self.first_call = False
                else:
                    self.publisher.rotate_counter_clockwise_small_angle()
            else:
                if abs(np.pi - self.yaw) > 0.01:
                    self.publisher.stop_robot()
                    self.target_angle = target_angle*(np.pi)/180.0 + self.yaw
                    if abs(self.target_angle - (np.pi)) > 0.02 and self.target_angle > 3.0:
                        self.target_angle1 = (np.pi) - self.yaw
                        self.target_angle2 = (np.pi) - (target_angle*np.pi/180.0 - self.target_angle1)
                    self.first_call = False
                else:
                    self.publisher.rotate_counter_clockwise_small_angle()

        while True:

            if self.target_angle1 != 999.0 and self.target_angle1 < 0:
                if self.rotate_part1_completed == False:
                    error_angle = self.yaw
                else:
                    while self.yaw < 0:
                        self.publisher.rotate_counter_clockwise_small_angle()

                    if self.yaw > 0:
                        self.publisher.stop_robot()
                        error_angle = self.target_angle2 - self.yaw

            elif self.target_angle1 != 999.0 and self.target_angle1 > 0:
                if self.rotate_part1_completed == False:
                    error_angle = np.pi - self.yaw
                else:
                    while self.yaw > 0:
                        self.publisher.rotate_counter_clockwise_small_angle()

                    if self.yaw < 0:
                        self.publisher.stop_robot()
                        error_angle = self.target_angle2 + self.yaw
            else:
                error_angle = self.target_angle - self.yaw
                self.rotate_part1_completed = True

            if self.rotate_counter_clk_completed == False:
                if abs(error_angle) > 0.005:
                    if abs(error_angle) > 0.03:
                        if abs(error_angle) < np.pi:
                            if abs(error_angle) > 2.5:
                                rotational_velocity = 0.1*abs(error_angle)
                            elif abs(error_angle) > 1.5 and abs(error_angle) <= 2.5:
                                rotational_velocity = 0.2*abs(error_angle)
                            else:
                                rotational_velocity = 0.5*abs(error_angle)
                    else:
                        rotational_velocity = 0.01
                    command_vel = 0.0
                    self.publisher.publish_vel(command_vel, rotational_velocity)
                    time.sleep(0.1)
                else:
                    if self.rotate_part1_completed == False:
                        self.rotate_part1_completed = True
                        self.get_logger().info('ROTATE PARTIALLY COMPLETED')
                    else:
                        self.get_logger().info('ROTATE COMPLETED')
                        self.rotating = False
                        self.rotate_counter_clk_completed = True
                        self.target_angle = 0.0
                        self.target_angle1 = 999.0
                        self.target_angle2 = 999.0
                        self.rotate_part1_completed = False
                        self.first_call = True
                        # self.count_rotate_counter_clk_time += 1
                        self.publisher.stop_robot()
                        break

    def rotate_clockwise(self,target_angle):
        self.rotating = True

        while self.first_call == True:
            if self.yaw <= 0:
                if abs(self.yaw + (np.pi)) > 0.01:
                    self.publisher.stop_robot()
                    self.target_angle = -target_angle*(np.pi)/180.0 + self.yaw
                    if abs(self.target_angle + np.pi) > 0.02 and abs(self.target_angle) > np.pi:
                        self.target_angle1 = -self.yaw - (np.pi)
                        self.target_angle2 = (np.pi) - (target_angle*np.pi/180.0 + self.target_angle1)

                    self.first_call = False
                else:
                    self.publisher.rotate_clockwise_small_angle()
            else:
                if abs(self.yaw) > 0.01:
                    self.publisher.stop_robot()
                    self.target_angle = self.yaw - target_angle*(np.pi)/180.0
                    if abs(self.target_angle) > 0.02 and self.target_angle < 0.0:
                        self.target_angle1 = self.yaw
                        self.target_angle2 = - (target_angle*np.pi/180.0 - self.target_angle1)
                        
                    self.first_call = False
                else:
                    self.publisher.rotate_clockwise_small_angle()

        while True:

            if self.target_angle1 != 999.0 and self.target_angle1 < 0:
                if self.rotate_part1_completed == False:
                    error_angle = self.yaw + (np.pi)
                else:
                    while self.yaw < 0:
                        self.publisher.rotate_clockwise_small_angle()

                    if self.yaw > 0:
                        self.publisher.stop_robot()
                        error_angle = -self.target_angle2 + self.yaw

            elif self.target_angle1 != 999.0 and self.target_angle1 > 0:
                if self.rotate_part1_completed == False:
                    error_angle = self.yaw
                else:
                    while self.yaw > 0:
                        self.publisher.rotate_clockwise_small_angle()

                    if self.yaw <= 0:
                        self.publisher.stop_robot()
                        error_angle = self.target_angle2 - self.yaw
            else:
                error_angle = self.target_angle - self.yaw
                self.rotate_part1_completed = True

            if self.rotate_clk_completed == False:
                if abs(error_angle) > 0.005:
                    if abs(error_angle) > 0.03:
                        if abs(error_angle) < np.pi:
                            if abs(error_angle) > 2.5:
                                rotational_velocity = -0.1*abs(error_angle)
                            elif abs(error_angle) > 1.5 and abs(error_angle) <= 2.5:
                                rotational_velocity = -0.2*abs(error_angle)
                            else:
                                rotational_velocity = -0.5*abs(error_angle)
                    else:
                        rotational_velocity = -0.01
                    command_vel = 0.0
                    self.publisher.publish_vel(command_vel, rotational_velocity)
                    time.sleep(0.1)
                else:
                    if self.rotate_part1_completed == False:
                        self.rotate_part1_completed = True
                        self.get_logger().info('ROTATE PARTIALLY COMPLETED')
                    else:
                        self.get_logger().info('ROTATE COMPLETED')
                        self.rotating = False
                        self.rotate_clk_completed = True
                        self.target_angle = 0.0
                        self.target_angle1 = 999.0
                        self.target_angle2 = 999.0
                        self.rotate_part1_completed = False
                        self.first_call = True
                        # self.count_rotate_clk_time += 1
                        self.publisher.stop_robot()
                        break

    def move_zigzag_new_version(self):
        while self.move_zigzag_completed == False:
            time.sleep(0.1)
            front_distance = self.laser_msg.ranges[359]
            left_wheel_distance = self.laser_msg.ranges[22]
            right_wheel_distance = self.laser_msg.ranges[332]

            if (front_distance > self.safe_distance or front_distance == 0) and \
                    (left_wheel_distance > self.safe_distance or left_wheel_distance == 0) and \
                    (right_wheel_distance > self.safe_distance or right_wheel_distance == 0):
                        self.can_move_straight = True
            else:
                self.can_move_straight = False

            counter_clk_90_degrees_distance = self.laser_msg.ranges[79]
            clk_90_degrees_distance = self.laser_msg.ranges[269]
            rear_distance = self.laser_msg.ranges[179]
            left_rear_distance = self.laser_msg.ranges[151]
            right_rear_distance = self.laser_msg.ranges[199]

            if (rear_distance > self.safe_distance or rear_distance == 0.0) and \
                    (left_rear_distance > self.safe_distance or left_rear_distance == 0.0)  and \
                        (right_rear_distance > self.safe_distance or right_rear_distance == 0.0):
                        self.can_move_backward = True
            else:
                self.can_move_backward = False

            if counter_clk_90_degrees_distance > self.safe_distance or counter_clk_90_degrees_distance == 0.0:
                self.can_rotate_counter_clk = True
            else:
                self.can_rotate_counter_clk = False

            if clk_90_degrees_distance > self.safe_distance or clk_90_degrees_distance == 0.0:
                self.can_rotate_clk = True
            else:
                self.can_rotate_clk = False

            if self.state == 'STOPPED':
                self.get_logger().info('Stopped')
                if self.can_move_straight and \
                    ((self.not_rotating_clk_yet and self.not_rotating_counter_clk_yet) or \
                    (self.rotated_counter_clk_second_time and self.not_rotating_clk_yet) or \
                    (self.rotated_clk_second_time and self.not_rotating_counter_clk_yet)) and \
                    not self.pausing_move_straight and not self.start_moving_backward_1s:
                        self.get_logger().info('MOVING STRAIGHT')
                        self.state = 'MOVING_STRAIGHT'

                elif self.can_rotate_clk and not self.start_moving_backward_1s and \
                        (self.not_rotating_clk_yet or \
                            (self.move_straight_in_a_period_time_finished and self.rotated_clk_first_time)):
                                self.get_logger().info('ROTATING CLK')
                                self.state = 'ROTATING_CLK'
                                self.rotate_clk_completed = False
                                self.pausing_move_straight = True

                elif self.can_rotate_counter_clk and not self.start_moving_backward_1s and \
                        (self.not_rotating_counter_clk_yet or \
                            (self.move_straight_in_a_period_time_finished and self.rotated_counter_clk_first_time)):
                                self.get_logger().info('ROTATING COUNTER_CLK')
                                self.state = 'ROTATING_COUNTER_CLK'
                                self.rotate_counter_clk_completed = False

                elif (self.rotated_clk_first_time or self.rotated_counter_clk_first_time) and \
                        not self.start_moving_backward_1s:
                            self.publisher.time_up = False
                            self.move_straight_in_a_period_time_finished = False
                            self.get_logger().info('MOVING STRAIGHT 3S')
                            self.state = 'MOVING_STRAIGHT_IN_A_PERIOD_TIME'
                elif ((self.not_rotating_clk_yet and self.not_rotating_counter_clk_yet and \
                        not self.can_rotate_clk and not self.can_rotate_counter_clk) or \
                        (self.rotated_clk_first_time and self.move_straight_in_a_period_time_finished and \
                        not self.can_rotate_clk) or \
                        (self.rotated_counter_clk_first_time and self.move_straight_in_a_period_time_finished and \
                        not self.can_rotate_counter_clk) or \
                        (self.not_rotating_counter_clk_yet and not self.can_rotate_counter_clk) or \
                        (self.not_rotating_clk_yet and not self.can_rotate_clk)) and not self.start_moving_backward_1s:
                            if self.can_move_backward:
                                self.move_backward_finished = False
                                self.pausing_move_straight = True
                                self.get_logger().info('MOVING BACKWARD')
                                self.state = 'MOVING_BACKWARD'

                elif self.start_moving_backward_1s:
                    self.publisher.time_up = False
                    self.get_logger().info('MOVING BACKWARD 1S')
                    self.state = 'MOVING_BACKWARD_1S'
            
            elif self.state == 'ROTATING_COUNTER_CLK':
                if self.rotate_counter_clk_completed:
                    self.pausing_move_straight = False
                    self.publisher.stop_robot()
                    if self.not_rotating_counter_clk_yet:
                        self.not_rotating_counter_clk_yet = False
                        self.rotated_counter_clk_first_time = True
                    elif self.rotated_counter_clk_first_time:
                        self.rotated_counter_clk_first_time = False
                        self.rotated_counter_clk_second_time = True
                        self.move_straight_in_a_period_time_finished = False
                        self.not_rotating_clk_yet = True
                        self.rotated_clk_second_time = False

                    self.state = 'STOPPED'
                else:
                    self.rotate_counter_clockwise(90)                
            
            elif self.state == 'ROTATING_CLK':
                if self.rotate_clk_completed:
                    self.pausing_move_straight = False
                    self.publisher.stop_robot()
                    if self.not_rotating_clk_yet:
                        self.not_rotating_clk_yet = False
                        self.rotated_clk_first_time = True
                    elif self.rotated_clk_first_time:
                        self.rotated_clk_first_time = False
                        self.rotated_clk_second_time = True
                        self.move_straight_in_a_period_time_finished = False
                        self.not_rotating_counter_clk_yet = True
                        self.rotated_counter_clk_second_time = False

                    self.state = 'STOPPED'
                else:
                    self.rotate_clockwise(90)    
                    

            elif self.state == 'MOVING_STRAIGHT':
                if not self.can_move_straight:
                    self.publisher.stop_robot()
                    self.state = 'STOPPED'
                else:
                    self.publisher.move_straight()


            elif self.state == 'MOVING_STRAIGHT_IN_A_PERIOD_TIME':
                if not self.can_move_straight or self.publisher.time_up:
                    self.publisher.stop_robot()
                    self.move_straight_in_a_period_time_finished = True
                    self.state = 'STOPPED'
                else:
                    self.publisher.move_straight_in_a_period_time(3)

            elif self.state == 'MOVING_BACKWARD_1S':
                if self.publisher.time_up:
                    self.move_backward_finished = False
                    self.start_moving_backward_1s = False
                    self.publisher.stop_robot()
                    self.state = 'STOPPED'
                else:
                    self.publisher.move_backward_1s()

            elif self.state == 'MOVING_BACKWARD':
                if not self.can_move_backward:
                    self.publisher.stop_robot()
                    self.is_moving_backward = False
                    self.state = 'FINISHED'
                    self.move_zigzag_completed = True
                    self.get_logger().info('MOVE ZIGZAG COMPLETED')
                else:
                    if (self.not_rotating_clk_yet and self.not_rotating_counter_clk_yet and \
                        self.can_rotate_clk or self.can_rotate_counter_clk) or \
                        (self.rotated_clk_first_time and self.move_straight_in_a_period_time_finished and self.can_rotate_clk) or \
                        (self.rotated_counter_clk_first_time and \
                        self.move_straight_in_a_period_time_finished and self.can_rotate_counter_clk) or \
                        (self.not_rotating_counter_clk_yet and self.can_rotate_counter_clk) or \
                        (self.not_rotating_clk_yet and self.can_rotate_clk):
                            self.publisher.stop_robot()
                            self.is_moving_backward = False
                            self.move_backward_finished = True
                            self.start_moving_backward_1s = True
                            self.state = "STOPPED"
                    else:
                        self.publisher.move_backward()

    def timer_callback(self):
        self.get_logger().info("Timer CallBack")
        try:

            self.move_zigzag_new_version()

            # self.move_zigzag()
            # self.turn_counter_clockwise(179)

            # self.move_straight_and_turn_clockwise()
            # if self.initial_pose_is_set == True:
            #     if self.publisher.ready_to_run == False:
            #         self.publisher.wait_for_10s_before_move()
            #     else:
            #         # self.move_zigzag()
            #         self.move_zigzag_new_version()
        except:
            pass

class PublisherClass(Node):

    def __init__(self):
        super().__init__('pub_node')
        rclpy.logging.set_logger_level('pub_node', rclpy.logging.LoggingSeverity.DEBUG)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()

        self.ready_to_run = False

        self.time_up = False
    
    def move_straight_in_a_period_time(self, period_time):
        for i in range(period_time):
            self.move_straight_1s()
        
        self.time_up = True

    def wait_for_10s_before_move(self):
        self.get_logger().info('In wait_for_10s_before_move function')
        count = 0
        while(count < 10):
            count += 1
            time.sleep(1.0)
        
        self.ready_to_run = True
    
    def stop_robot(self):
        # self.get_logger().info("MOVE STOP")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    def move_straight(self):
        # self.get_logger().info("MOVE STRAIGHT")
        self.cmd.linear.x = 0.07
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    def move_backward(self):
        # self.get_logger().info("MOVE BACK")
        self.cmd.linear.x = -0.07
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    def publish_vel(self,vx,wz):
        self.cmd.linear.x = vx
        self.cmd.angular.z = wz
        self.vel_pub.publish(self.cmd)

    def move_backward_1s(self):
        self.get_logger().info('in move_back_1s function')
        self.cmd.linear.x = -0.07
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)
        time.sleep(3.0)
        self.time_up = True

    def move_straight_1s(self):
        self.cmd.linear.x = 0.07
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)
        time.sleep(1.0)

    def rotate_clockwise_small_angle(self):
        # self.get_logger().info('In rotate_clockwise_small_angle function')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = -0.01
        self.vel_pub.publish(self.cmd)
    
    def rotate_counter_clockwise_small_angle(self):
        # self.get_logger().info('In rotate_counter_clockwise_small_angle function')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.01
        self.vel_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    try:
        subs_node = SubscriberClass()
        pub_node = PublisherClass()
        
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(subs_node)
        executor.add_node(pub_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            subs_node.destroy_node()
            pub_node.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()