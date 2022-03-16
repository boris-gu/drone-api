from re import A
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from threading import Thread
from math import pi, atan2, sqrt


class Drone_api:
    def __init__(self):
        # GENERAL attributes
        self.__started = False
        self.__type_of_move = None  # 'LOCAL_POSE' or 'GLOBAL_POSE' or 'VELOCITY'
        self.__current_state = State()
        self.__current_local_pose = PoseStamped()
        self.__current_global_pose = NavSatFix()
        self.__thread_command = None
        self.__yaw_head_first = False

        # LOCAL_POSE attributes
        self.__allowable_error = 0.5  # m
        self.__last_command_local_pose = PoseStamped()
        self.__last_command_local_pose.pose.orientation.w = 1  # correct quaternion

        # GLOBAL_POSE attributes
        self.__last_command_global_pose = GeoPoseStamped()
        self.__last_command_global_pose.pose.orientation.w = 1

        # VELOCITY attributes
        self.__last_command_vel = State()

        rospy.init_node('drone_offb', anonymous=True)
        # get state, position, set position, velocity, mode and arm
        self.__state_sub = rospy.Subscriber('mavros/state', State,
                                            self.__state_cb, queue_size=10)
        self.__local_pos_sub = rospy.Subscriber('/mavros/local_position/pose',
                                                PoseStamped, self.__local_pos_cb, queue_size=10)
        self.__global_pos_sub = rospy.Subscriber('/mavros/global_position/global',
                                                 NavSatFix, self.__global_pos_cb, queue_size=10)
        self.__local_pos_pub = rospy.Publisher('mavros/setpoint_position/local',
                                               PoseStamped, queue_size=10)
        self.__global_pos_pub = rospy.Publisher('mavros/setpoint_position/global',
                                                GeoPoseStamped, queue_size=10)
        self.__local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',
                                               Twist, queue_size=10)
        self.__set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.__arming_client = rospy.ServiceProxy('mavros/cmd/arming',
                                                  CommandBool)

    def __state_cb(self, state):
        self.__current_state = state

    def __local_pos_cb(self, pose):
        self.__current_local_pose = pose

    def __global_pos_cb(self, pose):
        self.__current_global_pose = pose

    def __command_target(self):
        rate = rospy.Rate(20)
        # Waiting for communication between MAVROS and autopilot
        while not rospy.is_shutdown() and not self.__current_state.connected:
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass
        last_request = rospy.Time.now()
        while self.__started and not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                # OFFBOARD and ARM
                if now - last_request > rospy.Duration(3):
                    last_request = now
                    if self.__current_state.mode != 'OFFBOARD':
                        self.__set_mode_client(base_mode=0,
                                               custom_mode='OFFBOARD')
                    elif not self.__current_state.armed:
                        # fixme: дрон бесконечно дизармится, если не взлететь
                        self.__arming_client(True)
                # CONTROL
                if self.__type_of_move == 'LOCAL_POSE':
                    delta_x = (self.__last_command_local_pose.pose.position.x
                               - self.__current_local_pose.pose.position.x)
                    delta_y = (self.__last_command_local_pose.pose.position.y
                               - self.__current_local_pose.pose.position.y)
                    distance = sqrt((delta_x)**2 + (delta_y)**2)
                    if self.__yaw_head_first and distance > self.__allowable_error:
                        yaw = atan2(delta_y, delta_x)
                        q = quaternion_from_euler(0, 0, yaw)
                        self.__last_command_local_pose.pose.orientation.x = q[0]
                        self.__last_command_local_pose.pose.orientation.y = q[1]
                        self.__last_command_local_pose.pose.orientation.z = q[2]
                        self.__last_command_local_pose.pose.orientation.w = q[3]
                    self.__local_pos_pub.publish(
                        self.__last_command_local_pose)
                elif self.__type_of_move == 'GLOBAL_POSE':
                    delta_x = (self.__last_command_local_pose.pose.position.x
                               - self.__current_local_pose.pose.position.x)
                    delta_y = (self.__last_command_local_pose.pose.position.y
                               - self.__current_local_pose.pose.position.y)
                    distance = sqrt((delta_x)**2 + (delta_y)**2)
                    if self.__yaw_head_first and distance > self.__allowable_error:
                        yaw = atan2(delta_y, delta_x)
                        q = quaternion_from_euler(0, 0, yaw)
                        self.__last_command_global_pose.pose.orientation.x = q[0]
                        self.__last_command_global_pose.pose.orientation.y = q[1]
                        self.__last_command_global_pose.pose.orientation.z = q[2]
                        self.__last_command_global_pose.pose.orientation.w = q[3]
                    self.__global_pos_pub.publish(
                        self.__last_command_global_pose)
                elif self.__type_of_move == 'VELOCITY':
                    self.__local_vel_pub.publish(self.__last_command_vel)
                rate.sleep()
            except rospy.exceptions.ROSException:
                pass

    # GENERAL methods
    def start(self):
        if not self.__started:
            self.__started = True
            self.__type_of_move = 'LOCAL_POSE'

            if self.__thread_command is None or not self.__thread_command.is_alive():
                self.__thread_command = Thread(target=self.__command_target,
                                               daemon=True)
                self.__thread_command.start()

    def stop(self):
        if self.__started:
            self.__started = False

    def sleep(self, time: float):
        try:
            rospy.sleep(time)
        except rospy.ROSInterruptException:
            pass

    def is_shutdown(self):
        return rospy.is_shutdown()

    # LOCAL_POSE methods
    @property
    def allowable_error(self):
        return self.__allowable_error

    @allowable_error.setter
    def allowable_error(self, distance: float):
        self.__allowable_error = distance

    def get_local_pose(self):
        x = self.__current_local_pose.pose.position.x
        y = self.__current_local_pose.pose.position.y
        z = self.__current_local_pose.pose.position.z
        angels = euler_from_quaternion([self.__current_local_pose.pose.orientation.x,
                                        self.__current_local_pose.pose.orientation.y,
                                        self.__current_local_pose.pose.orientation.z,
                                        self.__current_local_pose.pose.orientation.w])
        roll = angels[0]
        pitch = angels[1]
        yaw = angels[2]
        return x, y, z, roll, pitch, yaw

    def set_local_pose(self, x: float = None, y: float = None, z: float = None,
                       yaw: float = None, yaw_head_first: bool = False):
        if self.__started == False:
            raise Exception('Drone is not running. '
                            'Please run \'<object_name>.start()\'')
        new_pose = PoseStamped()
        if x is None:
            x = self.__last_command_local_pose.pose.position.x
        if y is None:
            y = self.__last_command_local_pose.pose.position.y
        if z is None:
            z = self.__last_command_local_pose.pose.position.z
        new_pose.pose.position.x = x
        new_pose.pose.position.y = y
        new_pose.pose.position.z = z
        if yaw is None:
            new_pose.pose.orientation = self.__last_command_local_pose.pose.orientation
        else:
            q = quaternion_from_euler(0, 0, yaw)
            new_pose.pose.orientation.x = q[0]
            new_pose.pose.orientation.y = q[1]
            new_pose.pose.orientation.z = q[2]
            new_pose.pose.orientation.w = q[3]
        print(new_pose.pose.orientation)
        self.__last_command_local_pose = new_pose
        self.__yaw_head_first = yaw_head_first
        self.__type_of_move = 'LOCAL_POSE'

    def point_is_reached(self):
        if self.__started == False:
            raise Exception('Drone is not running. '
                            'Please run \'<object_name>.start()\'')
        delta_x = (self.__last_command_local_pose.pose.position.x
                   - self.__current_local_pose.pose.position.x)
        delta_y = (self.__last_command_local_pose.pose.position.y
                   - self.__current_local_pose.pose.position.y)
        delta_z = (self.__last_command_local_pose.pose.position.z
                   - self.__current_local_pose.pose.position.z)
        distance = sqrt((delta_x)**2 + (delta_y)**2 + (delta_z)**2)
        if distance < self.__allowable_error:
            return True
        else:
            return False

    # GLOBAL_POSE methods
    def get_global_pose(self):
        latitude = self.__current_global_pose.latitude
        longitude = self.__current_global_pose.longitude
        altitude = self.__current_global_pose.altitude
        return latitude, longitude, altitude

    def set_global_pose(self, latitude: float = None, longitude: float = None, altitude: float = None,
                        yaw: float = None, yaw_head_first: bool = False):
        if self.__started == False:
            raise Exception('Drone is not running. '
                            'Please run \'<object_name>.start()\'')
        new_pose = GeoPoseStamped()
        if latitude is None:
            latitude = self.__last_command_global_pose.latitude
        if longitude is None:
            longitude = self.__last_command_global_pose.longitude
        if altitude is None:
            altitude = self.__last_command_global_pose.altitude
        new_pose.pose.position.latitude = latitude
        new_pose.pose.position.longitude = longitude
        new_pose.pose.position.altitude = altitude
        if yaw is None:
            new_pose.pose.orientation = self.__last_command_global_pose.pose.orientation
        else:
            q = quaternion_from_euler(0, 0, yaw)
            new_pose.pose.orientation.x = q[0]
            new_pose.pose.orientation.y = q[1]
            new_pose.pose.orientation.z = q[2]
            new_pose.pose.orientation.w = q[3]
        print(new_pose.pose.orientation)
        self.__yaw_head_first = yaw_head_first
        self.__last_command_global_pose = new_pose
        self.__type_of_move = 'GLOBAL_POSE'

    # VELOCITY velocity methods
    def set_velocity(self, x: float = 0, y: float = 0, z: float = 0, yaw: float = 0):
        if self.__started == False:
            raise Exception('Drone is not running. '
                            'Please run \'<object_name>.start()\'')
        new_vel = Twist()
        if x is None:
            x = self.__last_command_local_pose.pose.position.x
        if y is None:
            y = self.__last_command_local_pose.pose.position.y
        if z is None:
            z = self.__last_command_local_pose.pose.position.z
        if yaw is None:
            yaw = self.__last_command_vel.angular.z
        new_vel.linear.x = x
        new_vel.linear.y = y
        new_vel.linear.z = z
        new_vel.angular.z = yaw
        self.__last_command_vel = new_vel
        self.__type_of_move = 'VELOCITY'
