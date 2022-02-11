import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class Drone_api:
    def __init__(self):
        self.offboard = False
        self.arm = False
        self.state = State()
        rospy.init_node('drone_offb', anonymous=True)
        # Подписчик состояния, издатель локальной позиции, клиенты для смены режима и арма
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_cb, queue_size=10)
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local',
                                             PoseStamped, queue_size=10)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy(
            'mavros/cmd/arming', CommandBool)

    def state_cb(self, state):
        self.current_state = state

    def offboard(self):
        last_request = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.current_state.mode != 'OFFBOARD' and (now - last_request > rospy.Duration(5)):
                self.set_mode_client(base_mode=0, custom_mode='OFFBOARD')
                rospy.loginfo('Offboard enabled')
                last_request = now
