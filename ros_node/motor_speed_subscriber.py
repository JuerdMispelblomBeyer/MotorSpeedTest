import rospy
import serial
from exr2_interfaces.msg import TrackKinematics
from exr2_interfaces.msg import SetSpeedref
from exr2_interfaces.msg import MotorSpeedTorque
from geometry_msgs.msg import Twist

TOPIC_TRACK_KINEMATICS = "/exr_200/serial_hardware_interface/track_kinematics"
TOPIC_SET_SPEED_REF = "/exr_200/serial_hardware_interface/msgs/out/set_speed_ref"
TOPIC_MOTOR_SPEED_TORQUE_LEFT = "/exr_200/serial_hardware_interface/msgs/out/motor_speed_torque/left"
TOPIC_MOTOR_SPEED_TORQUE_RIGHT = "/exr_200/serial_hardware_interface/msgs/out/motor_speed_torque/right"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 115200

def noop_callback(_msg):
    pass

def listen_and_publish():
    """
    1. Initialize subscribers to:
          - TrackKinematics   - Actual speed of motors in rad/s
          - MotorSpeedTorque  - Rotational speed of motors in krpm
          - SetSpeedRef       - Outbound command to motors in krpm
    2. Initialize serial connection and start listening on /dev/ttyUSB0
    2. Init pub & start publishing a speed to /cmd_vel
    3. Log to file:
          - Linear speed cmd to /cmd_vel
          - Resulting motorspeed in rad/s as measured by ROS
          - Resulting motorspeed in rad/s as measured by Arduino (from Serial)
    """
    rospy.init_node('motor_speed_checker', anonymous=True)
    rate = rospy.Rate(10)
    last_rad_s_ros = 0.  # This will be updated from the subscriber
    last_rad_s_arduino = 0.  # This will be updated from the serial connection

    def track_kinematics_callback(msg):
        nonlocal last_rad_s_ros
        last_rad_s_ros = msg.motor_rotational_speed_l

    rospy.Subscriber(TOPIC_TRACK_KINEMATICS, TrackKinematics, track_kinematics_callback)
    rospy.Subscriber(TOPIC_SET_SPEED_REF, SetSpeedref, noop_callback)
    rospy.Subscriber(TOPIC_MOTOR_SPEED_TORQUE_LEFT, MotorSpeedTorque, noop_callback)
    rospy.Subscriber(TOPIC_MOTOR_SPEED_TORQUE_RIGHT, MotorSpeedTorque, noop_callback)

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0

    # Tokenize the output (comma separated), update last_rad_s_arduino with the second token
    def serial_callback(msg):
        nonlocal last_rad_s_arduino
        tokens = msg.split(",")
        if len(tokens) > 1:
            try:
                last_rad_s_arduino = float(tokens[1])
            except ValueError:
                pass

    ser = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUDRATE, timeout=0)

    while not rospy.is_shutdown():
        if ser.in_waiting:
            serial_callback(ser.readline().decode("utf-8", errors="ignore").strip())
        pub_cmd_vel.publish(cmd_vel)
        logstr = (f"cmd_vel_lin_x {cmd_vel.linear.x} "f"motor_rad_s_ros {last_rad_s_ros} "f"motor_rad_s_arduino {last_rad_s_arduino}")
        rospy.loginfo(logstr)
        rate.sleep()

if __name__ == "__main__":
    try:
        listen_and_publish()
    except rospy.ROSInterruptException:
        pass
