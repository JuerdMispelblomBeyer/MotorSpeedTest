#!/usr/bin/env python3

import math
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SensorDataQos
from exr2_interfaces.msg import MotorSpeedTorque
from exr2_interfaces.msg import SetSpeedref
from exr2_interfaces.msg import TrackKinematics

TOPIC_TRACK_KINEMATICS = "/exr_200/serial_hardware_interface/track_kinematics"
TOPIC_SET_SPEED_REF = "/exr_200/serial_hardware_interface/msgs/out/set_speed_ref"
TOPIC_MOTOR_SPEED_TORQUE_LEFT = "/exr_200/serial_hardware_interface/msgs/in/motor_speed_torque/left"
TOPIC_MOTOR_SPEED_TORQUE_RIGHT = "/exr_200/serial_hardware_interface/msgs/in/motor_speed_torque/right"

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 115200
PUBLISH_RATE_HZ = 10.0


class MotorSpeedChecker(Node):
    """ROS 2 node that monitors motor speed topics and serial telemetry."""

    def __init__(self) -> None:
        super().__init__("motor_speed_checker")

        # Gearbox ratio converts axis speed to motor speed.
        # Example: gearbox_ratio=50 means motor speed = axis speed * 50.
        self.declare_parameter("gearbox_ratio", 1.0)
        self.gearbox_ratio = float(self.get_parameter("gearbox_ratio").value)

        self.track_kinematics_left_motor_rad_s = 0.0
        self.track_kinematics_left_axis_rad_s = 0.0
        self.set_speed_ref_left_motor_rad_s = 0.0
        self.motor_speed_torque_left_motor_rad_s = 0.0
        self.arduino_left_axis_rad_s = 0.0
        self.arduino_left_motor_rad_s = 0.0

        # Placeholder until diff_drive output is available.
        # NOTE: this is AXIS speed in rad/s and must be scaled by gearbox_ratio.
        self.diff_drive_left_axis_rad_s_raw = 0.0
        self.diff_drive_left_motor_rad_s = 0.0

        self.create_subscription(TrackKinematics, TOPIC_TRACK_KINEMATICS, self.track_kinematics_callback, SensorDataQos)
        self.create_subscription(SetSpeedref, TOPIC_SET_SPEED_REF, self.set_speed_ref_callback, SensorDataQos)
        self.create_subscription(MotorSpeedTorque, TOPIC_MOTOR_SPEED_TORQUE_LEFT, self.motor_speed_torque_left_callback, SensorDataQos)
        self.create_subscription(MotorSpeedTorque, TOPIC_MOTOR_SPEED_TORQUE_RIGHT, self.noop_callback, SensorDataQos)
        self.ser = None

        try:
            self.ser = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUDRATE, timeout=0)
            self.get_logger().info(f"Opened serial port {SERIAL_PORT} @ {SERIAL_BAUDRATE} baud")
        except serial.SerialException as exc:
            self.get_logger().warning(f"Failed to open serial port {SERIAL_PORT}: {exc}. Continuing without serial input."
                                      )

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.timer_callback)

    def noop_callback(self, _msg) -> None:
        pass

    def track_kinematics_callback(self, msg: TrackKinematics) -> None:
        # TrackKinematics motor rotational speed (motor side, not axis side).
        self.track_kinematics_left_motor_rad_s = float(getattr(msg, "motor_rotational_speed_l", 0.0))
        self.track_kinematics_left_axis_rad_s = float(getattr(msg, "track_axis_rotational_speed_l", 0.0))
        self.diff_drive_left_axis_rad_s_raw = self.track_kinematics_left_axis_rad_s

    def set_speed_ref_callback(self, msg: SetSpeedref) -> None:
        # SetSpeedRef left speed is in kRPM; convert to rad/s.
        # Field name may differ across interface versions, so try common variants.
        left_krpm = self._get_first_attr(msg, ("speed_l", "speed_ref_l", "motor_speed_ref_l", "left_speed_ref", "left"), 0.0,)
        self.set_speed_ref_left_motor_rad_s = self.krpm_to_rad_s(left_krpm)

    def motor_speed_torque_left_callback(self, msg: MotorSpeedTorque) -> None:
        self.motor_speed_torque_left_motor_rad_s = self.krpm_to_rad_s(self._get_first_attr(msg, ("speed", "speed_l", "left_speed"), 0.0))

    def serial_callback(self, line: str) -> None:
        tokens = line.split(",")
        if len(tokens) > 1:
            try:
                # Arduino value is axis speed in rad/s; scale to motor speed.
                self.arduino_left_axis_rad_s = float(tokens[1])
                self.arduino_left_motor_rad_s = (
                    self.arduino_left_axis_rad_s * self.gearbox_ratio)
            except ValueError:
                pass

    @staticmethod
    def krpm_to_rad_s(value_krpm: float) -> float:
        return float(value_krpm) * 1000.0 * (2.0 * math.pi / 60.0)

    @staticmethod
    def _get_first_attr(msg, names: tuple[str, ...], default: float) -> float:
        for name in names:
            if hasattr(msg, name):
                return float(getattr(msg, name))
        return default

    def timer_callback(self) -> None:
        if self.ser is not None and self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            self.serial_callback(line)

        # Placeholder conversion from diff_drive AXIS speed -> MOTOR speed.
        self.diff_drive_left_motor_rad_s = (self.diff_drive_left_axis_rad_s_raw * self.gearbox_ratio)

        self.get_logger().info(
            f"dd[axis] {self.diff_drive_left_axis_rad_s_raw:.2f} "
            # f"dd_motor {self.diff_drive_left_motor_rad_s} "
            f"setpoint[motor] {self.set_speed_ref_left_motor_rad_s:.2f} "
            f"msg[motor] {self.track_kinematics_left_motor_rad_s:.2f} "
            # f"mst_motor {self.motor_speed_torque_left_motor_rad_s:.2f} "
            f"measured[motor] {self.arduino_left_motor_rad_s:.2f}"
        )

    def destroy_node(self) -> bool:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorSpeedChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
