"""Actuator bridge node: cmd_vel -> Teensy UDP.

Ports logic from AV2.1-API:
  - actuators/udp.py       (UDP socket, keepalive, command parsing)
  - control/pid.py         (PID speed controller)
  - control/ackermann_vehicle.py (bicycle model inverse kinematics)

Subscribes:
  /cmd_vel                  geometry_msgs/Twist
  /avros/actuator_command   avros_msgs/ActuatorCommand (e-stop override)

Publishes:
  /avros/actuator_state     avros_msgs/ActuatorState @ 20Hz
"""

import math
import socket
import json
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from avros_msgs.msg import ActuatorCommand, ActuatorState


class PID:
    """PID controller with anti-windup. Ported from control/pid.py."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0

        p = self.kp * error

        self._integral += error * dt
        i = self.ki * self._integral

        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = p + i + d
        output = max(self.output_min, min(self.output_max, output))

        # Anti-windup: clamp integral if output saturated
        if output == self.output_max or output == self.output_min:
            self._integral -= error * dt

        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


class ActuatorNode(Node):
    """ROS2 node bridging cmd_vel to Teensy UDP actuator protocol."""

    def __init__(self):
        super().__init__('actuator_node')

        # Declare parameters
        self.declare_parameter('teensy_ip', '192.168.13.177')
        self.declare_parameter('teensy_port', 5005)
        self.declare_parameter('keepalive_interval', 0.2)
        self.declare_parameter('wheelbase', 1.23)
        self.declare_parameter('max_steering_rad', 0.489)
        self.declare_parameter('steering_sign', -1)
        self.declare_parameter('pid_kp', 0.55)
        self.declare_parameter('pid_ki', 0.055)
        self.declare_parameter('pid_kd', 0.08)
        self.declare_parameter('max_throttle', 0.6)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)

        # Read parameters
        self._teensy_ip = self.get_parameter('teensy_ip').value
        self._teensy_port = self.get_parameter('teensy_port').value
        self._keepalive_interval = self.get_parameter('keepalive_interval').value
        self._wheelbase = self.get_parameter('wheelbase').value
        self._max_steering_rad = self.get_parameter('max_steering_rad').value
        self._steering_sign = self.get_parameter('steering_sign').value
        self._max_throttle = self.get_parameter('max_throttle').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        control_rate = self.get_parameter('control_rate').value

        # PID for speed control (linear.x -> throttle/brake)
        self._speed_pid = PID(
            kp=self.get_parameter('pid_kp').value,
            ki=self.get_parameter('pid_ki').value,
            kd=self.get_parameter('pid_kd').value,
            output_min=-1.0,
            output_max=1.0,
        )

        # UDP socket
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(0.1)
        self._socket.bind(('', 0))
        self._lock = threading.Lock()
        self.get_logger().info(
            f'UDP actuator: {self._teensy_ip}:{self._teensy_port}'
        )

        # Internal state
        self._throttle = 0.0
        self._brake = 0.0
        self._steer = 0.0
        self._mode = 'N'
        self._estop = False
        self._watchdog_active = False
        self._last_cmd_vel_time = self.get_clock().now()
        self._last_linear_x = 0.0

        # Actual state from Teensy response
        self._actual_estop = False
        self._actual_throttle = 0.0
        self._actual_mode = 'N'
        self._actual_brake = 0.0
        self._actual_steer = 0.0

        # Subscribers
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        self._actuator_cmd_sub = self.create_subscription(
            ActuatorCommand, '/avros/actuator_command',
            self._actuator_cmd_callback, 10
        )

        # Publisher
        self._state_pub = self.create_publisher(
            ActuatorState, '/avros/actuator_state', 10
        )

        # Control loop timer
        self._dt = 1.0 / control_rate
        self._timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info('Actuator node started')

    def _cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to steering + speed command."""
        self._last_cmd_vel_time = self.get_clock().now()
        self._last_linear_x = msg.linear.x

        v = msg.linear.x
        omega = msg.angular.z

        # Ackermann inverse: steering_rad = atan2(omega * wheelbase, v)
        if abs(v) > 0.01:
            steering_rad = math.atan2(omega * self._wheelbase, abs(v))
        elif abs(omega) > 0.01:
            # Turning in place: use max steering in direction of omega
            steering_rad = math.copysign(self._max_steering_rad, omega)
        else:
            steering_rad = 0.0

        # Clip to mechanical limits
        steering_rad = max(-self._max_steering_rad,
                           min(self._max_steering_rad, steering_rad))

        # Normalize to [-1, 1] and apply hardware sign convention
        self._steer = (steering_rad / self._max_steering_rad) * self._steering_sign

        # Set mode based on direction
        if v < -0.01:
            self._mode = 'R'
        elif v > 0.01:
            if self._mode not in ('D', 'S'):
                self._mode = 'D'
        else:
            pass  # keep current mode

    def _actuator_cmd_callback(self, msg: ActuatorCommand):
        """Handle direct actuator commands (e-stop override)."""
        if msg.estop:
            self._estop = True
            self._throttle = 0.0
            self._brake = 1.0
            self._speed_pid.reset()
            self.get_logger().warn('E-STOP activated via actuator_command')
        else:
            self._estop = msg.estop
            self._throttle = max(0.0, min(1.0, msg.throttle))
            self._brake = max(0.0, min(1.0, msg.brake))
            self._steer = max(-1.0, min(1.0, msg.steer))
            if msg.mode in ('N', 'D', 'S', 'R'):
                self._mode = msg.mode

    def _control_loop(self):
        """Main control loop at control_rate Hz."""
        now = self.get_clock().now()

        # Check cmd_vel timeout
        dt_since_cmd = (now - self._last_cmd_vel_time).nanoseconds / 1e9
        if dt_since_cmd > self._cmd_vel_timeout:
            # No cmd_vel received — brake to stop
            self._throttle = 0.0
            self._brake = 1.0
            self._steer = 0.0
            self._speed_pid.reset()
        else:
            # PID speed control: error = desired_speed - 0 (no odometry feedback yet)
            # When odometry is available, replace 0 with actual speed
            pid_output = self._speed_pid.compute(self._last_linear_x, self._dt)

            if pid_output >= 0:
                self._throttle = min(pid_output, self._max_throttle)
                self._brake = 0.0
            else:
                self._throttle = 0.0
                self._brake = min(abs(pid_output), 1.0)

        # Send to Teensy
        self._send_all()

        # Publish state
        state_msg = ActuatorState()
        state_msg.header.stamp = now.to_msg()
        state_msg.header.frame_id = 'base_link'
        # Publish actual Teensy state (falls back to commanded if no response)
        state_msg.estop = self._actual_estop
        state_msg.throttle = self._actual_throttle
        state_msg.mode = self._actual_mode
        state_msg.brake = self._actual_brake
        state_msg.steer = self._actual_steer
        state_msg.watchdog_active = self._watchdog_active
        self._state_pub.publish(state_msg)

    def _send_all(self):
        """Send all-in-one command to Teensy via UDP."""
        cmd = (
            f"A E={1 if self._estop else 0} "
            f"T={self._throttle:.3f} "
            f"M={self._mode} "
            f"B={self._brake:.3f} "
            f"S={self._steer:.3f}"
        )
        data = (cmd + '\n').encode('ascii')

        with self._lock:
            try:
                self._socket.sendto(
                    data, (self._teensy_ip, self._teensy_port)
                )
                try:
                    response, _ = self._socket.recvfrom(256)
                    resp_str = response.decode('ascii').strip()
                    if resp_str.startswith('{'):
                        resp = json.loads(resp_str)
                        # Update state from Teensy response (actual values)
                        self._actual_estop = bool(resp.get('e', 0))
                        self._actual_throttle = float(resp.get('t', 0.0))
                        self._actual_mode = str(resp.get('m', 'N'))
                        self._actual_brake = float(resp.get('b', 0.0))
                        self._actual_steer = float(resp.get('s', 0.0))
                        self._watchdog_active = bool(resp.get('w', 0))
                except (socket.timeout, json.JSONDecodeError):
                    pass
            except Exception as e:
                self.get_logger().error(f'UDP send error: {e}')

    def destroy_node(self):
        """Clean shutdown: e-stop and close socket."""
        self.get_logger().info('Shutting down — sending e-stop')
        self._estop = True
        self._throttle = 0.0
        self._brake = 1.0
        try:
            self._send_all()
        except Exception:
            pass
        if self._socket:
            self._socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
