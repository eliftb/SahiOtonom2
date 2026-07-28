import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from yolov8_msgs.msg import DetectionWithDepth
from ackermann_msgs.msg import AckermannDriveStamped
import time
from enum import Enum


class VehicleState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    DURAK_CEBINE_GIRME = "durak_cebinde"
    TURNING_RIGHT = "turning_right"
    CLOSE_TURN_RIGHT = "close_turn_right"
    MOVING_FORWARD_AFTER_TURN = "moving_forward_after_turn"
    WAITING_AT_STOP = "waiting_at_stop"
    REVERSE_TURN = "reverse_turn"
    MOVING_AFTER_REVERSE = "moving_after_reverse"
    TURNING_LEFT = "turning_left"
    MOVING_AFTER_LEFT_TURN = "moving_after_left_turn"
    APPROACHING_PARK = "approaching_park"
    PARKING = "parking"
    PARKED = "parked"
    WAITING_AT_TRAFFIC_LIGHT = "waiting_at_traffic_light"


class DecisionMaker(Node):
    def __init__(self):
        super().__init__('decision_maker_ackermann')

        self.declare_parameter('stop_distance', 7.85)
        self.declare_parameter('turn_duration', 11.0)
        self.declare_parameter('default_speed', 1.0)
        self.declare_parameter('turn_speed', 0.4)
        self.declare_parameter('turn_angular_speed', 0.3)
        self.declare_parameter('parking_distance', 4.0)
        self.declare_parameter('wait_at_stop_duration', 15.0)
        self.declare_parameter('reverse_turn_duration', 13.0)
        self.declare_parameter('after_reverse_duration', 3.0)
        self.declare_parameter('max_delta_x', 100.0)
        self.declare_parameter('traffic_light_stop_distance', 6.0)
        self.declare_parameter('park_approach_distance', 10.0)
        self.declare_parameter('park_centering_gain', 0.5)

        self.STOP_DISTANCE = self.get_parameter('stop_distance').value
        self.TURN_DURATION = self.get_parameter('turn_duration').value
        self.DEFAULT_SPEED = self.get_parameter('default_speed').value
        self.TURN_SPEED = self.get_parameter('turn_speed').value
        self.TURN_ANGULAR_SPEED = self.get_parameter('turn_angular_speed').value
        self.PARKING_DISTANCE = self.get_parameter('parking_distance').value
        self.WAIT_AT_STOP_DURATION = self.get_parameter('wait_at_stop_duration').value
        self.REVERSE_TURN_DURATION = self.get_parameter('reverse_turn_duration').value
        self.AFTER_REVERSE_DURATION = self.get_parameter('after_reverse_duration').value
        self.MAX_DELTA_X = self.get_parameter('max_delta_x').value
        self.TRAFFIC_LIGHT_STOP_DISTANCE = self.get_parameter('traffic_light_stop_distance').value
        self.PARK_APPROACH_DISTANCE = self.get_parameter('park_approach_distance').value
        self.PARK_CENTERING_GAIN = self.get_parameter('park_centering_gain').value

        self.current_state = VehicleState.IDLE
        self.previous_state = VehicleState.IDLE
        self.state_change_time = time.time()
        self.close_turn_right_entered_time = None

        self.nearest_sign = None
        self.lateral_deviation = 0.0
        self.intersection_direction = 0
        self.intersection_waiting = False
        self.intersection_detected_time = None
        self.pending_turn_direction = None
        self.park_target_delta_x = 0.0
        self.park_sign_first_seen = False

        self.create_subscription(Float32, '/lane/lateral_deviation', self.lateral_callback, 10)
        self.create_subscription(DetectionWithDepth, '/detection_with_depth', self.depth_callback, 10)
        self.create_subscription(Int32, '/lane/intersection_direction', self.intersection_callback, 10)

        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('🚗 Ackermann Decision Maker is running.')

    def lateral_callback(self, msg):
        self.lateral_deviation = msg.data

    def depth_callback(self, msg):
        if msg.class_name and msg.class_name.lower() != "none":
            self.nearest_sign = {
                'name': msg.class_name.lower(),
                'depth': msg.depth_value,
                'delta_x': msg.delta_x if hasattr(msg, 'delta_x') else 0.0
            }

            if (self.nearest_sign['name'] == 'park'
                    and not self.park_sign_first_seen
                    and self.nearest_sign['depth'] <= 15.0):
                self.park_sign_first_seen = True
                self.park_target_delta_x = self.nearest_sign['delta_x']
                self.get_logger().info('🅿️ Park sign locked.')

    def intersection_callback(self, msg):
        if self.current_state in [VehicleState.APPROACHING_PARK, VehicleState.PARKING, VehicleState.PARKED]:
            return

        self.intersection_direction = msg.data
        if self.intersection_direction in [1, 2] and not self.intersection_waiting:
            self.intersection_waiting = True
            self.intersection_detected_time = time.time()
            self.pending_turn_direction = self.intersection_direction

    def change_state(self, new_state):
        if self.current_state != new_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_change_time = time.time()
            if new_state == VehicleState.CLOSE_TURN_RIGHT:
                self.close_turn_right_entered_time = time.time()
            self.get_logger().info(f'🔄 State changed: {self.previous_state.value} ➜ {new_state.value}')

    def get_state_duration(self):
        return time.time() - self.state_change_time

    def should_stop_for_red_or_yellow_light(self):
        if not self.nearest_sign:
            return False
        return (self.nearest_sign['name'] in ['kirmizi-isik', 'sari-isik']
                and self.nearest_sign['depth'] <= self.TRAFFIC_LIGHT_STOP_DISTANCE)

    def control_loop(self):
        next_state = self.decide_next_state()
        self.change_state(next_state)
        self.execute_state_action()

    def decide_next_state(self):
        elapsed = self.get_state_duration()

        if self.park_sign_first_seen:
            if self.current_state not in [VehicleState.APPROACHING_PARK, VehicleState.PARKING, VehicleState.PARKED]:
                return VehicleState.APPROACHING_PARK

            if self.current_state == VehicleState.APPROACHING_PARK:
                if self.nearest_sign and self.nearest_sign['depth'] <= self.PARKING_DISTANCE:
                    return VehicleState.PARKING
                return VehicleState.APPROACHING_PARK

            if self.current_state == VehicleState.PARKING:
                if self.nearest_sign and self.nearest_sign['depth'] <= 1.5:
                    return VehicleState.PARKED
                return VehicleState.PARKING

            if self.current_state == VehicleState.PARKED:
                return VehicleState.PARKED

        if self.current_state == VehicleState.WAITING_AT_TRAFFIC_LIGHT:
            if (self.nearest_sign and self.nearest_sign['name'] == 'yesil-isik') or not self.nearest_sign:
                return VehicleState.MOVING
            return VehicleState.WAITING_AT_TRAFFIC_LIGHT

        if self.current_state == VehicleState.MOVING and self.should_stop_for_red_or_yellow_light():
            return VehicleState.WAITING_AT_TRAFFIC_LIGHT

        if self.current_state == VehicleState.CLOSE_TURN_RIGHT:
            if time.time() - self.close_turn_right_entered_time >= 6.0:
                return VehicleState.MOVING_FORWARD_AFTER_TURN
            return VehicleState.CLOSE_TURN_RIGHT

        if self.intersection_waiting:
            if time.time() - self.intersection_detected_time >= 5.0:
                self.intersection_waiting = False
                return (VehicleState.TURNING_LEFT if self.pending_turn_direction == 1
                        else VehicleState.TURNING_RIGHT)
            return VehicleState.MOVING

        if self.current_state == VehicleState.TURNING_RIGHT and elapsed >= self.TURN_DURATION:
            return VehicleState.MOVING_FORWARD_AFTER_TURN

        if self.current_state == VehicleState.TURNING_LEFT and elapsed >= self.TURN_DURATION:
            return VehicleState.MOVING_AFTER_LEFT_TURN

        if self.current_state == VehicleState.MOVING_FORWARD_AFTER_TURN and elapsed >= 3.0:
            return VehicleState.MOVING

        if self.current_state == VehicleState.MOVING_AFTER_LEFT_TURN:
            return VehicleState.MOVING

        return VehicleState.MOVING

    def execute_state_action(self):
        msg = AckermannDriveStamped()
        drive = msg.drive

        if self.current_state == VehicleState.MOVING:
            drive.speed = self.DEFAULT_SPEED
            drive.steering_angle = -0.4 * self.lateral_deviation

        elif self.current_state == VehicleState.APPROACHING_PARK:
            drive.speed = self.DEFAULT_SPEED * 0.5
            if self.nearest_sign and self.nearest_sign['name'] == 'park':
                drive.steering_angle = -self.PARK_CENTERING_GAIN * self.nearest_sign['delta_x'] / 100.0

        elif self.current_state == VehicleState.PARKING:
            drive.speed = self.DEFAULT_SPEED * 0.2
            if self.nearest_sign:
                drive.steering_angle = -0.01 * self.nearest_sign['delta_x']

        elif self.current_state == VehicleState.PARKED:
            drive.speed = 0.0
            drive.steering_angle = 0.0

        elif self.current_state == VehicleState.WAITING_AT_TRAFFIC_LIGHT:
            drive.speed = 0.0
            drive.steering_angle = 0.0

        elif self.current_state == VehicleState.TURNING_RIGHT:
            drive.speed = self.TURN_SPEED
            drive.steering_angle = abs(self.TURN_ANGULAR_SPEED)

        elif self.current_state == VehicleState.TURNING_LEFT:
            drive.speed = self.TURN_SPEED
            drive.steering_angle = -abs(self.TURN_ANGULAR_SPEED)

        elif self.current_state == VehicleState.CLOSE_TURN_RIGHT:
            elapsed = time.time() - self.close_turn_right_entered_time
            drive.speed = self.TURN_SPEED * 1.5 if elapsed > 4.0 else self.DEFAULT_SPEED
            drive.steering_angle = abs(self.TURN_ANGULAR_SPEED) * (2.0 if elapsed > 4.0 else 0.0)

        elif self.current_state == VehicleState.MOVING_FORWARD_AFTER_TURN:
            drive.speed = self.DEFAULT_SPEED * 0.5
            drive.steering_angle = -0.3 * self.lateral_deviation

        self.ackermann_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionMaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
