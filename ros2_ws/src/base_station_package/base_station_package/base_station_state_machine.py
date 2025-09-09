#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from statemachine import State, StateMachine
from shared_interfaces.msg import DroneState

class BaseStationStateMachine(StateMachine):
    """State machine definitions"""

    # Define States
    idle = State(initial=True)
    arms_centred = State()
    arms_not_centred = State()
    charging = State()
    doors_closed = State()
    doors_open = State()
    drone_landed = State()
    ready_for_takeoff = State()
    in_flight = State()

    # Define Transitions
    uncentring_arms = idle.to(arms_not_centred)
    doors_opening = arms_not_centred.to(doors_open)
    uploading_mission = doors_open.to(ready_for_takeoff)
    takeoff = ready_for_takeoff.to(in_flight)
    landing = in_flight.to(drone_landed)
    centring_arms = drone_landed.to(arms_centred)
    start_charging = arms_centred.to(charging)
    closing_doors = charging.to(doors_closed)
    moving_to_idle = doors_closed.to(idle)

class BaseStationStateMachineNode(Node):
    def __init__(self):
        super().__init__('base_station_state_machine')
        self.state_machine = BaseStationStateMachine(model=self)

        # Subscriber and Publisher Declarations
        self.drone_state_subscription = self.create_subscription(
            DroneState,
            'drone/state',
            self.drone_state_callback,
            10
        )

        self.drone_state = DroneState()

    def drone_state_callback(self, msg):
        self.drone_state = msg
        self.get_logger().info(f'Received Drone State: {msg.current_state}')
        

def main():
    rclpy.init()
    
    node = BaseStationStateMachineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Base Station State Machine...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()