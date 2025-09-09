#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from statemachine import State, StateMachine
from shared_interfaces.msg import DroneState
from shared_interfaces.srv import PrepareForMission

class BaseStationStateMachine(StateMachine):
    """State machine definitions"""

    # Define States
    idle = State(initial=True)
    preparing_drone = State()
    arms_centred = State()
    arms_not_centred = State()
    charging = State()
    doors_closed = State()
    doors_open = State()
    drone_landed = State()
    ready_for_takeoff = State()
    in_flight = State()

    # Define Transitions
    prepare_drone = idle.to(preparing_drone)
    uncentring_arms = preparing_drone.to(arms_not_centred)
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

        # Service Clients Declarations (Drone_state_machine communication)
        self.prepare_mission_client = self.create_client(
            PrepareForMission,
            'drone/prepare_for_mission'
        )

        # Timer for testing - triggers mission after 5 seconds
        self.create_timer(5.0, self.trigger_test_mission)
        self.mission_triggered = False  # Flag to only trigger once

        self.drone_state = DroneState()

    def drone_state_callback(self, msg):
        self.drone_state = msg
        #self.get_logger().info(f'Received Drone State: {msg.current_state}')

    def prepare_drone_for_mission(self, mission_id, waypoints):
        """Calls drone service to prepare for mission"""
        if not self.prepare_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Drone prepare service not available')
            return
        
        request = PrepareForMission.Request()
        request.mission_id = mission_id
        request.waypoints = waypoints
        
        def drone_preparation_callback(future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Drone ready: {response.drone_state}')
                    self.get_logger().info("Drone preparation successful - ready to proceed to Step 2")
                    # TODO: Transition to next state (uncentering arms)
                    # self.state_machine.uncentring_arms()
                else:
                    self.get_logger().error(f'Drone not ready: {response.error_message}')
                    self.get_logger().error("Drone preparation failed - aborting mission")
                    # TODO: Add transition back to idle or error state
            except Exception as e:
                self.get_logger().error(f'Service call failed: {str(e)}')
                self.get_logger().error("Drone preparation failed - aborting mission")
        
        future = self.prepare_mission_client.call_async(request)
        future.add_done_callback(drone_preparation_callback)

#############################
#SIMULATION
#############################
        
    def simulate_mission_received(self):
        """Simulate receiving a mission from API - for testing"""
        from shared_interfaces.msg import Waypoint
        
        # Create test waypoints
        waypoints = []
        waypoint1 = Waypoint()
        waypoint1.latitude = -33.9249
        waypoint1.longitude = 18.4241
        waypoint1.altitude = 50.0
        waypoint1.speed = 5.0
        waypoints.append(waypoint1)
        
        # Transition to preparing_drone state
        self.state_machine.prepare_drone()
        
        # Call drone service (now async with callbacks)
        self.prepare_drone_for_mission("test_mission_001", waypoints)

    def trigger_test_mission(self):
        """Timer callback to trigger test mission"""
        if not self.mission_triggered and self.state_machine.current_state.id == 'idle':
            self.get_logger().info("Timer triggered - starting test mission...")
            self.mission_triggered = True
            self.simulate_mission_received()
        else:
            # Stop the timer after first trigger
            pass

#############################
#SIMULATION
#############################

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