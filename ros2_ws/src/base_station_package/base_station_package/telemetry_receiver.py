#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from shared_interfaces.msg import DroneState

class TelemetryReceiver(Node):
    def __init__(self):
        super().__init__('telemetry_receiver')
        
        # Subscribe to drone telemetry
        self.subscription = self.create_subscription(
            DroneState,
            'drone/state',
            self.telemetry_callback,
            10
        )
        
        self.get_logger().info('Telemetry receiver started - waiting for drone data...')
        
    def telemetry_callback(self, msg):
        """Process incoming drone telemetry"""
        self.get_logger().info(
            f'Received telemetry: '
            f'armed={msg.armed}, '
            f'in_air={msg.is_in_air}, '
            f'battery={msg.battery_percentage:.1f}%, '
            f'lat={msg.latitude:.6f}, '
            f'lon={msg.longitude:.6f}, '
            f'alt={msg.altitude:.1f}m'
        )

def main():
    rclpy.init()
    node = TelemetryReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down telemetry receiver...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()