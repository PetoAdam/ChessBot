#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chess_move_srv.srv import ChessMove  # Adjust the import according to your package structure

class DummyMotionPlanner(Node):
    def __init__(self):
        super().__init__('dummy_motion_planner')
        self.service = self.create_service(ChessMove, 'process_chess_move', self.process_chess_move_callback)

    def process_chess_move_callback(self, request, response):
        self.get_logger().info('Received move request, sending dummy response...')
        response.success = True
        response.message = "Move processed successfully by dummy motion planner"
        return response

def main(args=None):
    print('Starting dummy motion planner')
    rclpy.init(args=args)
    node = DummyMotionPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
