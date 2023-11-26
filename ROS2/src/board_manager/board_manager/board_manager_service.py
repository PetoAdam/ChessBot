#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from chess_move_srv.srv import ChessMove
import chess
from flask import Flask, request, jsonify
import threading
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_piece_heights(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

package_share_directory = get_package_share_directory('board_manager')
yaml_path = os.path.join(package_share_directory, 'config', 'piece_heights.yaml')
piece_heights = load_piece_heights(yaml_path)

# Flask App Setup
app = Flask(__name__)
board = chess.Board()

@app.route('/board', methods=['GET'])
def get_board_state():
    return jsonify({'board': board.fen()})

@app.route('/move', methods=['GET'])
def move_piece():
    from_square = request.args.get('from')
    to_square = request.args.get('to')
    move = chess.Move.from_uci(f"{from_square}{to_square}")
    is_clash = board.is_capture(move)

    if move in board.legal_moves:
        future = board_manager.send_move_to_motion_planner(from_square, to_square, is_clash)
        rclpy.spin_until_future_complete(board_manager, future)
        response = future.result()
        if response.success:
            board.push(move)
            return jsonify({'success': True, 'board': board.fen(), 'message': response.message})
        else:
            return jsonify({'success': False, 'message': response.message})
    else:
        return jsonify({'success': False, 'message': 'Illegal move'})

def run_flask_app():
    app.run(debug=False, port=5000)

# ROS 2 Node for Chess Board Manager
class BoardManagerService(Node):
    def __init__(self):
        super().__init__('board_manager_service')
        self.client = self.create_client(ChessMove, 'process_chess_move')

    def convert_to_real_world_coordinates(self, from_square, to_square):
        square_size = 0.5 / 8  # 0.5 meters divided by 8 squares
        origin_x = 0.4  # X-coordinate of the bottom-left corner of the chessboard
        origin_y = 0.0  # Y-coordinate of the bottom-left corner of the chessboard

        def square_to_coords(square, z_height):
            col = ord(square[0]) - ord('a')
            row = int(square[1]) - 1
            x = origin_x + col * square_size
            y = origin_y + row * square_size
            return [x, y, z_height]

        # Determine the z-coordinate based on the piece at the to_square
        z = 0.0  # Default height
        piece = board.piece_at(chess.SQUARE_NAMES.index(from_square))
        if piece:
            piece_symbol = piece.symbol().upper()
            z = piece_heights.get(piece_symbol, z)
        else:
            self.get_logger().info(f"No piece on {to_square}, using default height")

        from_coordinates = square_to_coords(from_square, z)
        to_coordinates = square_to_coords(to_square, z)  # Using the same z-coordinate for to_square

        return from_coordinates + to_coordinates


    def send_move_to_motion_planner(self, from_square, to_square, is_clash):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for motion planner service...')

        req = ChessMove.Request()
        coordinates = self.convert_to_real_world_coordinates(from_square, to_square)
        req.from_x, req.from_y, req.from_z, req.to_x, req.to_y, req.to_z = coordinates
        req.is_clash = is_clash

        return self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    global board_manager
    board_manager = BoardManagerService()

    executor = MultiThreadedExecutor()
    executor.add_node(board_manager)

    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        board_manager.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
