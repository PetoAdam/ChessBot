#!/usr/bin/env python3

import os
import sys  # noqa
from pathlib import Path
parent_dir = Path(__file__).parent
sys.path.append(parent_dir)

print('Parentdir: ', parent_dir)
print(sys.path)

import rclpy
from rclpy.node import Node
#from .srv import ChessMove
#from .srv import ChessMove
import chess
from flask import Flask, request, jsonify
import threading

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
    if move in board.legal_moves:
        board.push(move)
        return jsonify({'success': True, 'board': board.fen()})
    else:
        return jsonify({'success': False, 'message': 'Illegal move'})

def run_flask_app():
    app.run(debug=False, port=5000)

# ROS 2 Node for Chess Board Manager
class BoardManagerService(Node):
    def __init__(self):
        super().__init__('board_manager_service')
        #self.srv = self.create_service(ChessMove, 'chess_move', self.chess_move_callback)

    def chess_move_callback(self, request, response):
        # ROS service callback implementation
        # ...
        pass

def main(args=None):
    rclpy.init(args=args)

    board_manager_service = BoardManagerService()

    # Start Flask app in a separate thread
    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()

    rclpy.spin(board_manager_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
