import pygame
import os
import requests
from PIL import Image
from io import BytesIO

script_dir = os.path.dirname(os.path.realpath(__file__))
image_dir = os.path.join(script_dir, 'images')

class ChessGUI:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((400, 400))
        pygame.display.set_caption("Chess GUI")
        self.clock = pygame.time.Clock()
        self.selected_square = None
        self.board = [[' ' for _ in range(8)] for _ in range(8)]  # Initialize an empty board
        self.load_images()  # Load and resize chess piece images
        self.update_board()  # Initialize the board from the server

    def load_images(self):
        self.piece_images = {
            'K': self.load_and_resize_image('white_king.png'),
            'Q': self.load_and_resize_image('white_queen.png'),
            'R': self.load_and_resize_image('white_rook.png'),
            'N': self.load_and_resize_image('white_knight.png'),
            'B': self.load_and_resize_image('white_bishop.png'),
            'P': self.load_and_resize_image('white_pawn.png'),
            'k': self.load_and_resize_image('black_king.png'),
            'q': self.load_and_resize_image('black_queen.png'),
            'r': self.load_and_resize_image('black_rook.png'),
            'n': self.load_and_resize_image('black_knight.png'),
            'b': self.load_and_resize_image('black_bishop.png'),
            'p': self.load_and_resize_image('black_pawn.png')
        }

    def load_and_resize_image(self, filename, size=(50, 50)):
        image = Image.open(os.path.join(image_dir, filename))
        image = image.resize(size, Image.ANTIALIAS)
        img_byte_array = BytesIO()
        image.save(img_byte_array, format="PNG")
        return pygame.image.load(BytesIO(img_byte_array.getvalue()))

    def update_board(self):
        try:
            # Request the current board state from the server
            response = requests.get('http://localhost:5000/board')

            if response.status_code == 200:
                fen = response.json().get('board')

                # Update the client-side board representation
                ranks = fen.split()[0].split('/')
                for row in range(8):
                    col = 0
                    for char in ranks[row]:
                        if char.isnumeric():
                            col += int(char)
                        else:
                            self.board[row][col] = char
                            col += 1

                # Redraw the GUI to display the updated board with pieces
                self.draw_board()
            else:
                print("Failed to retrieve the board state from the server.")
        except Exception as e:
            print(f"Error updating board: {str(e)}")

    def draw_board(self):
        self.screen.fill((255, 255, 255))  # Clear the screen with a white background

        for row in range(8):
            for col in range(8):
                color = (255, 255, 255) if (row + col) % 2 == 0 else (40,40,40)
                pygame.draw.rect(self.screen, color, (col * 50, row * 50, 50, 50))

                piece = self.board[row][col]
                if piece != ' ':
                    image = self.piece_images.get(piece)
                    if image:
                        self.screen.blit(image, (col * 50, row * 50))

                # Highlight the selected square with a green border
                if self.selected_square == (row, col):
                    pygame.draw.rect(self.screen, (0, 255, 0), (col * 50, row * 50, 50, 50), 3)  # 3 is the thickness of the green border

        pygame.display.flip()

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.MOUSEBUTTONDOWN:
                    x, y = pygame.mouse.get_pos()
                    row = y // 50
                    col = x // 50
                    self.on_square_click(row, col)

            self.draw_board()
            pygame.display.update()
            self.clock.tick(60)

    def on_square_click(self, row, col):
        if self.selected_square is None:
            self.selected_square = (row, col)
        else:
            from_square = self.selected_square
            to_square = (row, col)

            # Check if the same square is clicked twice
            if from_square == to_square:
                self.selected_square = None
            else:
                move = f"{chr(from_square[1] + 97)}{8 - from_square[0]}{chr(to_square[1] + 97)}{8 - to_square[0]}"

                # Send the move to the server through REST API and get the response
                try:
                    response = requests.get(f'http://localhost:5000/move?from={move[0:2]}&to={move[2:]}')

                    # Check if the move is valid based on the server response
                    if response.status_code == 200:
                        if response.json().get('success'):
                            self.board[to_square[0]][to_square[1]] = self.board[from_square[0]][from_square[1]]
                            self.board[from_square[0]][from_square[1]] = ' '
                        else:
                            # Show an error popup or message here
                            print("Invalid move. Try again.")
                    else:
                        print("Failed to send the move request to the server.")
                except Exception as e:
                    print(f"Error sending move request: {str(e)}")

                self.selected_square = None

if __name__ == '__main__':
    gui = ChessGUI()
    gui.run()
