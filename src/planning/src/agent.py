class CheckersAgent:
    def __init__(self, move_function):
        self.move_function = move_function

    def make_move(self, board):
        return self.move_function(board)
