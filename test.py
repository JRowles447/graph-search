from __future__ import with_statement # Required in 2.5
import signal
from contextlib import contextmanager

from eight_puzzle import Puzzle
from hw1_key import Astar as master
from my_hw1 import Astar as student


# https://stackoverflow.com/questions/2052390/manually-raising-throwing-an-exception-in-python
class TimeoutException(Exception): pass

@contextmanager
def time_limit(seconds):
    def signal_handler(signum, frame):
        raise TimeoutException('Timed out!')
    signal.signal(signal.SIGALRM, signal_handler)
    signal.alarm(seconds)
    try:
        yield
    finally:
        signal.alarm(0)

def test_bhi(board, student, mx=1.48):
    trials = 100
    b_hi = 0
    puzzle = Puzzle(board,test=False)
    for t in range(trials):
        puzzle = puzzle.shuffle()
        solver = student(puzzle)
        actions, states, num_explored, num_generated = solver.solve()
        depth = len(actions)
        b_hi += 1/trials*num_generated**(1./depth)
    return b_hi < mx


if __name__ == '__main__':
    board = [[3,1,2],
             [4,0,5],
             [6,7,8]]

    puzzle = Puzzle(board,test=True)
    master(puzzle).solve()
    master_path = puzzle.path_string

    try:
        with time_limit(5):
            puzzle.path_string = ''
            student(puzzle).solve()
            student_path = puzzle.path_string
            if master_path==student_path:
                print('Simple puzzle: \t\t\t5/5')
            else:
                print('Simple puzzle: \t\t\t0/5')
    except Exception as e:
        print('Simple puzzle: \t\t\t0/5')
        print(e.args[0])

    board = [[7,2,4],
             [5,0,6],
             [8,3,1]]

    try:
        with time_limit(10):
            puzzle = Puzzle(board,test=False)
            solver = student(puzzle)
            solution = solver.solve()
            if len(solution[0])==26:
                print('Harder puzzle: \t\t\t10/10')
            else:
                print('Harder puzzle: \t\t\t0/10')
    except Exception as e:
        print('Harder puzzle: \t\t\t0/10')
        print(e.args[0])

    try:
        with time_limit(60):
            manual = test_bhi(board, student, mx=1.48)
            auto = solver.branching_factor(board, trials=100) < 1.48
            if manual and auto:
                print('Adequate branching factor: \t15/15')
            elif manual:
                print('Adequate branching factor: \t0/15')
                print('Auto failed.')
            else:
                print('Adequate branching factor: \t0/15')
                print('Manual failed.')
    except Exception as e:
        print('Adequate branching factor: \t0/15')
        print(e.args[0])
