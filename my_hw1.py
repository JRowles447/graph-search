from eight_puzzle import Puzzle
from collections import deque
from queue import PriorityQueue
import heapq
import time

##################################################################
### Node class and helper functions provided for your convenience.
### DO NOT EDIT!
##################################################################
class Node:
    """
    A class representing a node.
    - 'state' holds the state of the node.
    - 'parent' points to the node's parent.
    - 'action' is the action taken by the parent to produce this node.
    - 'path_cost' is the cost of the path from the root to this node.
    """
    def __init__(self, state, parent, action, path_cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def gen_child(self, problem, action):
        """
        Returns the child node resulting from applying 'action' to this node.
        """
        return Node(state=problem.transitions(self.state, action),
                    parent=self,
                    action=action,
                    path_cost=self.path_cost + problem.step_cost(self.state, action))

    @property
    def state_hashed(self):
        """
        Produces a hashed representation of the node's state for easy
        lookup in a python 'set'.
        """
        return hash(str(self.state))

##################################################################
### Node class and helper functions provided for your convenience.
### DO NOT EDIT!
##################################################################
def retrieve_solution(node,num_explored,num_generated):
    """
    Returns the list of actions and the list of states on the
    path to the given goal_state node. Also returns the number
    of nodes explored and generated.
    """
    actions = []
    states = []
    while node.parent is not None:
        actions += [node.action]
        states += [node.state]
        node = node.parent
    states += [node.state]
    return actions[::-1], states[::-1], num_explored, num_generated

##################################################################
### Node class and helper functions provided for your convenience.
### DO NOT EDIT!
##################################################################
def print_solution(solution):
    """
    Prints out the path from the initial state to the goal given
    a tuple of (actions,states) corresponding to the solution.
    """
    actions, states, num_explored, num_generated = solution
    print('Start')
    for step in range(len(actions)):
        print(puzzle.board_str(states[step]))
        print()
        print(actions[step])
        print()
    print('Goal')
    print(puzzle.board_str(states[-1]))
    print()
    print('Number of steps: {:d}'.format(len(actions)))
    print('Nodes explored: {:d}'.format(num_explored))
    print('Nodes generated: {:d}'.format(num_generated))


################################################################
### Search Algorithms
################################################################
class BFS:
    """
    Breadth first search (slight modification from the book).
    - 'problem' is a Puzzle instance.
    """
    def __init__(self, problem):
        self.problem = problem
        self.init_state = problem.init_state

    def solve(self):
        """
        Perform breadth-first search and return a goal state node from which a
        solution can be reconstructed (if a solution exists).
        """
        root = Node(state=self.init_state,
                    parent=None,
                    action=None,
                    path_cost=0)
        if self.problem.is_goal(root.state):
            return retrieve_solution(root,0,1)
        frontier = deque([root])
        explored = set([root.state_hashed])
        num_generated = 1

        for node in iter(frontier.pop, None):
            for action in self.problem.actions(node.state):
                child = node.gen_child(self.problem, action)
                num_generated += 1
                child_state_hashed = child.state_hashed
                if child_state_hashed not in explored:
                    if self.problem.is_goal(child.state):
                        return retrieve_solution(child,len(explored),num_generated)
                    else:
                        frontier.appendleft(child)
                        explored.add(child_state_hashed)


class BFS_book:
    """
    Breadth first search (according to Figure 3.11 in the book).
    - 'start' is a Puzzle instance
    """
    def __init__(self, problem):
        self.problem = problem
        self.start = problem.init_state

    def solve(self):
        """
        Perform breadth-first search and return a goal state node from which a
        solution can be reconstructed (if a solution exists).
        """
        root = Node(state=self.start,
                    parent=None,
                    action=None,
                    path_cost=0)
        if self.problem.is_goal(root.state):
            return retrieve_solution(root,0,1)
        frontier = deque([root])
        frontier_set = set([root.state_hashed])
        explored  = set([])
        num_generated = 1

        for node in iter(frontier.pop, None):
            frontier_set.remove(node.state_hashed)
            explored.add(node.state_hashed())
            for action in self.problem.actions(node.state):
                child = node.gen_child(self.problem, action)
                num_generated += 1
                child_state_hashed = child.state_hashed
                if child_state_hashed not in explored and \
                    child_state_hashed not in frontier_set:
                    if self.problem.is_goal(child.state):
                        return retrieve_solution(child,len(explored),num_generated)
                    else:
                        frontier.appendleft(child)
                        frontier_set.add(child_state_hashed)



class Astar:
    """
    A* search.
    - 'problem' is a Puzzle instance.
    """
    def __init__(self, problem, verbose=False):
        self.problem = problem
        self.init_state = problem.init_state
        self.verbose = verbose
        self.num_explored = 0
        self.num_generated = 1

    def solve(self):
        """
        Perform A* search and return a solution using `retrieve_solution'
        (if a solution exists).
        IMPORTANT: Use node generation time to split ties among nodes with
        equal f(n).
        """
        node = Node(state=self.init_state,
                    parent=None,
                    action=None,
                    path_cost=0)
        frontier = []
        node_id = 0
        heapq.heappush(frontier,(self.f(node),node_id,node))
        frontier_set = set([node.state_hashed])
        explored  = set([])

        while len(frontier) > 0:
            priority, t, node = heapq.heappop(frontier)
            frontier_set.remove(node.state_hashed)
            if self.problem.is_goal(node.state):
                return retrieve_solution(node,self.num_explored,self.num_generated)
            explored.add(node.state_hashed)
            self.num_explored += 1
            if self.verbose:
                print('explored: ')
                print(self.problem.board_str(node.state))
            for action in self.problem.actions(node.state):
                child = node.gen_child(self.problem, action)
                self.num_generated += 1
                if self.verbose:
                    print('generated: ')
                    print(self.problem.board_str(child.state))
                child_state_hashed = child.state_hashed
                if child_state_hashed not in explored and child_state_hashed not in frontier_set:
                    node_id += 1
                    heapq.heappush(frontier,(self.f(child),node_id,child))
                    frontier_set.add(child_state_hashed)
                elif child_state_hashed in frontier_set:
                    swap_idx = next(idx for idx in range(len(frontier)) if frontier[idx][-1].state == child.state)
                    fval = frontier[swap_idx][0]
                    if fval > self.f(child):
                        node_id += 1
                        frontier[swap_idx] = (self.f(child),node_id,child)
                        heapq.heapify(frontier)

    def f(self,node):
        '''
        Returns a lower bound estimate on the cost from root through node
        to the goal.
        '''
        return node.path_cost + self.h(node)

    def h(self,node):
        '''
        Returns a lower bound estimate on the cost from node to the goal
        using the Manhattan distance heuristic.
        '''
        dist_sum = 0
        for i, number in enumerate(node.state):
            if number != self.problem.HOLE:
                r,c = divmod(i,self.problem.width)
                rgoal,cgoal = divmod(self.problem.goal_state.index(number),self.problem.width)
                manhattan = abs(rgoal-r) + abs(cgoal-c)
                dist_sum += manhattan
        return dist_sum

    def branching_factor(self, board, trials=100):
        '''
        Returns an average upper bound for the effective branching factor.
        '''
        b_hi = 0  # average upper bound for branching factor
        for t in range(trials):
            puzzle = Puzzle(board).shuffle()
            solver = Astar(puzzle)
            actions, states, num_explored, num_generated = solver.solve()
            depth = len(actions)
            b_hi += 1/trials*num_generated**(1./depth)
        return b_hi


if __name__ == '__main__':
    board = [[3,1,2],
             [4,0,5],
             [6,7,8]]
    # board = [[7,2,4],
    #          [5,0,6],
    #          [8,3,1]]
    # board = [[0,2,4],
    #          [7,5,6],
    #          [8,3,1]]

    puzzle = Puzzle(board)
    solver = Astar(puzzle,verbose=True)
    solution = solver.solve()
    print_solution(solution)

    board = [[7,2,4],
             [5,0,6],
             [8,3,1]]
    # board = [[0,2,4],
    #          [7,5,6],
    #          [8,3,1]]
    puzzle = Puzzle(board)
    solver = Astar(puzzle)
    solution = solver.solve()
    print_solution(solution)

    b_hi = solver.branching_factor(board, trials=100)
    print('Upper bound on effective branching factor: {:.2f}'.format(b_hi))
