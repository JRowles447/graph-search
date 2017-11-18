from eight_puzzle import Puzzle
from collections import deque
from queue import PriorityQueue
import heapq
import time

##################################################################
### Vertex class to track nodes in graph
##################################################################
class Vertex:
    """
    A class representing a node.
    - 'state' holds the state of the node.
    - 'parent' points to the node's parent.
    - 'action' is the action taken by the parent to produce this node.
    - 'path_cost' is the cost of the path from the root to this node.
    """
    def __init__(self, id, estimated):
        self.id = id
        self.parent = None
        self.estimated = estimated
        self.path_cost = -1
        self.visited = False
        # list of tuples of form (vertex, path_cost)
        self.edges = []

    def info(self):
        edge_string = str(self.edges)
        print("Vertex: " + self.id + " with edges: " + edge_string)

