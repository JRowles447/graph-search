from itertools import chain
from math import sqrt
from random import choice
from vertex import Vertex


class Graph:
    def __init__(self, vertices, search_type, start_vertex, goal_vertices):
        self.vertices = vertices
        self.search_type = search_type
        self.start_vertex = start_vertex
        self.goal_vertices = goal_vertices

def parse_file(filename):
    print('gets to parse')
    reader = open(filename, 'r')
    num_vertices = int(reader.readline())
    vertices = []
    while num_vertices:
        line = reader.readline()
        print(line)
        line = line.split(' ')
        vertices.append(Vertex(line[0], line[1]))
        num_vertices -= 1
    return None

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Traverse a graph with a provided search strategy and return traversal and total cost')

    parser.add_argument('graph_file', help='file with the vertex information')
    parser.add_argument('search', help='search to preform. Options: \'bfs\', \'dfs\', \'astar\', \'ids\'')
    parser.add_argument('start_state', default='S', help='Vertex to start search from')
    args = parser.parse_args()

    parse_file(args.graph_file)

