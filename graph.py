from itertools import chain
from math import sqrt
from random import choice
from vertex import Vertex
from queue import Queue


class Graph:
    def __init__(self, vertices, search_type, start_vertex, goal_vertices):
        self.vertices = vertices
        self.search_type = search_type
        self.start_vertex = start_vertex
        self.goal_vertices = goal_vertices
        self.path_cost = 0
        self.path = []

    def bfs(self):
        print('bfs')
        vertices = self.vertices
        for k, v in vertices.items():
            v.info()
        queue = Queue()
        queue.put(vertices['S'])
        self.path.append('S')
        while not queue.empty():
            curr = queue.get()
            i = 0
            while (i < len(curr.edges)):
                adjacent = curr.edges[i]
                if not vertices[adjacent[0]].visited and vertices[adjacent[0]] not in self.goal_vertices:
                    self.path_cost += curr.edges[i][1]
                    vertices[adjacent[0]].visited = True
                    queue.put(vertices[adjacent[0]])
                    self.path.append(adjacent[0])
                else:
                    return
                i+=1




def parse_file(filename):
    print('gets to parse')
    reader = open(filename, 'r')
    num_vertices = int(reader.readline())
    vertices = {}
    while num_vertices:
        line = reader.readline()
        print(line)
        line = line.split(' ')
        vertices[line[0]] = Vertex(line[0], line[1])
        num_vertices -= 1
    line = reader.readline()
    while line:
        print(line)
        edge = line.split(' ')
        vertices[edge[0]].edges.append((edge[1], int(edge[2])))
        line = reader.readline()

    for k, v in vertices.items():
        print(k, v)
        v.info()
    return vertices

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Traverse a graph with a provided search strategy and return traversal and total cost')

    parser.add_argument('graph_file', help='file with the vertex information')
    parser.add_argument('search', help='search to preform. Options: \'bfs\', \'dfs\', \'astar\', \'ids\'')
    parser.add_argument('start_state', default='S', help='Vertex to start search from')
    args = parser.parse_args()

    goal_vertices = ['G1', 'G2']
    vertice_dict = parse_file(args.graph_file)


    graph = Graph(vertice_dict, args.search, args.start_state, goal_vertices)

    graph.bfs()

