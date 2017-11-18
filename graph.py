from itertools import chain
from math import sqrt
from random import choice
from vertex import Vertex
from queue import Queue, LifoQueue


class Graph:
    def __init__(self, vertices, search_type, start_vertex, goal_vertices):
        self.vertices = vertices
        self.search_type = search_type
        self.start_vertex = start_vertex
        self.goal_vertices = goal_vertices
        self.path_cost = 0
        self.path = []
        self.complete = False

    def solve(self):
        if self.search_type == 'bfs':
            self.bfs()
        elif self.search_type == 'dfs':
            self.dfs()
        elif self.search_type == 'ids':
            self.ids()
        else:
            self.astar()
        print(self.path_cost)
        for x in self.path:
            print(x)

    def bfs(self):
        print('bfs')
        vertices = self.vertices

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

    def dfs(self):
        print('dfs')
        print(self.goal_vertices)
        vertices = self.vertices

        queue = LifoQueue()
        queue.put(vertices['S'])
        self.path.append('S')
        while not queue.empty():
            curr = queue.get()
            curr.edges.sort()
            print(curr.edges)
            i = 0
            while (i < len(curr.edges)) and not self.complete:
                adjacent = curr.edges[i]
                if not vertices[adjacent[0]].visited and vertices[adjacent[0]] not in self.goal_vertices and not self.complete:
                    self.dfs_visit(vertices[adjacent[0]], curr.edges[i][1])
                else:
                    return
                i += 1

    def dfs_visit(self, curr, cost):
        vertices = self.vertices

        # sort the edges alpha
        self.path_cost += cost
        self.path.append(curr.id)
        curr.visited = True
        curr.edges.sort()
        i = 0
        while (i < len(curr.edges)) and not self.complete:
            adjacent = curr.edges[i]
            # print(vertices[adjacent[0]].id)
            if not vertices[adjacent[0]].visited and vertices[adjacent[0]].id not in self.goal_vertices:
                self.dfs_visit(vertices[adjacent[0]], curr.edges[i][1])
            else:
                self.path.append(vertices[adjacent[0]].id)
                self.path_cost += curr.edges[i][1]
                self.complete = True
                return
            i += 1




def parse_file(filename):
    reader = open(filename, 'r')
    num_vertices = int(reader.readline())
    vertices = {}
    while num_vertices:
        line = reader.readline()
        line = line.split(' ')
        vertices[line[0]] = Vertex(line[0], line[1])
        num_vertices -= 1
    line = reader.readline()
    while line:
        edge = line.split(' ')
        vertices[edge[0]].edges.append((edge[1], int(edge[2])))
        line = reader.readline()
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

    graph.solve()

    # graph.bfs()

