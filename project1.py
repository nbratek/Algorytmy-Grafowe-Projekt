#Natalia Bratek
import sys
sys.setrecursionlimit(100000)

#import resource as rc
#rc.setrlimit(rc.RLIMIT_STACK, (rc.RLIM_INFINITY, rc.RLIM_INFINITY))

from data import runtests

visited = []
low = []
plac_tranzytowy = []
time = 0

def dfs(v, neighbours):
    global visited, time, low
    time += 1
    visited[v] = time
    for u, t in neighbours[v]:
        if visited[u] == 0:
            dfs(u, neighbours)

def calc_low(v, neighbours):
    global visited, time, low
    low[v] = visited[v]
    for u, t in neighbours[v]:
        if visited[u] > visited[v]: 
            if low[u] == 0:
                calc_low(u, neighbours)
            low[v] = min(low[v], low[u])
        elif visited[u] < visited[v]: 
            low[v] = min(low[v], visited[u])

from queue import PriorityQueue

def dijkstra(v, neighbours):
    N = len(neighbours) - 1
    distance = [float("inf") for _ in range(N+1)]
    distance[v] = 0
    queue = PriorityQueue()
    queue.put((0, v))

    while not queue.empty():
        dist, u = queue.get()
        if dist == distance[u]:
            for w, t in neighbours[u]:
                if distance[w] > distance[u] + t:
                    distance[w] = distance[u] + t
                    queue.put((distance[w], w))
    return distance

def my_solve(N, streets):
    global visited, time, low, plac_tranzytowy

    neighbours = [[] for _ in range(N+1)]
    M = len(streets)

    for a, b, t in streets:
        neighbours[a].append((b, t))
        neighbours[b].append((a, t))

    visited = [0 for _ in range(N+1)]
    time = 0
    dfs(1, neighbours)

    low = [0 for _ in range(N+1)]
    calc_low(1, neighbours)

    plac_tranzytowy = [False for _ in range(N+1)]

    count_black = 0
    for u, t in neighbours[1]:
        if visited[u] >= visited[1]:
            count_black += 1

    if count_black > 1:
        plac_tranzytowy[1] = True

    for v in range(2, N+1):
        for u, t in neighbours[v]:
            if visited[u] > visited[v] and low[u] >= visited[v]:
                plac_tranzytowy[v] = True
                break
    
    transit_graph = [[] for _ in range(N+1+M)]
    for i in range(len(streets)):
        a, b, t = streets[i]
        v = N+i+1

        wa = 1 if plac_tranzytowy[a] else 0
        
        transit_graph[a].append((v, wa))
        transit_graph[v].append((a, wa))

        wb = 1 if plac_tranzytowy[b] else 0

        transit_graph[v].append((b, wb))
        transit_graph[b].append((v, wb))

    best_solution = 0
    best_solution_trans = 0

    for i in range(1, N+1):
        distance = dijkstra(i, neighbours)
        transit_points = dijkstra(i, transit_graph)

        for u in range(1, N+1):
            if transit_points[u] > best_solution_trans:
                best_solution_trans = transit_points[u]
                best_solution = distance[u]
            elif transit_points[u] == best_solution_trans and distance[u] < best_solution:
                best_solution = distance[u]

    return best_solution_trans // 2, best_solution

runtests(my_solve)