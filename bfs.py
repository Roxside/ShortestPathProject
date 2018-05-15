import queue as Q


def getPriorityQueue(list, amat, end):
    q = Q.PriorityQueue()
    for node in list:
        q.put(Heuristic_Node(amat[node][end], node))
    return q, len(list)


class Heuristic_Node(object):
    def __init__(self, priority, description):
        self.priority = priority
        self.description = description
        return

    def __lt__(self, other):
        return self.priority< other.priority
    def __eq__(self,other):
        return  self.priority==other.priority


def greedyBFS(G, source, dest, amat):
    visited = {}
    for node in G.nodes:
        visited[node] = False
    fin = []
    goal = greedyBFSRec(G, source, visited, fin, dest, amat, 0)#if goal =0 no path
    return fin,goal


def greedyBFSRec(G, v, visited, final_path, dest, amat, goal):
    if goal == 1:
        return goal
    visited[v] = True
    final_path.append(v)
    if v == dest:
        goal = 1
    else:
        pq_list = []
        pq, size = getPriorityQueue(G.edges[v],amat,dest)#fill priority
        for i in range(size):
            pq_list.append(pq.get().description)
        for i in pq_list:
            if goal != 1:
                #print ("current city:"+ str(i))  #for debug
                if visited[i] == False:
                    goal = greedyBFSRec(G, i, visited, final_path, dest, amat, goal)
    return goal
