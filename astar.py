import queue as Q


def getAstarPriorityQueue(list,v, amat,dmat, end):
    q = Q.PriorityQueue()
    for node in list:
        q.put(HeuristicAstar_Node(amat[node][end]+dmat[v][node], node))
    return q, len(list)


class HeuristicAstar_Node(object):
    def __init__(self, priority, description):
        self.priority = priority
        self.description = description
        return

    def __lt__(self, other):
        return self.priority< other.priority
    def __eq__(self,other):
        return  self.priority==other.priority


def Astar(G, source, dest, amat,dmat):
    visited = {}
    for node in G.nodes:
        visited[node] = False
    fin = []
    goal = AstarRec(G, source, visited, fin, dest, amat,dmat,0)#if goal =0 no path
    return fin,goal


def AstarRec(G, v, visited, final_path, dest,amat,dmat, goal):
    if goal == 1:
        return goal
    visited[v] = True
    final_path.append(v)
    if v == dest:
        goal = 1
    else:
        pq_list = []
        pq, size = getAstarPriorityQueue(G.edges[v],v,amat,dmat,dest)#fill priority
        for i in range(size):
            pq_list.append(pq.get().description)
        for i in pq_list:
            if goal != 1:
                #print ("current city:"+ str(i))  #for debug
                if visited[i] == False:
                    goal = AstarRec(G, i, visited, final_path, dest,amat,dmat, goal)
    return goal
