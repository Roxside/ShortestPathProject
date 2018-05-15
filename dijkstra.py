from collections import defaultdict,deque
class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.distances[(from_node, to_node)] = distance
    # from given source 'src'
    def DLS(self,src,target,maxDepth,way):

        if src == target :
            return True,way

        # If reached the maximum depth, stop recurse
        if maxDepth <= 0:
            return False,way

        # Recur for all the vertices
        for i in self.edges[src]:
            way.append(i)
            p1,p2=self.DLS(i,target,maxDepth-1,way)
            if(p1):
                return True,p2
            del way[-1]
        return False,way

    # iterdeep to search if target is reachable from v.
    # It uses recursive DLS()
    def iterdeep(self, src, target, maxDepth):
        way=[src]
        # Repeatedly depth-limit search till the
        # maximum depth
        for i in range(maxDepth):
            p1,p2=self.DLS(src, target, i,way)
            if (p1):
                return True,p2
        return False,way

def dijkstra(graph, initial):
    visited = {initial: 0}
    path = {}

    nodes = set(graph.nodes)

    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
        if min_node is None:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]

        for edge in graph.edges[min_node]:
            try:
                weight = current_weight + graph.distances[(min_node, edge)]
            except:
                continue
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path

def shortest_path(graph, origin, destination):
    visited, paths = dijkstra(graph, origin)
    full_path = deque()
    dist=0
    _destination = paths[destination]

    while _destination != origin:
        full_path.appendleft(_destination)
        _destination = paths[_destination]



    full_path.appendleft(origin)
    full_path.append(destination)

    return visited[destination], list(full_path)
