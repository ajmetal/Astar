from math import inf, sqrt
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    #need to redefine the adjacency function to work on a mesh
    def adj(graph, current):
        return None        

    # The priority queue
    queue = [(0, source_point)]

    # The dictionary that will be returned with the costs
    distances = {}
    distances[source_point] = 0

    # The dictionary that will store the backpointers
    backpointers = {}
    backpointers[source_point] = None

    while queue:
        current_dist, current_node = heappop(queue)

        # Check if current node is the destination_point
        if current_node == destination_point:

            # List containing all cells from source_point to destination_point
            path = [current_node]

            # Go backwards from destination_point until the source using backpointers
            # and add all the nodes in the shortest path into a list
            current_back_node = backpointers[current_node]
            while current_back_node is not None:
                path.append(current_back_node)
                current_back_node = backpointers[current_back_node]

            return path[::-1]

        # Calculate cost from current note to all the adjacent ones
        for adj_node, adj_node_cost in adj(graph, current_node):
            pathcost = current_dist + adj_node_cost

            # If the cost is new
            if adj_node not in distances or pathcost < distances[adj_node]:
                distances[adj_node] = pathcost
                backpointers[adj_node] = current_node
                heappush(queue, (pathcost, adj_node))

    return None

    path = []
    boxes = {}

    return path, boxes.keys()
