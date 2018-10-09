from math import inf, sqrt, floor, ceil
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
    print("src, dst: ", source_point, destination_point)

    def getBox(point):
        for box in mesh['boxes']:
            if point[0] >= box[0] and point[0] <= box[1] and point[1] >= box[2] and point[1] <= box[3]:
                return box
        print("point not in any box! ", point)
        return None


    #step 1: find which boxes in mesh contain the source and destination
    source_box = getBox(source_point)
    destination_box = getBox(destination_point)
    
    print(source_box, destination_box)

    #need to redefine the adjacency function to work on a mesh
    def adj(graph, current):
        current_box = getBox(current)
        
        adjBoxes = graph['adj'][current_box]
        
        ret = []
        for box in adjBoxes:
            midpoint = ( floor((box[0] + box[1]) / 2), floor((box[2] + box[3]) / 2) )
            ret.append( ( midpoint, get_distance(midpoint, current)) )
        return ret

    def get_distance(goal, next):
        #(x1, x2, y1, y2)
        #(x1, y1) = top-left
        #(x2, y2) = bottom-right
        return abs(goal[0] - next[0]) + abs(goal[1] - next[1])

    # The priority queue
    queue = [(0, source_point)]

    # The dictionary that will be returned with the costs
    distances = {}
    distances[source_point] = 0

    # The dictionary that will store the backpointers
    backpointers = {}
    backpointers[source_point] = None

    path = []
    boxes = []

    while queue:
        current_dist, current_node = heappop(queue)

        # Check if current node is the destination_point
        if getBox(current_node) == getBox(destination_point):

            # List containing all cells from source_point to destination_point
            

            # Go backwards from destination_point until the source using backpointers
            # and add all the nodes in the shortest path into a list
            current_back_node = backpointers[current_node]
            path = [(current_node, current_back_node)]
            while current_back_node is not None:
                path.append((current_back_node, backpointers[current_back_node]))
                current_back_node = backpointers[current_back_node]
                
            path = path[:-1]
            break

        # Calculate cost from current node to all the adjacent ones
        for adj_node, adj_node_cost in adj(mesh, current_node):
            pathcost = current_dist + adj_node_cost

            # If the cost is new
            if adj_node not in distances or pathcost < distances[adj_node]:
                distances[adj_node] = pathcost
                backpointers[adj_node] = current_node
                boxes.append(getBox(adj_node))
                priority = pathcost + get_distance(destination_point, adj_node)
                heappush(queue, (priority, adj_node))

    print('path: ', path)
    return path, boxes
