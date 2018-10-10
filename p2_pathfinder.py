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

    #finds the box in the mesh containing the given point
    def get_box(point):
        for box in mesh['boxes']:
            if point[0] >= box[0] and point[0] <= box[1] and point[1] >= box[2] and point[1] <= box[3]:
                return box
        print("point not in any box! ", point)
        return None

    source_box = get_box(source_point)
    destination_box = get_box(destination_point)

    #early return if the points are in the same box
    if source_box is destination_box:
        return [(source_point, destination_point)], [source_box]

    # The priority queue
    queue = [(0, source_point, 'destination'), (0, destination_point, 'source')]

    # The dictionary that will be returned with the costs
    forward_distances = {}
    forward_distances[source_point] = 0

    backward_distances = {}
    backward_distances[destination_point] = 0

    # The dictionary that will store the backpointers
    forward_prev = {}
    forward_prev[source_point] = None

    backward_prev = {}
    backward_prev[destination_point] = None


    #map to allow quicker lookup of visited boxes
    point_box_map = { source_point : source_box }

    #return values
    path = []
    visited_boxes = []

    #calculates eucilidean distance between two points
    def get_distance(a, b):
        return sqrt( (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 )

    def clamp(val, lower, upper): return max( min( val, upper), lower )

    def adj(graph, current):       
        adj_list = []

        for box in graph['adj'][ point_box_map[current] ]:
            #constrained point
            closest_point = ( clamp(current[0], box[0], box[1]), clamp(current[1], box[2], box[3]) )
            #midpoint
            #midpoint = ( (box[0] + box[1]) / 2, (box[2] + box[3]) / 2 )

            point_box_map[closest_point] = box
            
            adj_list.append( ( closest_point, get_distance(current, closest_point) ) )
        return adj_list

    #perform a*
    while queue:
        current_dist, current_point, current_goal = heappop(queue)

        if current_goal == 'source':
            distances = backward_distances
            backpointers = backward_prev
        else:
            backpointers = forward_prev
            distances = forward_distances

        current_dist = distances[current_point]

        #need to check is the current box has been visited by a point from the other direction
        if point_box_map[current_point] == point_box_map[]:
            #construct path
            prev_point = backpointers[current_point]
            path = [(destination_point, current_point), (current_point, prev_point)]
            while prev_point is not None:
                path.append( (prev_point, backpointers[prev_point]) )
                prev_point = backpointers[prev_point]
            
            path = path[:-1]
            return path, visited_boxes

        # Calculate cost from current node to all the adjacent ones
        for adj_point, adj_point_cost in adj(mesh, current_point):
            pathcost = current_dist + adj_point_cost

            # If the cost is new or less
            if adj_point not in distances or pathcost < distances[adj_point]:
                distances[adj_point] = pathcost
                backpointers[adj_point] = current_point
                visited_boxes.append(point_box_map[adj_point])
                if current_goal == 'source':
                    priority = pathcost + get_distance(adj_point, source_point)
                else:
                    priority = pathcost + get_distance(adj_point, destination_point)
                heappush(queue, (priority, adj_point, current_goal))

        if current_goal == 'source':
            backward_distances = distances
            backward_prev = backpointers
        else:
            forward_prev =  backpointers
            forward_distances = distances

    print("No path found!")
    return path, visited_boxes
