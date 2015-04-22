from collections import deque
from math import sqrt
from math import fabs
from heapq import heappush, heappop

def find_path(src_point, dest_point, mesh):

    src_box = None
    dest_box = None
    path = []
    visited_boxes = []
    src_adj = None

    for box in mesh["boxes"]:
        if not src_box:
            if src_point[0] > box[0] and src_point[0] < box[1]:
                if src_point[1] > box[2] and src_point[1] < box[3]:
                    src_adj = mesh["adj"][box]
                    print "source coordinates:"
                    print "x1, x2: ", box[0], ", ", box[1]
                    print "y1, y2: ", box[2], ", ", box[3]
                    src_box = box
        if not dest_box:
            if dest_point[0] > box[0] and dest_point[0] < box[1]:
                if dest_point[1] > box[2] and dest_point[1] < box[3]:
                    print "destination coordinates:"
                    print "x1, x2: ", box[0], ", ", box[1]
                    print "y1, y2: ", box[2], ", ", box[3]
                    dest_box = box
        if src_box and dest_box:
            break

    if not src_box or not dest_box:
        print "No path possible"
        return [], []

    """biDirA*"""
    forward_parent = {}
    forward_parent[src_box] = None
    backward_parent = {}
    backward_parent[dest_box] = None
    forward_dist = {}
    backward_dist = {}
    forward_dist[src_box] = 0
    backward_dist[dest_box] = 0

    last_forward = src_point
    last_backward = dest_point

    queue = [(getDist(src_point, dest_point), src_box, "destination")]
    heappush(queue, (getDist(dest_point, src_point), dest_box, "source"))
    visited_boxes.append(src_box)
    visited_boxes.append(dest_box)

    if src_box == dest_box:
        path.append((src_point, dest_point))
        visited_boxes.append(src_box)
        return path, visited_boxes

    while queue:
        priority, box, curr_goal = heappop(queue)

        if box in backward_parent and box in forward_parent:
            last_point = ((box[1] + box[0])/2, (box[3] + box[2])/2)
            center_box = box
            while forward_parent[box] != None:
                next_point = getClosestPoint(last_point, forward_parent[box])
                path.append((last_point, next_point))
                last_point = next_point
                box = forward_parent[box]
            path.append((last_point, src_point))
            path.reverse()
            box = center_box
            last_point = ((box[1] + box[0])/2, (box[3] + box[2])/2)
            while backward_parent[box] != None:
                next_point = getClosestPoint(last_point, backward_parent[box])
                path.append((last_point, next_point))
                last_point = next_point
                box = backward_parent[box]
            path.append((last_point, dest_point))
            return path, visited_boxes

        neighbors = mesh["adj"][box]
        if curr_goal == "destination":
            for neighbor in neighbors:
                weight = getDist(getPoint(box), getPoint(neighbor))
                alt = forward_dist[box] + weight
                if neighbor not in forward_dist or alt < forward_dist[neighbor]:
                    forward_dist[neighbor] = alt
                    forward_parent[neighbor] = box
                    last_forward = getPoint(box)
                    if neighbor not in queue:
                        visited_boxes.append(neighbor)
                        #heappush(queue, ((getDist(getPoint(neighbor), last_backward), neighbor, curr_goal)))
                        heappush(queue, ((alt + getDist(getPoint(neighbor), dest_point), neighbor, curr_goal)))
        else:
            for neighbor in neighbors:
                weight = getDist(getPoint(box), getPoint(neighbor))
                alt = backward_dist[box] + weight
                if neighbor not in backward_dist or alt < backward_dist[neighbor]:
                    backward_dist[neighbor] = alt
                    backward_parent[neighbor] = box
                    last_backward = getPoint(box)
                    if neighbor not in queue:
                        visited_boxes.append(neighbor)
                        #heappush(queue, ((getDist(getPoint(neighbor), last_forward), neighbor, curr_goal)))
                        heappush(queue, ((alt + getDist(getPoint(neighbor), src_point), neighbor, curr_goal)))

    """A*"""
    # parent = {}
    # parent[src_box] = None
    # dist = {}
    # dist[src_box] = 0
    # queue = [(getDist(src_point, dest_point), src_box)]
    # visited_boxes.append(src_box)
    #
    # if src_box == dest_box:
    #     path.append((src_point, dest_point))
    #     visited_boxes.append(src_box)
    #     return path, visited_boxes
    #
    # while queue:
    #     box = heappop(queue)[1]
    #
    #     if box == dest_box:
    #         last_point = dest_point
    #         next_point = getClosestPoint(last_point, parent[box])
    #         path.append((last_point, next_point))
    #         last_point = next_point
    #         box = parent[box]
    #         while parent[box] != None:
    #             next_point = getClosestPoint(last_point, parent[box])
    #             path.append((last_point, next_point))
    #             last_point = next_point
    #             box = parent[box]
    #         path.append((last_point, src_point))
    #         path.reverse()
    #
    #         return path, visited_boxes
    #
    #     neighbors = mesh["adj"][box]
    #     for neighbor in neighbors:
    #         weight = getDist(getPoint(box), getPoint(neighbor))
    #         alt = dist[box] + weight
    #         if neighbor not in dist or alt < dist[neighbor]:
    #             dist[neighbor] = alt
    #             parent[neighbor] = box
    #             if neighbor not in queue:
    #                 visited_boxes.append(neighbor)
    #                 heappush(queue, (alt + getDist(getPoint(neighbor), dest_point), neighbor))

    """Dijkstra's"""
    # parent = {}
    # parent[src_box] = None
    # dist = {}
    # dist[src_box] = 0
    # queue = [(0, src_box)]
    # visited_boxes.append(src_box)
    #
    # if src_box == dest_box:
    #     path.append((src_point, dest_point))
    #     visited_boxes.append(src_box)
    #     return path, visited_boxes
    #
    # while queue:
    #     box = heappop(queue)[1]
    #
    #     if box == dest_box:
    #         last_point = dest_point
    #         next_point = getClosestPoint(last_point, parent[box])
    #         path.append((last_point, next_point))
    #         last_point = next_point
    #         box = parent[box]
    #         while parent[box] != None:
    #             next_point = getClosestPoint(last_point, parent[box])
    #             path.append((last_point, next_point))
    #             last_point = next_point
    #             box = parent[box]
    #         path.append((last_point, src_point))
    #         path.reverse()
    #         return path, visited_boxes
    #
    #     neighbors = mesh["adj"][box]
    #     for neighbor in neighbors:
    #         weight = getDist(getPoint(box), getPoint(neighbor))
    #         alt = dist[box] + weight
    #         if neighbor not in dist or alt < dist[neighbor]:
    #             dist[neighbor] = alt
    #             parent[neighbor] = box
    #             if neighbor not in queue:
    #                 visited_boxes.append(neighbor)
    #                 heappush(queue, (dist[neighbor], neighbor))

    """BFS"""
    # queue = deque()
    # queue.append(src_box)
    # disc = {}
    # disc[src_box] = True
    # parent = {}
    # parent[src_box] = None
    # last_point = dest_point
    #
    # if src_box == dest_box:
    #     path.append((src_point, dest_point))
    #     visited_boxes.append(src_box)
    #     return path, visited_boxes
    #
    # while queue:
    #     cur_box = queue.popleft()
    #     for box in mesh["adj"][cur_box]:
    #         if not box in disc:
    #             queue.append(box)
    #             disc[box] = True
    #             visited_boxes.append(box)
    #             parent[box] = cur_box
    #             if box == dest_box:
    #                 last_point = dest_point
    #                 next_point = getClosestPoint(last_point, parent[box])
    #                 path.append((last_point, next_point))
    #                 last_point = next_point
    #                 box = parent[box]
    #                 while parent[box] != None:
    #                     next_point = getClosestPoint(last_point, parent[box])
    #                     path.append((last_point, next_point))
    #                     last_point = next_point
    #                     box = parent[box]
    #                 path.append((last_point, src_point))
    #                 path.reverse()
    #                 return path, visited_boxes



    """Universal End Block"""
    print "No path possible!"
    print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    return [], []

def getClosestPoint(point, box):
    x = point[0]
    y = point[1]

    if x > box[0] and x < box[1]:
        if fabs(y - box[2]) > fabs(y - box[3]):
            y = box[3]
        else:
            y = box[2]
    else:
        if fabs(x - box[0]) > fabs(x - box[1]):
            x = box[1]
        else:
            x = box[0]
        if y < box[2] or y > box[3]:
            if fabs(y - box[2]) > fabs(y - box[3]):
                y = box[3]
            else:
                y = box[2]

    return (x, y)

def getDist(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def getPoint(box):
    return ((box[0] + box[1])/2, (box[2] + box[3])/2)