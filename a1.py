import copy


manhattan_values = {
    0: [0, 0],
    1: [0, 1],
    2: [0, 2],
    3: [1, 0],
    4: [1, 1],
    5: [1, 2],
    6: [2, 0],
    7: [2, 1],
    8: [2, 2]
}

class Node:
    def __init__(self, state, path_cost, heuristic, path):
        self.state = state
        self.path_cost = path_cost
        self.heuristic = heuristic
        self.score = heuristic + path_cost
        self.path = path


    def __eq__(self, other):
        return (self.state == other.state) and (self.path_cost == other.path_cost) and (self.path == other.path)


def generate_available_moves(empty_tile_index):
    available_moves = ["u", "d", "l", "r"]

    manhattan_x = manhattan_values[empty_tile_index][0]
    manhattan_y = manhattan_values[empty_tile_index][1]

    if manhattan_x == 0:
        available_moves.remove("l")
    if manhattan_x == 2:
        available_moves.remove("r")
    if manhattan_y == 0:
        available_moves.remove("u")
    if manhattan_y == 2:
        available_moves.remove("d")

    return available_moves


# calculates the heuristic value between two states where its determined by the manhattan distance
def heuristic_value(current_state, goal_state):
    h_score = 0
    for index, element in enumerate(current_state, start=0):
        goal_state_index = goal_state.index(element)
        h_score += abs(manhattan_values[index][0] - manhattan_values[goal_state_index][0]) + abs(manhattan_values[index][1] - manhattan_values[goal_state_index][1])
    return h_score

def transform_state(current_state, empty_tile_index, move):
    # new_state = current_state[:]
    new_state = copy.copy(current_state)

    #u means that 0 has to switch with -1 index
    if move == "u":
        new_state[empty_tile_index], new_state[empty_tile_index - 1] = new_state[empty_tile_index - 1], new_state[empty_tile_index]

    #d means that 0 has to switch with +1 index
    if move == "d":
        new_state[empty_tile_index], new_state[empty_tile_index + 1] = new_state[empty_tile_index + 1], new_state[empty_tile_index]

    #r means that 0 has to switch with +3 index
    if move == "r":
        new_state[empty_tile_index], new_state[empty_tile_index + 3] = new_state[empty_tile_index + 3], new_state[empty_tile_index]


    #l means 0 has to switch with -3 index
    if move == "l":
        new_state[empty_tile_index], new_state[empty_tile_index - 3] = new_state[empty_tile_index - 3], new_state[empty_tile_index]

    return new_state


def astar_search(init_state, goal_state, move_cost):
    frontier = []
    explored = []
    optimal_path = []


    move_costs = {
    "u": move_cost[0],
    "d": move_cost[1],
    "l": move_cost[2],
    "r": move_cost[3]
    }

    start_node = Node(init_state, 0, heuristic_value(init_state, goal_state), [])
    frontier.append(start_node)

    while True:
        minimal_score = -1
        minimal_score_index = 0
        for node in frontier:
            if minimal_score == -1:
                minimal_score = node.score
                minimal_score_index = frontier.index(node)
            if node.score < minimal_score:
                minimal_score = node.score
                minimal_score_index = frontier.index(node)

        node_to_expand = frontier[minimal_score_index]

        path = node_to_expand.path

        if node_to_expand.state == goal_state:
            optimal_path = node_to_expand.path
            break

        available_moves = generate_available_moves(node_to_expand.state.index(0))

        for move in available_moves:
            state = transform_state(node_to_expand.state, node_to_expand.state.index(0), move)
            path_cost = node_to_expand.path_cost + move_costs[move]
            heuristic = heuristic_value(state, goal_state)
            path = copy.copy(node_to_expand.path) + [move]
            new_frontier_node = Node(state, path_cost, heuristic, path)
            if new_frontier_node not in explored:
                frontier.append(new_frontier_node)

        frontier.remove(node_to_expand)
        explored.append(node_to_expand)


    return ''.join(optimal_path)


print(astar_search([0,8,7,1,2,6,3,4,5], [1,8,7,2,0,6,3,4,5], [1,1,1,1]))
print(astar_search([1,8,7,3,0,2,4,5,6], [1,8,7,2,0,6,3,4,5], [1,1,1,1]))
print(astar_search([1,6,0,2,7,8,3,4,5], [1,8,7,2,0,6,3,4,5], [1,1,2,2]))
print(astar_search([1,6,0,2,7,8,3,4,5], [1,8,7,2,0,6,3,4,5], [3,3,1,1]))
print(astar_search([8,0,7,1,4,3,2,5,6], [1,8,7,2,0,6,3,4,5], [1,1,1,1]))
print(astar_search([8,0,7,1,4,3,2,5,6], [1,8,7,2,0,6,3,4,5], [1,1,2,2]))
