def board():
    '''
    DOCSTRING: A function to print out the initial visual of 6-Puzzle board with the initial configuration given.
    We return a list out of this.
    input: N/A
    output: A list of a visual representation of the board, and printing that list to get the required visuals
    '''

    # a dictionary of types of characters used to build the board. b = blank, h = horizontal, v = vertical
    dict1 = {'b': ' ', 'h': '_', 'v': '|'}

    # defining strings of the different sort of lines
    my_line1 = dict1['h'] * 13
    my_line2 = dict1['v'] + dict1['b'] * 3 + dict1['v'] + dict1['b'] * 3 + dict1['v'] + dict1['b'] * 3 + dict1['v']
    my_line3 = dict1['v'] + dict1['h'] * 3 + dict1['v'] + dict1['h'] * 3 + dict1['v'] + dict1['h'] * 3 + dict1['v']

    # defining a list that holds this board
    my_list = []
    # printing lines repeatedly in a pattern
    for line in range(1, 8):
        if line == 1:
            my_list.append(my_line1)
        elif line in (4, 7):
            my_list.append(my_line3)
        else:
            my_list.append(my_line2)

    return my_list


def visual(my_list, state):
    # visualizing the board with a given configuration
    new_list = loader(my_list, state)

    for index in range(0, 7):
        print(new_list[index])

    return new_list


def loader(prev_list, configuration):
    '''
    DOCSTRING: This is to load in a configuration of the board and change the state of the board list
    input: previous board list, needed configuration
    :return: changed state of board list
    '''

    new_list = prev_list
    visual_config = configuration.copy()

    for i in range(len(visual_config)):
        if visual_config[i] == -1:
            visual_config[i] = ' '
    new_list[2] = new_list[2][:2] + str(visual_config[0]) + new_list[2][3:6] + str(visual_config[1]) + new_list[2][
                                                                                                       7:10] + str(
        visual_config[2]) + new_list[2][11:]
    new_list[5] = new_list[5][:2] + str(visual_config[3]) + new_list[5][3:6] + str(visual_config[4]) + new_list[5][
                                                                                                       7:10] + str(
        visual_config[5]) + new_list[5][11:]

    return new_list


def possible(current_state):
    # a function to determine what are the possible moves/transitions from a given state.
    # the moves are left, right, up, and down - l,r,u,d

    moves = []
    empty_space = current_state.index(-1)

    if empty_space == 0:
        moves.append('l')
        moves.append('u')
    elif empty_space == 1:
        moves.append('l')
        moves.append('r')
        moves.append('u')
    elif empty_space == 2:
        moves.append('r')
        moves.append('u')
    if empty_space == 3:
        moves.append('l')
        moves.append('d')
    elif empty_space == 4:
        moves.append('l')
        moves.append('r')
        moves.append('d')
    elif empty_space == 5:
        moves.append('r')
        moves.append('d')

    return moves


def generateState(current_state, possible_moves):
    # a function to generate a new state based on possible moves available

    empty_space = current_state.index(-1)
    possible_states = []
    number_moved = []
    for possibilities in possible_moves:
        state = current_state.copy()
        if possibilities == 'r':
            t = state[empty_space - 1]
            state[empty_space - 1] = -1
            state[empty_space] = t
            possible_states.append(state)
            number_moved.append(t)
        elif possibilities == 'l':
            t = state[empty_space + 1]
            state[empty_space + 1] = -1
            state[empty_space] = t
            possible_states.append(state)
            number_moved.append(t)
        elif possibilities == 'u':
            t = state[empty_space + 3]
            state[empty_space + 3] = -1
            state[empty_space] = t
            possible_states.append(state)
            number_moved.append(t)
        else:
            t = state[empty_space - 3]
            state[empty_space - 3] = -1
            state[empty_space] = t
            possible_states.append(state)
            number_moved.append(t)

    # Applying a sort method for ensuring that possibilities are listed in ascending order
    # i.e. smaller tile moving is preferred
    zipped_lists = zip(number_moved, possible_states)
    sorted_pairs = sorted(zipped_lists)

    tuples = zip(*sorted_pairs)
    list2, possible_states_sorted = [list(tuple) for tuple in tuples]
    return possible_states_sorted


def backtrace(parent, start, end):
    path = [end]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path


def BFS(graph, start, end):
    parent = {}
    queue = []
    visited_nodes = []
    queue.append(start)
    while queue:
        node = queue.pop(0)
        if str(node) == end:
            return backtrace(parent, start, end)
        elif node not in visited_nodes:
            visited_nodes.append(node)
            for adjacent in graph.get(node, []):
                if adjacent not in queue and adjacent not in visited_nodes:
                    parent[str(adjacent)] = node  # <<<<< record its parent
                    queue.append(str(adjacent))


def DFS(graph, start, goal):
    stack = [(start, [start])]
    visited = set()

    while stack:
        (vertex, path) = stack.pop()
        if vertex not in visited:
            if vertex == goal:
                return path
            visited.add(vertex)
            for i in range(len(graph[vertex]) - 1, -1, -1):
                neighbor = graph[vertex][i]
                stack.append((str(neighbor), path + [neighbor]))


def IDS(graph, start, goal, depth):
    limit = 0
    stack = [(start, [start], limit)]
    visited = set()

    while stack:
        (vertex, path, limit) = stack.pop()
        if vertex not in visited:
            while limit == depth - 1:
                if vertex == goal:
                    return path
                if not stack:
                    break
                (vertex, path, limit) = stack.pop()

            if vertex == goal:
                return path

            if limit != depth - 1:
                limit = limit + 1
                visited.add(vertex)
                for i in range(len(graph[vertex]) - 1, -1, -1):
                    neighbor = graph[vertex][i]
                    stack.append((str(neighbor), path + [neighbor], limit))
    return '$$'


def buildTree(possible_states_list, goal_state, visited_state, building_dict):
    for possible_state in possible_states_list:
        p = str(possible_state)
        if possible_state in visited_state:
            continue
        elif possible_state == goal_state:
            continue
        else:
            pass

        possible_moves = possible(possible_state)
        possible_states_list = generateState(possible_state, possible_moves)

        for visits in visited_state:
            try:
                possible_states_list.remove(visits)
            except:
                pass
        building_dict[p] = possible_states_list
        visited_state.append(possible_state)
        buildTree(possible_states_list, goal_state, visited_state, building_dict)
    return building_dict


# Main function positioned at the end of all code
if __name__ == "__main__":

    # index locations in list from left to right and then top to bottom. A -1 denotes an empty tile
    initial_state = [1, 4, 2, 5, 3, -1]
    goal_state = [-1, 1, 2, 5, 4, 3]

    board = board()
    visual(board, initial_state)

    possible_moves = possible(initial_state)
    possible_states_list = generateState(initial_state, possible_moves)

    # Generating a search tree. 1 is a key corresponding to the initial state
    d = {}
    p = str(initial_state)
    d[p] = possible_states_list

    visited_state = [initial_state]
    search_tree = buildTree(possible_states_list, goal_state, visited_state, d)

    # for possible_state in possible_states_list:
    #    current_board = visual(board, possible_state)

    choice = int(input(
        "\nWhich method of search would you like?\n1. Breadth First Search\n2. Uniform Cost Search\n3. Depth First Search\n4. Iterative Deepening Search\n"))
    if choice == 1:
        path = BFS(search_tree, p, str(goal_state))
        print(f'Path: {path} \nLength: {len(path)}')

        for state in path:
            redef_state = state[1:-1].split(',')
            redef_state_2 = list(map(int,redef_state))
            visual(board,redef_state_2)

    elif choice == 2:
        # Uniform cost search for a problem where all operations are unit means that BFS = Uniform Cost Search
        path = BFS(search_tree, p, str(goal_state))
        print(f'Path: {path} \nLength: {len(path)}')

        for state in path:
            redef_state = state[1:-1].split(',')
            redef_state_2 = list(map(int,redef_state))
            visual(board,redef_state_2)

    elif choice == 3:
        # Depth first search
        path = DFS(search_tree, p, str(goal_state))
        print(f'Path: {path} \nLength: {len(path)}')

        redef_path = path.copy()
        redef_path[0] = list(map(int,redef_path[0][1:-1].split(',')))

        for state in redef_path:
            visual(board,state)

    else:
        # Iterative Deepening Search
        depth = 0
        to_continue = True
        while to_continue:
            depth = depth + 1
            path = IDS(search_tree, p, str(goal_state), depth)
            if path != '$$':
                to_continue = False
        print(f'Path: {path} \nLength: {len(path)}')

        redef_path = path.copy()
        redef_path[0] = list(map(int, redef_path[0][1:-1].split(',')))

        for state in redef_path:
            visual(board, state)
