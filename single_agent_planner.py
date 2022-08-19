import heapq
from operator import ge

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = dict()
    if constraints:
        for constraint in constraints:
            if 'positive' not in constraint:
                constraint['positive'] = False

            if constraint['agent'] == agent:
                if constraint['timestep'] in constraint_table:
                    constraint_table[constraint['timestep']].append(constraint)
                else:
                    constraint_table[constraint['timestep']] = [constraint]
            else:
                if constraint['positive']:
                    converted = {'agent': constraint['agent'],
                                 'loc': constraint['loc'],
                                 'timestep': constraint['timestep'],
                                 'positive': False}
                    if len(constraint['loc']) == 2:
                        converted['loc'] = [constraint['loc'][1], constraint['loc'][0]]
                        
                    if converted['timestep'] in constraint_table:
                        constraint_table[converted['timestep']].append(converted)
                    else:
                        constraint_table[converted['timestep']] = [converted]

    # print(constraint_table)
    return constraint_table


# def interval(state):



def split_SI(timestep, SI):
    for interval in SI:
        if timestep >= interval[0] and timestep <= interval[1]:
            SI.remove(interval)
            if timestep == interval[0] and timestep == interval[1]:
                continue
            elif timestep == interval[0] and timestep < interval[1]:
                SI.append((timestep+1, interval[1]))
            else:
                SI.append((interval[0], timestep-1))
                SI.append((timestep+1, interval[1]))

# example SI = [(0,2), (4,7), (9, inf)]
def build_SI_table(constraint_table):
    SI_table = {}
    for t in constraint_table:
        constraint = constraint_table[t][0]
        # print(constraint)
        if constraint['positive'] == False and len(constraint['loc']) == 1:
            loc = constraint['loc'][0]
            if loc not in SI_table:
                SI_table[loc] = [(0, float('inf'))]
            # print(SI_table)
            split_SI(constraint['timestep'], SI_table[loc])
            # print(SI_table)
    return SI_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time in constraint_table:
        constraints = constraint_table[next_time]
        for constraint in constraints:
            if constraint['positive']:
                if len(constraint['loc']) == 1 and constraint['loc'] != [next_loc]:
                    return True
                elif len(constraint['loc']) == 2 and constraint['loc'] != [curr_loc, next_loc]:
                    return True

            else:
                if len(constraint['loc']) == 1 and constraint['loc'] == [next_loc]:
                    return True
                elif len(constraint['loc']) == 2 and constraint['loc'] == [curr_loc, next_loc]:
                    return True

    return False

# child = {'loc': child_loc,
#         'interval': i,
#         'earliest_arrival': t,
#         'timestep': curr['timestep']+1,
#         'parent': curr}

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['g_val'], node['loc'], node['interval'], node['earliest_arrival'], node['timestep'], node))


def pop_node(open_list):
    _, _, _, _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    h, l = len(my_map), len(my_map[0])
    limit = 1
    for row in my_map:
        for col in row:
            if not col:
                limit += 1

    # print('\n======================= Agent', agent, '=======================')
    constraint_table = build_constraint_table(constraints, agent)
    # print("constraint_table")
    # print(constraint_table, '\n')
    SI_table = build_SI_table(constraint_table)
    print("SI_table")
    print(SI_table)

    open_list = []
    closed_list = dict()
    if constraint_table:
        earliest_goal_timestep = max(constraint_table)
    else:
        earliest_goal_timestep = limit
    h_value = h_values[start_loc]
    try:
        root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'interval': SI_table[start_loc][0], 'earliest_arrival': 0,'timestep': 0}
    except:
        root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'interval': (0, 'inf'), 'earliest_arrival': 0,'timestep': 0}

    push_node(open_list, root)
    closed_list[(root['loc'], root['interval'])] = root

    while len(open_list) > 0:
        # print(open_list)
        try:
            curr = pop_node(open_list)
            # print(curr)
            # print(agent)
            # print(goal_loc)
        except:
            print(*open_list, sep = "\n")
            exit()
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints

        if curr['timestep'] > limit:
            return None
        if curr['loc'] == goal_loc:
            if not constraint_table:
                print('===================================================================================================================================================')
                print('============================================================== FOUND PATH FOR', agent, '==============================================================')
                print('===================================================================================================================================================\n\n')
                return get_path(curr)
            if curr['interval'][1] == float('inf'):
                print('===================================================================================================================================================')
                print('============================================================== FOUND PATH FOR', agent, '==============================================================')
                print('===================================================================================================================================================\n\n')
                return get_path(curr)
            # earliest_goal_timestep = max(constraint_table)
            # if curr['timestep'] >= limit or curr['timestep'] >= earliest_goal_timestep:
            #     # cut off the wait time after all agents reach their goal, added after 2.3
            #     while curr['parent']:
            #         if curr['loc'] != curr['parent']['loc']:
            #             break
            #         curr = curr['parent']
            #     print('===================================================================================================================================================')
            #     print('============================================================== FOUND PATH FOR', agent, '==============================================================')
            #     print('===================================================================================================================================================\n\n')
            #     return get_path(curr)

        # getSuccessors
        successors = []
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= h or child_loc[1] < 0 or child_loc[1] >= l: # out of the map
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            # if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
            #     continue
            
            m_time = 1
            start_t = curr['timestep'] + 1
            if curr['interval'][1] == 'inf':
                end_t = float('inf')
            else:
                end_t = curr['interval'][1] + m_time
            
            if child_loc not in SI_table:
                SI_table[child_loc] = [(0, float('inf'))]

            # print('\n',child_loc,SI_table[child_loc])
            for i in SI_table[child_loc]:
                if i[0] > end_t or i[1] < start_t:
                    # print(i)
                    # print(start_t, end_t)
                    continue

                if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
                    continue
                t = max(i[0], start_t)
                # if (child_loc, curr['timestep']+1) not in closed_list:
                #     g = float('inf')
                #     h = float('inf')

                # child = {'loc': child_loc,
                #         'interval': i,
                #         'earliest_arrival': t,
                #         'timestep': curr['timestep']+1,
                #         'g_val': float('inf'),
                #         'parent': curr}
                # g = float('inf')
                # if (child_loc, i) in closed_list:
                #     g = closed_list[(child_loc, i)]['g_val']
                
                child = {'loc': child_loc,
                        'interval': i,
                        'earliest_arrival': t,
                        'timestep': curr['timestep']+1,
                        'parent': curr}
                

                successors.append(child)

        # print(successors)
        print('=================================================================', curr['loc'], curr['interval'], curr['timestep'], '=================================================================')
        for s in successors:
            # if s['loc'] == (2,3):
            #     print(s)
            s['g_val'] = float('inf')
            if (s['loc'], s['interval']) in closed_list:
                s['g_val'] = closed_list[(s['loc'], s['interval'])]['g_val']

            print(s)
            if s['g_val'] > curr['g_val'] + 1:
                s['g_val'] = curr['g_val'] + 1
                s['h_val'] = h_values[s['loc']]
                # s['timestep'] = s['timestep'] +1
                # closed_list[(s['loc'], s['timestep'])] = s
                print('===================================================\n', agent, 'PUSH \n===================================================')
                print(s, '\n')
                closed_list[(s['loc'], s['interval'])] = s
                push_node(open_list, s)
                # if (s['loc'], s['interval']) in closed_list:
                #     existing_node = closed_list[(s['loc'], s['interval'])]
                #     if compare_nodes(s, existing_node):
                #         closed_list[(s['loc'], s['interval'])] = s
                #         push_node(open_list, s)
                # else:
                #     closed_list[(s['loc'], s['interval'])] = s
                #     push_node(open_list, s)
            else:
                print('\n')


    return None  # Failed to find solutions
