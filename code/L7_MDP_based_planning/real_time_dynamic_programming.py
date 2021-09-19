import pickle
from racetracks import *
from graph_node import Node
import matplotlib.pyplot as plt


seed = np.random.seed(1234)
graph = {}


def explore_action(u_idx, epsilon=0.2):
    if np.random.uniform(0, 1) < epsilon:
        return np.random.randint(0, len(ACTION_SPACE))
    else:
        return u_idx


def build_up_graph(grid, save_path):
    max_vel = 5

    # velocity dimension
    vel_list = []
    for i_vel in range(-max_vel + 1, max_vel):
        for j_vel in range(-max_vel + 1, max_vel):
            vel_list.append([i_vel, j_vel])

    # position dimension
    x_idx, y_idx = np.where(grid == FREE)
    coord = np.stack([x_idx, y_idx], axis=1)
    for p_idx in range(coord.shape[0]):
        pnt = coord[p_idx]
        for vel in vel_list:
            state = Node(pnt[0], pnt[1], vel[0], vel[1])
            m_dist = np.abs(np.asarray(FINISH_LINE) - np.array([state.px, state.py]))

            # IMPORTANT-1 Heuristic Function design here
            # TO BE IMPLEMENTED
            
            # Note: Both the two heuristics are not the best
            # example-1 heuristic = np.linalg.norm(m_dist, axis=1)  # Euclidean distance
            # example-2 heuristic = m_dist[:, 0] + m_dist[:, 1]  # Mahalonobis distance

            state.g_value = np.min(heuristic)
            state.connect_to_graph(grid)
            graph[state.key] = state

    for pnt in START_LINE:
        state = Node(pnt[0], pnt[1], 0, 0)
        heuristic = np.linalg.norm(np.asarray(FINISH_LINE) - np.array([state.px, state.py]), axis=1)
        state.g_value = np.min(heuristic)
        state.connect_to_graph(grid)
        graph[state.key] = state

    for pnt in FINISH_LINE:
        state = Node(pnt[0], pnt[1], 0, 0)
        state.is_goal = True
        graph[state.key] = state

    output = open(save_path, 'wb')
    pickle.dump(graph, output)


def track_the_best_plan(idx = 0):
    start_node = Node(START_LINE[idx][0], START_LINE[idx][1], 0, 0)
    start_key = start_node.key
    state = graph[start_key]
    trajectory = [state]
    # for i in range(grid.shape[0]+grid.shape[1]) a safer condition
    while not state.is_goal:
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):
            child_key_9 = state.next_prob_9[child_idx]
            child_9 = graph[child_key_9]
            value_uk.append(child_9.g_value)
        child_key = state.next_prob_9[np.argmin(value_uk)]
        state = graph[child_key]
        trajectory.append(state)
        print(state.px, state.py)
    return trajectory



def visualize_the_best_plan(plan, grid_para):
    assert isinstance(plan, list)
    plt.figure(figsize=(4.5, 16))
    plt.pcolor(grid_para, edgecolors='k', linewidths=1)
    plan_len = len(plan)
    plan.append(plan[-1])
    for i in range(plan_len):
        plt.arrow(plan[i].py + 0.5, plan[i].px + 0.5,
                  plan[i+1].py - plan[i].py, plan[i+1].px - plan[i].px,
                  color='r', head_width=0.3, head_length=0.1)
    plt.show()



def greedy_policy(idx=0, explore=True):
    start_node = Node(START_LINE[idx][0], START_LINE[idx][1], 0, 0)
    start_key = start_node.key
    state = graph[start_key]
    trajectory = [state.key]
    while not state.is_goal:
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):
            child_key_9 = state.next_prob_9[child_idx]
            child_9 = graph[child_key_9]
            value_uk.append(child_9.g_value)

        action_idx = np.argmin(value_uk)
        if explore:
            action_idx = explore_action(action_idx)
        child_key = state.next_prob_9[action_idx]
        trajectory.append(child_key)
        state = graph[child_key]
        # if [state.px, state.py] in START_LINE:
        #     trajectory = [state.key]
        print('finding feasible path: {}, {}'.format(state.px, state.py))
    # print('found trajectory: {}'.format(trajectory))
    return trajectory


def real_time_dynamic_programming():
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []

    # IMPORTAN-2: implement RTDP
    # while bellman_error > 0.0001:
    for i in range(500): # YOU MAY CHANGE THIS VALUE
        itr_num += 1
        bellman_error = 0.0
        rand_start = np.random.randint(low=0, high=3, size=1)[0]
        greedy_plan = greedy_policy(idx=rand_start)
        for key in greedy_plan:
            state = graph[key]
            if state.is_goal:
                state.g_value = 0
            else:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    child_key_9 = state.next_prob_9[child_idx]
                    child_9 = graph[child_key_9]
                    child_key_1 = state.next_prob_1[child_idx]
                    child_1 = graph[child_key_1]

                    # TO BE IMPLEMENTED
                    expected_cost_uk = ?
                    value_uk.append(expected_cost_uk)

                # TO BE IMPLEMENTED
                current_value = ?
                bellman_error = ?

                # TO BE IMPLEMENTED
                state.g_value = ?
                
            # end if
        # end for
        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
    # end while

    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()


if __name__ == '__main__':
    path = './solution/graph_rtdp.dat'
    track_map = race_track
    # build_up_graph(track_map, path)
    pkl_file = open(path, 'rb')
    graph = pickle.load(pkl_file)
    real_time_dynamic_programming()
    plan = track_the_best_plan(idx=3)
    visualize_the_best_plan(plan, track_map)


