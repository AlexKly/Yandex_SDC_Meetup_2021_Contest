import numpy as np


test_dir = "sdc_meetup_2021_tests/"
test_file = "01"


# States of FM:
SEARCH_NEAREST_AND_THE_OLDEST_ORDER = "0"
MOVE_ALONG_THE_SHORTEST_PATH = "1"
TAKE_ORDER = "2"
SEARCH_NEAREST_DELIVERY_PATH = "3"
GIVE_ORDER = "4"
IDLE_FREE_ORDER = "5"


# Order status:
ACTIVE = "0"
IN_PROGRESS = "1"
NOT_ACTIVE = "2"


class Robot:
    def __init__(self, pos_x, pos_y, robot_ID, order_ID, state):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.robot_ID = robot_ID
        self.order_ID = order_ID
        self.state = state
        self.current_direction = None
        self.path = ''

    def SearchNearestAndTheOldestOrder(self, order_info, grid):
        list_generation_order = list()
        list_order_ID = list()
        list_min_path = list()
        list_lens_min_path = list()
        list_ind_order = list()
        for i in range(len(order_info)):
            pos_x_order = order_info[i]['Pos_X_start']
            pos_y_order = order_info[i]['Pos_Y_start']
            status_order = order_info[i]['Status']
            generation_order = order_info[i]['Generation']
            order_ID = order_info[i]['Order_ID']
            if status_order == ACTIVE:
                min_path = find_shortest_path(grid=grid, begin=[self.pos_x, self.pos_y], end=[pos_x_order, pos_y_order])
                list_min_path.append(min_path)
                list_lens_min_path.append(len(min_path))
                list_generation_order.append(generation_order)
                list_order_ID.append(order_ID)
                list_ind_order.append(i)
        if len(list_min_path) > 1:
            oldest_generation = np.min(np.array(list_generation_order), axis=0)
            for i in range(len(list_min_path)):
                if list_generation_order[i] != oldest_generation:
                    list_min_path.pop(i)
                    list_lens_min_path.pop(i)
                    list_generation_order.pop(i)
                    list_order_ID.pop(i)
                    list_ind_order.pop(i)
            min_path_ind = np.argmin(np.array(list_lens_min_path), axis=0)
            self.order_ID = list_order_ID[min_path_ind]
            self.current_direction = list_min_path[min_path_ind]
            self.state = MOVE_ALONG_THE_SHORTEST_PATH
            return list_ind_order[min_path_ind]
        elif len(list_min_path) == 1:
            self.order_ID = list_order_ID[0]
            self.current_direction = list_min_path[0]
            self.state = MOVE_ALONG_THE_SHORTEST_PATH
            return list_ind_order[0]
        else:
            self.state = IDLE_FREE_ORDER
            return None

    def Move(self, step, extra):
        if step < len(self.current_direction):
            current_step = self.current_direction[step]
            if current_step == "U":
                self.pos_x = self.pos_x - 1
                self.path = self.path + "U"
            elif current_step == "D":
                self.pos_x = self.pos_x + 1
                self.path = self.path + "D"
            elif current_step == "L":
                self.pos_y = self.pos_y - 1
                self.path = self.path + "L"
            elif current_step == "R":
                self.pos_y = self.pos_y + 1
                self.path = self.path + "R"
            return True
        else:
            if extra == 0:
                self.state = TAKE_ORDER
            else:
                self.state = GIVE_ORDER
            return False

    def TakeOrder(self):
        self.path = self.path + "T"
        self.state = SEARCH_NEAREST_DELIVERY_PATH

    def SearchNearestPath(self, order_info, grid):
        endpoint_X = 0
        endpoint_Y = 0
        for i in range(len(order_info)):
            if order_info[i]['Order_ID'] == self.order_ID:
                endpoint_X = order_info[i]['Pos_X_finish']
                endpoint_Y = order_info[i]['Pos_Y_finish']
        self.current_direction = find_shortest_path(grid=grid, begin=[self.pos_x, self.pos_y], end=[endpoint_X, endpoint_Y])
        self.state = MOVE_ALONG_THE_SHORTEST_PATH

    def GiveOrder(self, max_tips, delivery_time):
        self.path = self.path + "P"
        self.order_ID = None
        self.current_direction = None
        self.state = SEARCH_NEAREST_AND_THE_OLDEST_ORDER

        return np.max(np.array([0, max_tips - delivery_time]), axis=0)

    def Wait(self, sec):
        self.path = self.path + "S"
        if sec == 59:
            print(self.path, flush=True)
            self.path = ''
            self.state = SEARCH_NEAREST_AND_THE_OLDEST_ORDER


def FormedStructOrder(order_ID, status, pos_x_start, pos_y_start, pos_x_finish, pos_y_finish, max_tips, gen):
    return {
        "Order_ID": order_ID,
        "Status": status,
        "Pos_X_start": pos_x_start,
        "Pos_Y_start": pos_y_start,
        "Pos_X_finish": pos_x_finish,
        "Pos_Y_finish": pos_y_finish,
        "Max_Tips": max_tips,
        "Generation": gen
    }


def find_shortest_path(grid: np.array, begin: list, end: list) -> list:
    """ function for finding shortest way between to points in matrix grid - matrix NxN size of points;
        begin, end - lists [x, y] of x and y coords for begin and end points """
    # initialize distance matrix as numpy array NxN
    dist = np.zeros_like(grid)
    # get N
    n = grid.shape[0] - 1
    # create queue add begin point to in, set begin point for distance matrix to 1
    q, dist[begin[0], begin[1]] = [begin], 1
    # while queue is not empty
    while len(q):
        # pop point from queue and get it's coordinates
        curr = q.pop(0)
        i, j = curr[0], curr[1]
        # if reach the end point, exit function
        if i == end[0] and j == end[1]:
            shortest_path = get_shortest_path(dist, n, end)
            return shortest_path
        # check adjacent points in all directions: up, down, left, right
        for c in [[max(i-1, 0), j], [min(i+1, n), j], [i, max(j-1, 0)], [i, min(j+1, n)]]:
            if grid[c[0], c[1]] == 0 and dist[c[0], c[1]] == 0:
                dist[c[0], c[1]] = dist[i, j] + 1
                q.append(c)
    return []


def get_shortest_path(dist: np.array, n: int, end: list) -> list:
    """ Get the shortest path according to distance matrix """
    path = []
    # start from the end point
    i, j = end[0], end[1]
    while dist[i, j] != 1:
        # if upper point has current distance value minus 1 - move to that point
        if dist[max(i-1, 0), j] == dist[i, j] - 1:
            i -= 1
            # add "UP" movement to path
            path.append("D")
        # same the for other points
        elif dist[min(i+1, n), j] == dist[i, j] - 1:
            i += 1
            path.append("U")
        elif dist[i, max(j-1, 0)] == dist[i, j] - 1:
            j -= 1
            path.append("R")
        else:
            j += 1
            path.append("L")
    return path[::-1]


if __name__ == '__main__':
    test_list = list()
    f = open(test_dir + test_file, "r")
    while True:
        line = f.readline()
        if not line:
            break
        test_list.append(line.strip())
    f.close()

    # Input city size, max cost for delivery and cost for building robot
    N, MaxTips, Cost_c = test_list[0].split()
    N, MaxTips, Cost_c = int(N), int(MaxTips), int(Cost_c)

    # Input map of city:
    Map = list()
    for i in range(N):
        Map.append(test_list[i+1].split())

    # Input number of iterations of action and total number of orders:
    T, D = test_list[N+1].split()
    T, D = int(T), int(D)

    # Create adjacency matrix:
    grid = list()
    for i in range(N):
        row_list = list()
        for j in range(N):
            if Map[i][0][j] == '.':
                row_list.append(0)
            else:
                row_list.append(1)
        grid.append(row_list)
    grid = np.array(grid)

    # Number of robots and initial positions:
    R = np.max([1, round(D/T)])
    Robots = list()
    valid_points = list()
    for i in range(N):
        for j in range(N):
            if grid[i][j] == 0:
                valid_points.append([i, j])
    valid_points = np.array(valid_points)
    for i in range(R):
        posXY_Ind = np.random.choice(valid_points.shape[0], size=1)
        posXY = valid_points[posXY_Ind]
        posX = posXY[0][0]
        posY = posXY[0][1]
        Robots.append(Robot(pos_x=posX, pos_y=posY, robot_ID=i, order_ID=None, state=SEARCH_NEAREST_AND_THE_OLDEST_ORDER))

    print(int(R), flush=True)

    for i in range(R):
        print(int(Robots[i].pos_x + 1), int(Robots[i].pos_y + 1), flush=True)

    Orders = list()
    step = np.zeros(R, dtype=int)
    extra = np.zeros(R, dtype=int)
    delivery_cost = np.zeros(R, dtype=int)
    virtual_minute = 60
    for i in range(T):
        # Number of new orders:
        k = test_list[N+2+2*i]
        k = int(k)
        if k != 0:
            for j in range(k):
                # Coordinates start and finish points:
                Srow, Scol, Frow, Fcol = test_list[N+2+2*i+1].split()
                Srow, Scol, Frow, Fcol = int(Srow)-1, int(Scol)-1, int(Frow)-1, int(Fcol)-1
                Orders.append(FormedStructOrder(order_ID=j, status=ACTIVE, pos_x_start=Srow, pos_y_start=Scol, pos_x_finish=Frow, pos_y_finish=Fcol, max_tips=MaxTips, gen=i))
            v_sec = 0
            while v_sec < virtual_minute:
                for j in range(R):
                    # Connect free robots with nearest free orders:
                    if Robots[j].state == SEARCH_NEAREST_AND_THE_OLDEST_ORDER:
                        #print("======== 0 ========")
                        ind_order = Robots[j].SearchNearestAndTheOldestOrder(order_info=Orders, grid=grid)
                        if ind_order is not None:
                            Orders[ind_order]['Status'] = IN_PROGRESS
                            step[j] = 0
                    elif Robots[j].state == MOVE_ALONG_THE_SHORTEST_PATH:
                        #print("======== 1 ========")
                        valid_move = Robots[j].Move(step[j], extra=extra[j])
                        if valid_move is True:
                            v_sec = v_sec + 1
                            step[j] = step[j] + 1
                        else:
                            step[j] = 0
                    elif Robots[j].state == TAKE_ORDER:
                        #print("======== 2 ========")
                        Robots[j].TakeOrder()
                        extra[j] = 1
                        v_sec = v_sec + 1
                    elif Robots[j].state == SEARCH_NEAREST_DELIVERY_PATH:
                        #print("======== 3 ========")
                        Robots[j].SearchNearestPath(order_info=Orders, grid=grid)
                    elif Robots[j].state == GIVE_ORDER:
                        #print("======== 4 ========")
                        Orders[Robots[j].order_ID]['Status'] = NOT_ACTIVE
                        delivery_cost[j] = delivery_cost[j] + Robots[j].GiveOrder(max_tips=MaxTips, delivery_time=v_sec + 1)
                        extra[j] = 0
                        v_sec = v_sec + 1
                    elif Robots[j].state == IDLE_FREE_ORDER:
                        #print("======== 5 ========")
                        Robots[j].Wait(sec=v_sec)
                        v_sec = v_sec + 1
                    #print(Robots[j].state, Robots[j].robot_ID, Robots[j].order_ID, Robots[j].path, Robots[j].current_direction, Robots[j].pos_x + 1, Robots[j].pos_y + 1)
                    #print(Orders)
        else:
            for j in range(R):
                print("S"*60, flush=True)


    total_tips = np.sum(delivery_cost, axis=0)
    total_sum = total_tips - Cost_c * R
    print(total_sum)
