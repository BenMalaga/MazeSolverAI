#
# Project 1 - Intro to AI
#
# Benjamin Malaga - bdm104@scarletmail.rutgers.edu
#

import random
from collections import deque
from queue import PriorityQueue
import matplotlib.pyplot as plt

# Dimension and spreadability of the ship and its fire
dimension = 50
q = 0.0

# The current fire cells and adjacent potential fire cells
fire_cells = []
adj_fire_cells = []

# Location of the button and the robot in a given ship
bot_loc = []
button_loc = []

# List of successful runs to be used for the simulations
success_list = [[],[],[],[]]

# Global dictionaries to store the fire map simulations for bot four
fire_map = dict()

# Dictionary to store the bfs distances for bot 4 costs
distances_from_goal = dict()

# Main method
def main ():
    global q
    global success_list

    val_q = [] # Stores the values of q for each simulation to plot later on
    num_success = 0
    num_sims = 250

    for i in range(0, 21):
        q = i/20
        val_q.append(q)

        # Stores the successes in a list of lists
        for i in range(1, 5):
            print("Successes: ", success_list)
            num_success = create_simulation(i, num_sims)
            success_list[i-1].append(num_success/num_sims)

    

    plot_simulation(val_q)


# Run a certain number of simulations for each robot, and count the successes
def create_simulation(bot_num, num_sims):
    global q
    global fire_cells
    global adj_fire_cells

    num_success = 0

    for n in range(0, num_sims):
        fire_cells = []
        adj_fire_cells = []
        ship = create_ship()

        if run_bot_simulation(ship, bot_num, q):
            num_success+=1

    return num_success
    
# Plot the simulations of each bot on the same graph
def plot_simulation(val_q):
    global success_list

    # Each robot gets plotted
    plt.plot(val_q, success_list[0], label = 'Bot One')
    plt.plot(val_q, success_list[1], label = 'Bot Two')
    plt.plot(val_q, success_list[2], label = 'Bot Three')
    plt.plot(val_q, success_list[3], label = 'Bot Four')

    plt.plot()

    # Labels for the axis
    plt.xlabel('Spreadability - q')
    plt.ylabel('Success Rate')

    plt.title('Success Rate for the Bots at different values of q with Dimension = 50')

    plt.legend()
    plt.show()

    return

# Creates the ship, generating the open and closed cells according to the rule set, and adding the bot, fire, and button
def create_ship():
    global fire_cells
    global adj_fire_cells

    ship = [['X' for _ in range(dimension)] for _ in range(dimension)] # 2D array of closed cells, dimension x dimension large

    rand_row = int(random.random() * (dimension))
    rand_col = int(random.random() * (dimension))

    # Fringes to contain the current closed/open cells in the ship while generating
    ship[rand_row][rand_col] = '_'

    # Intialize the fringes for open and closed cells
    fringe_close = []
    fringe_open = [[rand_row, rand_col]]
    fringe_new_close = []

    # Initialize the close fringe with the neighbors of the open cell
    fringe_close = find_neighbors(ship, rand_row, rand_col, 'closed')

    # While there are still valid closed cells, iterate
    while fringe_close:

        # Pick a random valid closed cell, and open it
        c = random.choice(fringe_close) 
        fringe_close.remove(c)
        fringe_open.append(c)
        ship[c[0]][c[1]] = '_'

        # For every open cell, add its surrounding closed cells to a fringe
        neighbors = find_neighbors(ship, c[0], c[1], 'closed')
        for n in neighbors:
            if n not in fringe_close:
                fringe_close.append(n)

        # For every closed cells, ensure that it only has one open neighbor
        for f in fringe_close:
            if len(find_neighbors(ship, f[0], f[1], 'open')) == 1:
                fringe_new_close.append(f)

        # New list to transfer over valid closed cells
        fringe_close = fringe_new_close
        fringe_new_close = []

    # Go through all open cells to check for dead ends   
    for f in fringe_open:
        neighbors = find_neighbors(ship, f[0], f[1], 'closed')
        max_neighbors = 3 # Max number of closed neighbors allowed around a cell
        row = f[0]
        col = f[1]

        # Checks for corners or edges to adjust the max number of closed neighbors
        if (row == dimension - 1 or row == 0) and (col == dimension - 1 or col == 0):
            max_neighbors = 1
        elif row == dimension - 1 or row == 0 or col == dimension - 1 or col == 0:
            max_neighbors = 2

        # If the cell is surrounded by max neighbors, open one of its neighbors
        if len(neighbors) == max_neighbors:
            c = random.choice(neighbors) 
            fringe_open.append(c)
            ship[c[0]][c[1]] = '_'

    # Add the robot
    c = random.choice(fringe_open)
    ship[c[0]][c[1]] = 'R'
    global bot_loc
    bot_loc = c
    fringe_open.remove(c)

    # Add the goal
    c = random.choice(fringe_open)
    ship[c[0]][c[1]] = 'B'
    global button_loc
    button_loc = c
    fringe_open.remove(c)

    # Add the fire
    c = random.choice(fringe_open)
    ship[c[0]][c[1]] = 'F'
    fire_loc = c
    fire_cells.append(c)
    adj_fire_cells = find_neighbors(ship, c[0], c[1], 'open')
    fringe_open.remove(c)

    return ship


# Prints ship
def print_ship(ship):
    count = 0
    print("  | 0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 |") # Legend for the columns of a 10x10 ship
    for row in ship:
        print(count, row)
        count+=1
    print()
    

# Returns the neighboring opened/closed/fire cells to a given cell (Button cell is an open cell)
def find_neighbors(ship, row, col, state):
    cells = []
    directions = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # Up, down, left, and right weights

    for dr, dc in directions:
        r, c = row + dr, col + dc # Row and column
        if 0 <= r < dimension and 0 <= c < dimension:
            if (state == 'closed' and ship[r][c] == 'X') or \
               (state == 'open' and ship[r][c] == '_') or \
               (state == 'search' and (ship[r][c] == '_' or ship[r][c] == 'B')) or \
               (state == 'fire' and ship[r][c] == 'F'):
                cells.append((r, c))
    
    return cells


# Spread the fire based on the spreadability specification
def spread_fire(ship, q):
    global fire_cells
    global adj_fire_cells

    new_fire_cells = [] # Array to copy in the fire cells, so that the fire doesnt catch while the simulation is running; messes up the formula
 
    # Go through every fire cell and find its neighbors, take random neighbor to ignite on fire
    for n in adj_fire_cells:
        # Equation to calculate probability of cell catching fire
        K = len(find_neighbors(ship, n[0], n[1], 'fire'))
        p = 1 - pow((1 - q), K)
        r = random.random()

        # Catch cell on fire
        if r <= p: 
            if n not in fire_cells:
                if ship[n[0]][n[1]] == 'R':
                    return True
                new_fire_cells.append(n)

    # Calculate adjacent fire cells
    for n in new_fire_cells:
        fire_cells.append(n)
        adj_fire_cells.remove(n)
        neighbors = find_neighbors(ship, n[0], n[1], 'open')
        for f in neighbors:
            if f not in adj_fire_cells:
                adj_fire_cells.append(f)
        ship[n[0]][n[1]] = 'F'

    return False


# BFS Search for the ship, varies the conditions based on if we check for adjacent fire cells
def bfs (ship, start, check_adj_fire):
    fringe = [(start, [start])]
    visited = []

    # Uninformed BFS Search for the ship, using neighbors of a node as children nodes
    # Keeps track of the coordinates in the ship that correspond to the shortest path after algo is run
    while fringe:
        curr, path = fringe.pop(0)

        if curr == button_loc:
            path.pop(0)
            return path

        neighbors = find_neighbors(ship, curr[0], curr[1], 'search')
        for n in neighbors:
            if n not in visited:
                # Checks adj fire cells if necessary, only used for bot three
                if not (check_adj_fire and n in adj_fire_cells):
                    fringe.append((n, path + [n])) 
                visited.append(n)

    return False

# Checks if the button is at all reachable from the current location
def button_reachable(ship, curr_loc):
    global button_loc
    global fire_cells
    if button_surrounded(ship) or not bfs(ship, curr_loc, False): return False
    
    return True

# Checks if the button is completely surrounded by fire
def button_surrounded(ship):
    global button_loc
    return len(find_neighbors(ship, button_loc[0], button_loc[1], 'open')) == 0

# Runs a simulation of a robot depending on the robot number
def run_bot_simulation(ship, bot_num, q):
    path = []

    # If ship is immedietely unsolvable, return failure
    p1 = bfs(ship, bot_loc, False)
    p2 = bfs(ship, fire_cells[0], False)
    if p1 and p2 and len(p1) < len(p2):
        return True

    check_adj_cells = False
    if bot_num == 3:
        check_adj_cells = True

    curr_bot_loc = bot_loc

    # Specific guidance for rules depending on which bot is being played
    if bot_num == 1:
        path = bfs(ship, bot_loc, False)
        
    # Run simulation while path is not empty  
    while True:
        # If the button isnt reachable, the bot fails
        if not button_reachable(ship, curr_bot_loc):
            return False

        # Recalculates path for bot 2 and 3
        if bot_num == 2 or bot_num == 3:
            path = bfs(ship, curr_bot_loc, check_adj_cells)
        
        # If path not reachable by checking adjacent fire cells, check again
        if bot_num == 3:
            if not path:
                path = bfs(ship, curr_bot_loc, False)

        if bot_num == 4:
            if p2 and q != 0.0:
                generate_fire_map(ship, curr_bot_loc, len(p2))
            if bfs(ship, curr_bot_loc, False):
                path = a_star(ship, curr_bot_loc)

        # If no path can be generated, the bot fails
        if not path:
            return False

        step = path.pop(0)

        # If it found button returns success, otherwise if it finds fire return failure
        if step == button_loc:
            return True
        if ship[step[0]][step[1]] == 'F':
            return False
        
        # Updates the R token in the 2D ship array, for visualization purpose
        ship[curr_bot_loc[0]][curr_bot_loc[1]] = '_'
        ship[step[0]][step[1]] = 'R'
        curr_bot_loc = step

        if spread_fire(ship, q):
            return False
    
    return False


# A* algorithm for bot four, uses a cost method that runs simulations of how the fire will spread using the given q value. 
def a_star(ship, start):
    global parent
    global button_loc
    global distances_from_goal

    start = tuple(start) # Casts the value into a tuple

    fringe = PriorityQueue()
    distTo = dict() 
    visited = set()
    parent = dict()

    fringe.put(start)

    distTo[start] = 0 
    parent[start] = None

    generate_bfs_distances(ship, button_loc)
    
    while fringe.queue:
        
        curr = fringe.get()

        if curr == button_loc:

            # Backtrack through the parents to find the shortest path
            path = []
            while curr:
                if curr == start:
                    break
                path.append(curr)
                curr = parent.get(curr)
                
            return path[::-1]

        neighbors = find_neighbors(ship, curr[0], curr[1], 'search')

        for child in neighbors:
            tempDist = distTo[curr] + Cost(child) # Cost from start to current node plus the cost from current node to its child

            if child not in visited or tempDist < distTo[child]: # If this child has never been seen or has a lesser path
                distTo.update({child : tempDist})
                parent.update({child : curr})
                fringe.put(child, distTo[child] + h(child, button_loc, distances_from_goal))
                visited.add(child)
    
    return False

# Heuristic for the A* algorithm above, uses the bfs distances
def h(cell, goal, distances_from_goal):
    return distances_from_goal.get(cell, distance_to(cell, goal))

# Generate a hashmap of the bfs distances of each cell to the goal cell (button location)
def generate_bfs_distances(ship, goal):
    global distances_from_goal 

    # Setting up the fringe and visited structures for bfs
    fringe = deque([(goal, 1)]) 
    visited = set([goal])

    while fringe:
        current, distance = fringe.popleft()
        distances_from_goal[current] = distance

        neighbors = find_neighbors(ship, current[0], current[1], 'search')
        for n in neighbors:
            if n not in visited:
                visited.add(n)
                fringe.append((n, distance + 1))

# Generates the HashMap of fire costs given a complete simulation of how the fire will spread over time
def generate_fire_map(ship, curr_loc, path):
    global fire_cells
    global adj_fire_cells
    global fire_map
    global button_loc

    # Copies of the initial array just to have for the fire simulation, will reset at the end
    ship_copy = [row[:] for row in ship]
    ship_copy[curr_loc[0]][curr_loc[1]] = '_'

    adj_copy = [cell for cell in adj_fire_cells]
    fire_copy = [cell for cell in fire_cells]

    # Runs simulation of the fire spread, fire spread doesn't exceed dimension*dimension time steps 
    for i in range(1, dimension*dimension):
        for f in adj_fire_cells:

            # Calculates probability using the fire spread formula
            K = len(find_neighbors(ship_copy, f[0], f[1], 'fire'))
            p = 1 - pow((1 - q), K)
                
            # Update the fire cost based on the probability and distance
            fire_cost = i * p / distance_to(f, button_loc)
            
            # Update the fire cost in the fire_map
            fire_map[f] = max(fire_cost, fire_map.get(f, 0))

        spread_fire(ship_copy, 1)

    # Resets the fire cells and adjacent fire cells to their previous values
    fire_cells = fire_copy
    adj_fire_cells = adj_copy

    return

# The cost of choosing a cell for the A* algorithm based on the distance from the button, and the fire cost from the HashMap generated from simulating the fire
def Cost(cell): 
    global button_loc
    global distances_from_goal
    distance_factor = distances_from_goal.get(cell, distance_to(cell, button_loc)) # The distance from the cell to the button_loc
    fire_risk = fire_map.get(cell, 0) # The risk of the cell catching on fire

    # Cost based on a combination of the distance it will take to get to the button, and the risk of catching on fire
    return 1 - (1 / distance_factor) + fire_risk

# Manhattan Distance calculation
def distance_to(cell, goal):
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1]) + 1 

main()
