#
# Project 2 - Intro to AI
#
# Benjamin Malaga - bdm104@scarletmail.rutgers.edu
#

import random
from collections import deque
from queue import PriorityQueue
import matplotlib.pyplot as plt
import math

# Dimension and spreadability of the ship and its fire
dimension = 30

# Location of the leak and the robot in a given ship
bot_loc = []
leak_loc = []

# List of successful runs to be used for the simulations
success_list = [[], [], []]

# The open cells for ship generation
fringe_open = []

# The cells found in a deterministic sense action (called box search because the sense action scans a square area of cells)
box_search = []

# Stores all cells that could possibly contain the leak. Any open cells not in the list are considered to not contain the leak
possible_leak = []

# Booleans for if the leak was found, and if the first leak was found (for bots with more than one leak to find)
leak_found = False
first_leak_found = False

# The current phase of the backtracking algorithm for Bot 2 and Bot 6
backtrack_phase = 0

# A hashmap of each cell and its corresponding probability of containing the leak according to the current knowledge base
probability_map = dict()

# Stores the probability that is currently the highest probability of containing a leak
highest_prob = -1.0

# Dictionary to store the bfs distances for bot 4 costs
distances_from_goal = dict()


# Main method
def main ():
    val_param = []
    bot_num = 5
    num_sims = 100
    param_max = 0
    

    # Determines the parameter to use for the simulations, either k = dimension/2 or a = i/30 (i = simulation number)
    if bot_num == 1 or bot_num == 2 or bot_num == 5 or bot_num == 6:
        param_max = int(dimension/2)
    else:
        param_max = 30

    # Used for simulations, if we want to simulate bot 7 we have to simulate bot 7, 8, and 9 which is three bots instead of two for the other sims
    n = 2
    if bot_num == 7: n = 3

    # Runs simulations
    for p in range(1, param_max + 1):
        a = p
        # 0 <= a <= 1 for probabalistic bots, otherwise 0 <= k <= dimension/2
        if not (bot_num == 1 or bot_num == 2 or bot_num == 5 or bot_num == 6): a = p/30
        val_param.append(a)

        for i in range(bot_num, bot_num + n):
            success_list[i - bot_num].append(create_simulation(i, num_sims, a))
            print("Bot: ", i)
            print("Parameter = ", a)
            print(success_list)
            print()

    print()
    plot_simulation(val_param, bot_num)



# Helper method to run a single simulation
def simulate():
    param = 3
    bot_num = 6
    ship = create_ship(bot_num, param)
    sim = run_bot_simulation(ship, bot_num, param)
    if sim >= 0:
        print(sim)
    else:
        print("FAIL")

# Run a certain number of simulations for each robot, and count the successes
def create_simulation(bot_num, num_sims, param):

    num_success = 0
    avg_steps = 0

    for i in range(num_sims+1):
        ship = create_ship(bot_num, param)
        p = possible_leak.copy()
        ship_copy = ship.copy()
        steps = run_bot_simulation(ship, bot_num, param)
        print(steps)
        if not steps or steps == 0: 
            print_ship(ship_copy)
        avg_steps += steps

    return avg_steps/num_sims
    
# Plot the simulations of each bot on the same graph
def plot_simulation(val_param, bot_num):

    # Each robot gets plotted
    match bot_num:
        case 1:
            plt.plot(val_param, success_list[0], label = 'Bot One')
            plt.plot(val_param, success_list[1], label = 'Bot Two')
        case 3:
            plt.plot(val_param, success_list[0], label = 'Bot Three')
            plt.plot(val_param, success_list[1], label = 'Bot Four')
        case 5:
            plt.plot(val_param, success_list[0], label = 'Bot Five')
            plt.plot(val_param, success_list[1], label = 'Bot Six')
        case 7:
            plt.plot(val_param, success_list[0], label = 'Bot Seven')
            plt.plot(val_param, success_list[1], label = 'Bot Eight')
            plt.plot(val_param, success_list[2], label = 'Bot Nine')


    plt.plot()

    # Labels for the axis
    plt.xlabel('Value of a / k')
    plt.ylabel('Avg number of moves')

    plt.title('Average number of moves for values of a / k for Bots three and four with Dimension = 30')

    plt.legend()
    plt.show()

    return

# Creates the ship, generating the open and closed cells according to the rule set, and adding the bot, fire, and leak
def create_ship(bot_num, k):
    global fringe_open
    global possible_leak
    global leak_found
    global probability_map
    global ping_map
    global highest_prob

    leak_found = False

    possible_leak = []
    

    ship = [['X' for _ in range(dimension)] for _ in range(dimension)] # 2D array of closed cells, dimension x dimension large

    rand_row = int(random.random() * (dimension))
    rand_col = int(random.random() * (dimension))

    # Fringes to contain the current closed/open cells in the ship while generating
    ship[rand_row][rand_col] = '_'

    # Intialize the fringes for open and closed cells
    fringe_close = []
    fringe_open = [(rand_row, rand_col)]
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

    # Creates the list of possible cells
    possible_leak = fringe_open.copy()

    # If bot is probabilistic, initialize hashmaps
    if bot_num == 3 or bot_num == 4 or bot_num == 7 or bot_num == 8 or bot_num == 9:
        ping_map = dict()
        probability_map = dict()
        for p in possible_leak:
            probability_map.update({p : 1/len(possible_leak)})
        probability_map.update({bot_loc : 0})

    # If bot is deterministic, takes an initial sense to eliminate the surrounding detection area so the leak can be placed outside of it
    if bot_num == 1 or bot_num == 2 or bot_num == 5 or bot_num == 6: bot_box(ship, k, bot_loc, bot_num) # Removes cells
    
    # Add the leak
    c = random.choice(possible_leak)
    ship[c[0]][c[1]] = 'L'
    global leak_loc
    leak_loc = []
    if bot_num == 1 or bot_num == 2 or bot_num == 3 or bot_num == 4: leak_loc = c
    else: leak_loc.append(c)

    # If there is more than one leak for the robot to find, add it to a list instead of a single variable
    if bot_num == 5 or bot_num == 6 or bot_num == 7 or bot_num == 8 or bot_num == 9:
        # Add the leak
        c = random.choice(possible_leak)

        # Ensures leak is not in the same position as the first leak
        while c == leak_loc[0]:
            c = random.choice(possible_leak)
        ship[c[0]][c[1]] = 'L'
        leak_loc.append(c)

    return ship

# Prints ship
def print_ship(ship):
    s = "    "
    count = 0
    for i in range(0, dimension):
        s = s + str(i%10) + "|"
    print(s) # Legend for the columns of a 10x10 ship
    for i in range(dimension):
        print(count % 10, end = "   ")
        for j in range(dimension):
            c = ship[i][j]
            if (i, j) in possible_leak and c != 'L': 
                print('P', end = " ")
            else:
                print(c, end = " ")
        count+=1
        print()
    print()

# Returns the neighboring opened/closed/fire cells to a given cell (leak cell is an open cell)
def find_neighbors(ship, row, col, state):
    cells = []
    directions = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # Up, down, left, and right weights

    for dr, dc in directions:
        r, c = row + dr, col + dc # Row and column
        if 0 <= r < dimension and 0 <= c < dimension:
            if (state == 'closed' and ship[r][c] == 'X') or \
               (state == 'open' and ship[r][c] == '_') or \
               (state == 'search' and (ship[r][c] == '_' or ship[r][c] == 'L')):
                cells.append((r, c))
    
    return cells

# Runs a simulation of a robot depending on the robot number
def run_bot_simulation(ship, bot_num, parameter):
    global leak_found
    global first_leak_found
    global probability_map
    global backtrack_phase

    # Intializing the global variables to be reset
    path = []
    path_copy = []

    backtrack_phase = 0
    curr_bot_loc = bot_loc
    steps = 1

    # Variable that tracks if the robot took the move action, used to alternate between sense and move during each iteration of the while loop
    move_bot = False
    leak_found = False
    first_leak_found = False

    

    # If bot is probabilistic, generate the bfs distances for each pair of cells to be used for ping probabilities, and compute the highest probability in the map
    if bot_num == 3 or bot_num == 4 or bot_num == 7 or bot_num == 8 or bot_num == 9: 
        for p in possible_leak:
            generate_bfs_distances(ship, p)
            compute_highest_prob()

    # Run simulation while path is not empty  
    while True:
        
        # Deterministic bots
        if bot_num == 1 or bot_num == 2 or bot_num == 5 or bot_num == 6: 
            
            if leak_found:

                # Uses a backtracking algorithm for when the leak is found to "peel back" the layers of the detection square
                # In theory, once the leak is found, if we backtrack the robot from the most recent path it took to find the leak,
                # then at each step in the reverse step it will reveal more and more about which cells possibly have the cell
                # This is much faster than bot one's approach of just randomly searching the detection square
                if (bot_num == 2 or (bot_num == 6 and first_leak_found)) and len(box_search) > len(path_copy) and backtrack_phase != 2:
                    # Second phase of backtracking:
                    # Moves back each step in the reverse path, until either the leak is out of view or the bot goes back to where it was
                    # Goes to the next phase
                    if backtrack_phase == 1:
                        if move_bot:
                            bot_box(ship, parameter, curr_bot_loc, bot_num) # Sense
                        elif not move_bot:
                            if not path: backtrack_phase = 2
                        move_bot = not move_bot
                    
                    # Final phase of the backtracking:
                    # Goes back to the normal path-finding, and exits this main section of code
                    if backtrack_phase == 2:
                        if not path: path = bfs(ship, curr_bot_loc, bot_num, parameter)
                        move_bot = True

                    # First phase of the backtracking:
                    # Reverses the path that was just taken to find the leak
                    # Moves to the next phase
                    if backtrack_phase == 0:
                        backtrack_phase = 1
                        path = path_copy.copy()[1:]
                        move_bot = True
                    
                else:
                    if not path: path = bfs(ship, curr_bot_loc, bot_num, parameter)
                    move_bot = True

            else:
                # If the robot has just moved in the previous step and is at the end of its path, take the sense action
                if move_bot and not path:
                    bot_box(ship, parameter, curr_bot_loc, bot_num) # Sense
                    move_bot = False

                # Otherwise, if the robot is in a cell that is not in the cells possibly containing the leak, calculate a new path
                elif curr_bot_loc not in possible_leak:
                    if not path: 
                        path = bfs(ship, curr_bot_loc, bot_num, parameter)

                        # Stores the reverse of the current path to be used for backtracking in bot 2
                        if path and (bot_num == 2 or bot_num == 6):
                            path_copy = path.copy()
                            path_copy.reverse()

                    move_bot = True
        
        # Probabilistic Bots
        if bot_num == 3 or bot_num == 4 or bot_num == 7 or bot_num == 8 or bot_num == 9:

            # If bot has just moved and at the end of its path, take the sense action and update probabilties accordingly
            if move_bot and not path:    
                prob = 0

                # For bot 4 and 9, we stay in the cell for an arbitrary number of steps for a more accurate beep probability (3 steps for simulations was reasonable)
                if bot_num == 4 or bot_num == 9:
                    max_range = 3
                    for i in range(0, max_range + 1):
                        if received_ping(ship, parameter, bot_num, curr_bot_loc): 
                            prob = 1
                            break
                        steps+=1
                else:
                    if received_ping(ship, parameter, bot_num, curr_bot_loc): prob = 1


                if prob == 0:
                    # If there are two leaks and the first one was not found yet, we must update probabilities accordingly to having two leaks
                    if (bot_num == 8 or bot_num == 9) and not first_leak_found:
                        no_beep_prob_two_leaks(ship, curr_bot_loc, parameter)
                    else:
                        no_beep_prob(ship, curr_bot_loc, parameter)
                else:
                    if (bot_num == 8 or bot_num == 9) and not first_leak_found:
                        beep_prob_two_leaks(ship, curr_bot_loc, parameter)
                    else:
                        beep_prob(ship, curr_bot_loc, parameter)
                
                # Normalize the probabilities and then calculate the highest probability
                normalize()
                compute_highest_prob()

                move_bot = False
            else:
                #Move the bot
                if not path: 
                    path = bfs(ship, curr_bot_loc, bot_num, parameter)
                move_bot = True

        # Executes if a path was calculated and we want to move the bot
        if move_bot:

            if not path: return False # No path was found, search was failure
            
            step = path.pop(0)

            # Bots that are searching for two leaks
            if bot_num == 5 or bot_num == 6 or bot_num == 7 or bot_num == 8 or bot_num == 9:
                if step in leak_loc:
                    # If the first leak was not found, update variables, otherwise we found both leaks
                    if not first_leak_found: 
                        first_leak_found = True
                        leak_loc.remove(step)
                        leak_found = False
                        path = []
                    else:
                        return steps
            # Bots that are searching for one leak
            elif bot_num == 1 or bot_num == 2 or bot_num == 3 or bot_num == 4:
                if step == leak_loc: return steps

            # Updates the R token in the 2D ship array, for visualization purpose
            ship[curr_bot_loc[0]][curr_bot_loc[1]] = '_'
            ship[step[0]][step[1]] = 'R'
            curr_bot_loc = step

            # Updates possible leak cells, as well as the probability map to compensate for robot entering a cell without a leak
            if step in possible_leak: 
                possible_leak.remove(step)
                if bot_num == 3 or bot_num == 4 or bot_num == 7 or bot_num == 8 or bot_num == 9: 
                    probability_map.update({curr_bot_loc : 0})
                    enter_cell_prob(curr_bot_loc)
                    normalize()
                    compute_highest_prob()

        steps+=1

        # Edge case for if the leak is the only remaining possible cell
        if len(possible_leak) == 1:
            steps += len(normal_bfs(ship, curr_bot_loc, possible_leak[0]))
            return steps

        # print("Possible Cells: ", possible_leak)
        # print("Path: ", path)
        # print_ship(ship)


    return False

# BFS Search for the ship, varies the conditions based on if we check for adjacent fire cells
def bfs (ship, start, bot_num, k):
    fringe = [(start, [start])]
    visited = []
    copy_path = []
    # Uninformed BFS Search for the ship, using neighbors of a node as children nodes
    # Keeps track of the coordinates in the ship that correspond to the shortest path after algo is run
    while fringe:
        curr, path = fringe.pop(0)
        # If determinstic bot, and the first leak was found, we search only at the cells found in the box search sense
        if (bot_num == 5 or bot_num == 6) and not first_leak_found and leak_found and box_search and curr in box_search and curr in possible_leak:
            path.pop(0)
            return path
        # If probabilistic bot, check if the cell has the highest probability
        elif bot_num == 3 or bot_num == 4 or bot_num == 7 or bot_num == 8 or bot_num == 9:
            if probability_map.get(curr) >= highest_prob and curr != start:
                path.pop(0)
                return path
        # Otherwise, just check if the cell belongs to possible leak cells
        elif possible_leak and curr in possible_leak:
            path.pop(0)
            return path

        neighbors = find_neighbors(ship, curr[0], curr[1], 'search')
        # Shuffles the neighbors for deterministic bots because ties are meant to broken randomly
        if bot_num != 3 or bot_num != 4 or bot_num != 7 or bot_num != 8 or bot_num != 9: random.shuffle(neighbors)
        
        for n in neighbors:
            if n not in visited:
                fringe.append((n, path + [n])) 
                visited.append(n)

    return

# A normal BFS search for a given start and goal cell, used for the edge case of having one probable cell left
def normal_bfs(ship, start, goal):
    fringe = [(start, [start])]
    visited = []

    # Uninformed BFS Search for the ship, using neighbors of a node as children nodes
    # Keeps track of the coordinates in the ship that correspond to the shortest path after algo is run
    while fringe:
        curr, path = fringe.pop(0)
        
        if curr == goal:
            path.pop(0)
            return path

        neighbors = find_neighbors(ship, curr[0], curr[1], 'search')
        
        for n in neighbors:
            if n not in visited:
                fringe.append((n, path + [n])) 
                visited.append(n)

    return

# Sense action for the deterministic bots
def bot_box(ship, k, curr_bot_loc, bot_num):
    global possible_leak
    global leak_found
    global box_search    

    # Initializing variables
    box_search = []
    path = []
    found_leak_now = False

    # Defines the bounds for the box search area as to not have index out of bounds
    curr_row, curr_col = curr_bot_loc
    row_range = range(max(curr_row - k, 0), min(curr_row + k + 1, dimension))
    col_range = range(max(curr_col - k, 0), min(curr_col + k + 1, dimension))

    # Searches through the given sense area, marking if the leak was found at any point
    for r in row_range:
        for c in col_range:
            if is_cell_valid(ship, r, c):
                box_search.append((r, c))
                if ship[r][c] == 'L': 
                    leak_found = True
                    found_leak_now = True
    # It is possible that the leak variable is already set to true before the search, if we are bot 2 and doing backtracking.
    # This ensures that the sense action always configures leak_found regardless if its true or false at the beginning
    if leak_found != found_leak_now:
        leak_found = False

    # If the leak was not found, eliminate the cells in the search area as possibly having the leak
    if not leak_found:
        for cell in box_search:
            if cell in possible_leak: possible_leak.remove(cell)

    # Otherwise, if the leak was found, or if the first leak was already found for bots 5 and 6, remove all cells outside the search area that were possible
    elif leak_found:
        if (bot_num == 1 or bot_num == 2) or ((bot_num == 5 or bot_num == 6) and first_leak_found):
            new_possible = []
            for c in possible_leak:
                if c in box_search and c != curr_bot_loc:
                    new_possible.append(c)
            possible_leak = new_possible.copy()

    return

# Adjusts the probability of cells having the leak based on if we recieved a beep in the current cell
def beep_prob(ship, curr_bot_loc, a):
    global probability_map
    probability_map_copy = probability_map.copy()
    beep_prob = 0.0
    
    # Calculates P(Heard beep in curr_bot_loc)
    for k in possible_leak:
        if curr_bot_loc != k: beep_prob += probability_map.get(k) * ping_prob(a, distances_from_goal.get((curr_bot_loc, k)))

    for p in possible_leak:

        #   Probability Update Calculations:
        #
        #   P(Leak in p | heard beep in bot_loc)
        # 
        # = P(Leak in p) * P(heard beep in bot_loc | leak in p) / P(heard beep in bot_loc)
        # 
        # = P(Leak in P) * e^-a(distance to p - 1) / P(heard beep in bot_loc)
        # 
        # = P(Leak in P) * e^-a(distance to p - 1) /  For each cell K : P( leak in k ) * exp( -a * ( d(i,k) - 1 ) )

        new_prob = (probability_map.get(p) * ping_prob(a, distances_from_goal.get((curr_bot_loc, p)))) / beep_prob
        probability_map_copy.update({p : new_prob})

    probability_map = probability_map_copy.copy()

# Adjusts the probability of cells having the leak based on if we recieved no beep in the current cell
def no_beep_prob(ship, curr_bot_loc, a):
    global probability_map
    probability_map_copy = probability_map.copy()
    beep_prob = 0.0
    
    # Calculates P(Heard beep in curr_bot_loc)
    for k in possible_leak:
        if curr_bot_loc != k: beep_prob += probability_map.get(k) * ping_prob(a, distances_from_goal.get((curr_bot_loc, k)))

    for p in possible_leak:

        #   Probability Update Calculations:
        #
        #   P(Leak in p | heard no beep in bot_loc) =
        # 
        # = P(Leak in p) * P(heard no beep in bot_loc | leak in p) / P(heard no beep in bot_loc)
        # 
        # = P(Leak in p) * 1 - P(heard beep in bot_loc | leak in p) / P(heard no beep in bot_loc)
        # 
        # = P(Leak in P) * 1 - e^-a(distance to p - 1) / 1 - P(heard beep in bot_loc)
        #
        # = P(Leak in P) * e^-a(distance to p - 1) /  For each cell K : 1 - (P( leak in k ) * exp( -a * ( d(i,k) - 1 ) ))

        new_prob = (probability_map.get(p) * (1 - ping_prob(a, distances_from_goal.get((curr_bot_loc, p))))) / (1 - beep_prob)
        probability_map_copy.update({p : new_prob})

    probability_map = probability_map_copy.copy()
    
# Adjusts the probability of cells having the leak based on if we recieved a beep in the current cell given we also have two leaks
def beep_prob_two_leaks(ship, curr_bot_loc, a):
    global probability_map

    # Stores the old probability map so the probability updates are always calculating on the old values, turning into new values
    probability_map_copy = probability_map.copy()
    beep_prob = 0.0

    # Calculates P(Heard beep in curr_bot_loc) for every pair of cells i and j
    for i in possible_leak:
        beep_sum = 0.0
        for j in possible_leak:
            if curr_bot_loc != i and curr_bot_loc != j and i != j: 
                prob_i = probability_map.get(i)
                prob_j = probability_map.get(j)
                prob_ping = ping_prob(a, distances_from_goal.get((i, curr_bot_loc))) + ping_prob(a, distances_from_goal.get((j, curr_bot_loc)))
                beep_prob += prob_i * prob_j * prob_ping
                

    # Iterates through every pair of cells
    for i in range(0, len(possible_leak)):
        cell_i = possible_leak[i]

        # Stores the probabilities of each cell pairing at every iteration of outer loop
        temp_probs = []

        for j in range(0, len(possible_leak)):
            if i != j:
                
                cell_j = possible_leak[j]

                #   Probability Update Calculations:
                #
                #   P(Leak in i and leak in j | heard beep in curr_bot_loc)
                #
                # = P(Leak in i and leak in j) * P(heard beep in bot_loc | leak in i and leak in j) / P(heard beep in bot_loc)
                #
                # = P(Leak in i) * P(leak in j) * (1 - (1 - P( k does cause a beep in j | leak in cell j )) * (1 - P( k does cause a beep in i | leak in cell k )) / P(beep in loc)
                #
                # = P(Leak in i) * P(leak in j) * (1 - (1 - e^-a(distance to j - 1)) * (1 - e^-a(distance to i - 1)) / P(beep in loc)
                #
                # = P(Leak in i) * P(leak in j) * (1 - (1 - e^-a(distance to j - 1)) * (1 - e^-a(distance to i - 1)) / For each cell a and b: P(leak in a) * P(leak in b) * (e^-a(distance to a - 1) + e^-a(distance to b - 1))
                #
                #   P (Leak in i and Leak in SOME j) = P (Leak in i and (leak in first cell of possible leak cells or leak in second cell of possible leak cells or ...))
                #
                #   This compounded probability is calculated at the end of every inner loop for the initial pairs of cells
                #

                p_leak_in_i = probability_map.get(cell_i)

                p_leak_in_j = probability_map.get(cell_j)

                p_beep_given_i = ping_prob(a, distances_from_goal.get((curr_bot_loc, cell_i)))
                p_beep_given_j = ping_prob(a, distances_from_goal.get((curr_bot_loc, cell_j)))

                new_prob = (p_leak_in_i * p_leak_in_j * (1 - ((1 - p_beep_given_j) * (1 - p_beep_given_i)))) / beep_prob

                # The list of probabilities to be added at the end of the loop for the OR calculation of probability
                temp_probs.append(new_prob)

        # Updates cell i probability given the probabilities of each cell pairing found in the inner loop
        probability_map_copy.update({cell_i : sum(temp_probs)})
    
    # Updates the probability map with the new one
    probability_map = probability_map_copy.copy()

# Adjusts the probability of cells having the leak based on if we recieved no beep in the current cell given that we also have two leaks
def no_beep_prob_two_leaks(ship, curr_bot_loc, a):
    global probability_map

    # Stores the old probability map so the probability updates are always calculating on the old values, turning into new values
    probability_map_copy = probability_map.copy()
    beep_prob = 0.0

    # Calculates P(Heard beep in curr_bot_loc) for every pair of cells i and j
    for i in possible_leak:
        for j in possible_leak:
            if curr_bot_loc != i and curr_bot_loc != j and i != j: 
                prob_i = probability_map.get(i)
                prob_j = probability_map.get(j)
                prob_ping = ping_prob(a, distances_from_goal.get((i, curr_bot_loc))) + ping_prob(a, distances_from_goal.get((j, curr_bot_loc)))
                beep_prob += prob_i * prob_j * prob_ping
                

    # Iterates through every pair of cells
    for i in range(0, len(possible_leak)):
        cell_i = possible_leak[i]
        temp_probs = []
        for j in range(0, len(possible_leak)):
            if i != j:

                cell_j = possible_leak[j]

                #   Probability Update Calculations:
                #
                #   P(Leak in i and leak in j | heard no beep in curr_bot_loc)
                #
                # = P(Leak in i and leak in j) * P(heard no beep in bot_loc | leak in i and leak in j) / P(heard no beep in bot_loc)
                #
                # = P(Leak in i) * P(leak in j) * (1 - P( j does cause a beep in i | leak in cell j )) * (1 - P( k does cause a beep in i | leak in cell k ) / 1 - P(beep in loc)
                #
                # = P(Leak in i) * P(leak in j) * (1 - e^-a(distance to j - 1)) * (1 - e^-a(distance to i - 1)) / 1 - (For each cell a and b: P(leak in a) * P(leak in b) * (e^-a(distance to a - 1) + e^-a(distance to b - 1)))
                #
                #   Add this probability to a list of temp probabilities
                #
                #   P (Leak in i and Leak in SOME j) = P (Leak in i and (leak in first cell of possible leak cells or leak in second cell of possible leak cells or ...))
                #
                #   This compounded probability is calculated at the end of every inner loop for the initial pairs of cells
                
                p_leak_in_i = probability_map.get(cell_i)
                p_leak_in_j = probability_map.get(cell_j)

                p_beep_given_i = ping_prob(a, distances_from_goal.get((curr_bot_loc, cell_i)))
                p_beep_given_j = ping_prob(a, distances_from_goal.get((curr_bot_loc, cell_j)))

                new_prob = (p_leak_in_i * p_leak_in_j * ((1 - p_beep_given_j) * (1 - p_beep_given_i))) / (1 - beep_prob)
                
                # The list of probabilities to be added at the end of the loop for the OR calculation of probability
                temp_probs.append(new_prob)
        
        # Updates cell i probability given the probabilities of each cell pairing found in the inner loop
        probability_map_copy.update({cell_i : sum(temp_probs)})

    # Updates the probability map with the new one
    probability_map = probability_map_copy.copy()
    
# Adjusts the probabilities given the entered cell did not contain a leak
def enter_cell_prob(curr_bot_loc):
    global highest_prob
    global probability_map

    for p in possible_leak:
        beep_prob = ping_map.get(curr_bot_loc)

        #   P(Leak in p | leak not found in bot_loc) =
        # 
        # = P(Leak in p) * P(leak not found in bot_loc | leak in p) / P(leak not found in bot_loc)
        # 
        # = P(Leak in P) * 1 - (1 / number of possible locations) <- Will normalize the probabilties to add up to 1 again
        
        new_prob = probability_map.get(p) * (1 - 1/len(possible_leak))
        probability_map.update({p : new_prob})

    return

# Finds the highest probability in probability_map
def compute_highest_prob():
    global highest_prob

    highest_prob = probability_map.get(max(probability_map, key=probability_map.get))

# Calculates the probability of sending out the ping
def received_ping(ship, a, bot_num, curr_bot_loc):
    global first_leak_found

    # Distance and probability
    d = 0.0
    prob = 0.0

    # If there are two leaks, find the probabilities of either one sending out a ping by adding the probabilities of each respective ping
    if not first_leak_found and (bot_num == 7 or bot_num == 8 or bot_num == 9):
        d1 = distances_from_goal.get((curr_bot_loc, leak_loc[0]))
        d2 = distances_from_goal.get((curr_bot_loc, leak_loc[1]))

        prob = ping_prob(a, d1) + ping_prob(a, d2)

    # Otherwise, only calculate one probability
    else:
        if bot_num == 7 or bot_num == 8 or bot_num == 9: loc = leak_loc[0]
        else: loc = leak_loc

        d = distances_from_goal.get((curr_bot_loc, loc))
        prob = ping_prob(a, d)

    # Returns true if probability hits
    if prob >= random.random():
        return True
    return False

# Normalizes the probability_map so that all the probabilities add up to 1
def normalize():
    s = sum(probability_map.values())
    if s != 0:
        factor = 1 / sum(probability_map.values())
        for p in possible_leak:
            prob = probability_map.get(p)
            probability_map.update({p : factor * prob})

# Generates the BFS distances between all given cells until fringe is empty
def generate_bfs_distances(ship, goal):
    global distances_from_goal
    ship_copy = ship.copy()

    ship_copy[bot_loc[0]][bot_loc[1]] = '_' 

    # Setting up the fringe and visited structures for bfs
    fringe = deque([(goal, 0)]) 
    visited = set([goal])

    while fringe:
        current, distance = fringe.popleft()
        if current != goal: distances_from_goal.update({(current, goal) : distance})

        neighbors = find_neighbors(ship_copy, current[0], current[1], 'search')
        for n in neighbors:
            if n not in visited:
                visited.add(n)
                fringe.append((n, distance + 1))

# The probability formula for sending out a ping
def ping_prob(a, d):
    return pow(math.e, (-1 * a) * (d - 1))

# Checks if a given cell is valid (used for determinstic sense)
def is_cell_valid(ship, i, j):
    return ship[i][j] != 'X' and (i, j) in possible_leak

main()
