#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the LunarLockout  domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#import os for time functions
from search import * #for search engines
from lunarlockout import LunarLockoutState, Direction, lockout_goal_state #for LunarLockout specific classes and problems
import math

#LunarLockout HEURISTICS
def heur_trivial(state):
    '''trivial admissible LunarLockout heuristic'''
    '''INPUT: a LunarLockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
    return 0

def heur_L_distance(state):
    #IMPLEMENT
    '''L distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses mahnattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the L distances between each xanadu and the escape hatch.

    width = (state.width - 1)/2
    totalL = 0
    x = state.xanadus
    if type(x[0]) is int:
      return (abs(x[0]-width)+abs(x[1]-width))
    elif type(x[0]) is tuple:
      for xanadu in x:
        totalL = totalL + (abs(xanadu[0]-width)+abs(xanadu[1]-width))
      return totalL

    return 0

def heur_alternate(state):
#IMPLEMENT
    '''a better lunar lockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #Your function should return a numeric value for the estimate of the distance to the goal.

    x = state.xanadus
    r = state.robots
    width = (state.width-1)/2
    height = state.width

    board = [0] * (height*height)                #board lookup for occupied spaces
    for robot in r:
      board[(robot[0]*height)+robot[1]-1]=1
    if type(x[0])is int:
      board[(x[0] * height) + x[1]] = 1
    else:
      for xanadu in x:
       board[(xanadu[0]*height)+xanadu[1]]=1

    greatest_r_x = 0                             #subset of board that contains range of
    greatest_r_y = 0                             #locations of robots
    smallest_r_x = height - 1
    smallest_r_y = height - 1
    for robot in r:
      if robot[0] > greatest_r_x:
        greatest_r_x = robot[0]
      if robot[0] < smallest_r_x:
        smallest_r_x = robot[0]
      if robot[1] > greatest_r_y:
        greatest_r_y = robot[1]
      if robot[1] < smallest_r_y:
        smallest_r_y = robot[1]

    if type(x[0]) is int:                       #if only 1 xanadu
      if x[0] is width and x[1] is width:       #if xanadu is at goal return 0
        return 0

      if (x[0] is width or x[1] is width) and board[board_center] is 0: #xanadu 1 step from goal return 1
        if x[0] is width and x[1]<width and board[board_center-height] is 0 and board[board_center+height] is 1 and board[board_center-(height*2)] is 0:
          return 1
        if x[0] is width and x[1]>width and board[board_center+height] is 0 and board[board_center-height] is 1 and board[board_center+(height*2)] is 0:
          return 1
        if x[0]<width and x[1] is width and board[board_center-1] is 0 and board[board_center+1] is 1 and board[board_center-2] is 0:
          return 1
        if x[0]>width and x[1] is width and board[board_center+1] is 0 and board[board_center-1] is 1 and board[board_center+2] is 0:
          return 1

      #if xanadu is outside the subset rectangle of occupied spaces return ~infinity (999)
      if (x[0] > greatest_r_x and x[1] > greatest_r_y) or (x[0] > greatest_r_x and x[1] < smallest_r_y ) or\
        (x[0] < smallest_r_x and x[1] < smallest_r_y) or (x[0] < smallest_r_x and x[1] > greatest_r_y):
        return 999

      return heur_L_distance(state)+1

    else: #more than 1 xanadu

      for xanadu in x:                    #add the xanadus to the subset rectangle
        if xanadu[0] > greatest_r_x:      #of occupied spaces
          greatest_r_x = robot[0]
        if xanadu[0] < smallest_r_x:
          smallest_r_x = robot[0]
        if xanadu[1] > greatest_r_y:
          greatest_r_y = robot[1]
        if xanadu[1] < smallest_r_y:
          smallest_r_y = robot[1]

      for xanadu in x:               #if any xanadu is outside subset bound return ~infinity (999)
        if (xanadu[0] > greatest_r_x and xanadu[1] > greatest_r_y) or (
                xanadu[0] > greatest_r_x and xanadu[1] < smallest_r_y) or \
                (xanadu[0] < smallest_r_x and xanadu[1] < smallest_r_y) or (
                xanadu[0] < smallest_r_x and xanadu[1] > greatest_r_y):
          return 999

      lonely_x_count = 0


      return heur_L_distance(state) + lonely_x_count


def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + (weight * sN.hval)
    # return 0

def anytime_weighted_astar(initial_state, heur_fn, weight=4., timebound = 1):
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''

  #search is initialized and if a goal state is found, use the remainder of the time bound to
  #reduce the weight by half and initialize a new search with the new weight and a
  #costbound of the previous gval - 1

  timebound = 8
  weight = 8;
  se = SearchEngine('custom', 'full')
  se.init_search(initial_state, lockout_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
  start_time = os.times()[0]
  goal = se.search(timebound)  # Goal state
  if goal:
    # print("initial cost " + str(goal.gval)+" found in "+str(os.times()[0]-start_time)+"s")
    costbound = goal.gval
    remaining_time = timebound - (os.times()[0]-start_time)
    while remaining_time > 0 and not se.open.empty():
      temp_start_time = os.times()[0]
      weight = weight/2
      se.init_search(initial_state, lockout_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
      temp = se.search(remaining_time, costbound = (costbound-1, float("inf"), float("inf")))
      remaining_time = remaining_time - (os.times()[0] - temp_start_time)
      if temp:
        # print("better solution found " + str(temp.gval))
        costbound = temp.gval
        goal = temp
    return goal
  else:
    return False

def anytime_gbfs(initial_state, heur_fn, timebound = 8):
  #IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''

  #search is initialized and if a goal state is found, use the remainder of the time bound to
  #search for other goals with reduced costs by introducing a costbound of the previous gval -1

  timebound = 10
  se = SearchEngine('best_first', 'full')
  se.init_search(initial_state, lockout_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
  start_time = os.times()[0]
  goal = se.search(timebound)  # Goal state
  if goal:
    # print("initial cost " + str(goal.gval)+" found in "+str(os.times()[0]-start_time)+"s")
    costbound = goal.gval
    remaining_time = timebound - (os.times()[0]-start_time)
    while remaining_time > 0 and not se.open.empty():
      temp_start_time = os.times()[0]
      temp = se.search(remaining_time, costbound = (costbound-1, float("inf"), float("inf")))
      remaining_time = remaining_time - (os.times()[0] - temp_start_time)
      if temp:
        # print("better solution found " + str(temp.gval))
        costbound = temp.gval
        goal = temp
    return goal
  else:
    return False

PROBLEMS = (
  #5x5 boards: all are solveable
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 1))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 2))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 3))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 1))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 2))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 3))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 4))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 0))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 1))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (0, 2),(0,4),(2,0),(4,0)),((4, 4))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 0))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 1))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 3))),
  #7x7 BOARDS: all are solveable
  LunarLockoutState("START", 0, None, 7, ((4, 2), (1, 3), (6,3), (5,4)), ((6, 2))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (4, 2), (2,6)), ((4, 6))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (3, 1), (4, 1), (2,6), (4,6)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((1, 2), (0 ,2), (2 ,3), (4, 4), (2, 5)), ((2, 4),(3, 1),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 2), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 1), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (0 ,2), (1 ,2), (6, 4), (2, 5)), ((2, 0),(3, 0),(4, 0))),
  )

if __name__ == "__main__":

  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 1; #1 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(len(PROBLEMS)): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    print("*******RUNNING A STAR*******")
    se = SearchEngine('astar', 'full')
    se.init_search(s0, lockout_goal_state, heur_L_distance)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0;
  print("Running Anytime Weighted A-star")

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]
    print("*******RUNNING WEIGHTED A STAR*******")
    weight = 4
    final = anytime_weighted_astar(s0, heur_L_distance, weight, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime GBFS")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]
    print("*******RUNNING GBFS*******")
    final = anytime_gbfs(s0, heur_L_distance, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************")   



  

