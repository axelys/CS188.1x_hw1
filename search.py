# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and Pieter 
# Abbeel in Spring 2013.
# For more info, see http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    start = problem.getStartState();

    stack = [ start ]
    expanded = []

    # ???? ????? ????????? ???? ? ?????????
    # ?????????? ??????? expand
    
    node_list = { start: { 'action': None, 'cost': None, 'parent' : None } };

    while len(stack) > 0 :

        # ????? ??????? ?? ?????
        
        current = stack.pop()

        # ????????? ???? ??? ?????????????
        # ???? ??? ???? - ????????? ????
        
        if problem.isGoalState(current) :
            expanded.append( current )
            return getActions(current, node_list)
        
        # ???? ??? -
        # ????????? ??????????? ?? ??? ???? ??????
        # ???? ??? - ??????????? ???? current ? ?????? ????? ? ????
        #elif current not in visited :
        else :
            
            for node in problem.getSuccessors( current ) :
                state, action, cost = node
                if (state not in expanded) :
                    
                    expanded.append( current )
                    node_list[state] = { 'action':action, 'cost':cost, 'parent' : current }
                    stack.append( state )


def getActions(state, nodes):

    result = []

    while (nodes[state]['parent'] != None) :
        result.append(nodes[state]['action'])
        state=nodes[state]['parent']

    result.reverse()

    return result


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"

    start = problem.getStartState()
    node_list = { start: { 'action': None, 'cost': None, 'parent' : None } };
    closed = [] ;
    stack = [ start ]

    while len(stack) > 0:

        current = stack.pop()
        
        if problem.isGoalState(current):
            return getActions(current, node_list)
        
        if current not in closed:
            closed.append(current)

            for node in problem.getSuccessors(current):

                state, action, cost = node;
                if state not in node_list:
                    node_list[state] = { 'action':action, 'cost':cost, 'parent' : current }
                
                stack.insert(0, state )
                

def uniformCostSearch(problem):
    
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"

    start = problem.getStartState()
    queue = util.PriorityQueue()

    queue.push( (start, [], 0) , 0 )

    node_list = { start: { 'action': None, 'cost': None, 'parent' : None } };
    
    while not queue.isEmpty():
        
        current_node, current_path, current_cost = queue.pop()
        if problem.isGoalState(current_node):
            return current_path
        else:
            for expanded in problem.getSuccessors(current_node):

                node, action, cost = expanded
                item = (node, current_path + [action], current_cost + cost)
                if node in node_list:
                    if node_list[node]['cost'] > current_cost + cost:
                        queue.push(item, current_cost + cost)
                        node_list[node] = { 'action':current_path + [action], 'cost':cost, 'parent' : node }
                else:
                    queue.push(item, current_cost +cost)
                    node_list[node] = { 'action':current_path + [action], 'cost':cost, 'parent' : node }
                    

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def cost_plus_heuristic(item):
    return item[2] + item[3]

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"

    startNode = problem.getStartState()
    queue = util.PriorityQueueWithFunction(cost_plus_heuristic)
    start = (startNode, [], 0, heuristic(startNode, problem))
    
    queue.push(start)

    visited= {startNode: cost_plus_heuristic(start)}
    
    expanded_nodes = {}

    while not queue.isEmpty():

        current_item = queue.pop()
        current_node, currentPath, currentCost, currentHeuristic = current_item

        if current_node not in expanded_nodes.keys():

            expanded_nodes[current_node] = cost_plus_heuristic(current_item)
            
            if problem.isGoalState(current_node):
                return currentPath
            else:
                for tmp in problem.getSuccessors(current_node):

                    node, action, cost = tmp

                    item = (node, currentPath + [action], currentCost + cost, heuristic(node, problem))
                    
                    if node in visited.keys():
                        if visited[node] > cost_plus_heuristic(item) :
                            queue.push(item)
                            visited[node] = cost_plus_heuristic(item)
                    else:
                        queue.push(item)
                        visited[node] = cost_plus_heuristic(item)

    

def print_r(data):

    print "----------------------------"
    print data
    print "----------------------------"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch





