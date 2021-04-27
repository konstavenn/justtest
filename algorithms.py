
import random
from math import inf
from action import Action

# State sequences to be used with local search algorithms
# A successor of a state sequence is obtained by adding
# one of the successors of the last state in the sequence.
# Additionally, we consider successors that drop a sequence
# of states from the end of the current sequence.

class StateSequence():

    def __init__(self,laststate,sequence = []):
        self.lastState = laststate
        self.sequence = sequence

    def successors(self):
        # All possible next states for the last state in the sequence
        ss0 = self.lastState.successors()
        # Eliminate those states that are already in the sequence
        currentstates = [ s for a,s in self.sequence ]
        ss = [ (a,s) for (a,s) in ss0 if s not in currentstates ]
        # Extend current sequence with the new states
        forward = [ (a,StateSequence(s,self.sequence + [(a,s)])) for a,s in ss ]
        if len(self.sequence) == 0:
            return forward
        # For non-empty sequences allow dropping suffixes
        backward = []
        for i in range(2,len(self.sequence),1 + len(self.sequence) // 5):
            laststate = self.sequence[i][1]
            prefix = StateSequence(laststate,self.sequence[0:i+1])
            backward = backward + [ (Action(0,0,0),prefix) ]
        return backward + forward

    def seqlen(self):
        return len(self.sequence)

    def __str__(self):
        return str(self.lastState)

# Completely blind random walking

def randomwalk(s0,goaltest):
    s = s0
    iterations = 0
    while not goaltest(s):
        iterations = iterations + 1
        if iterations > 2000:
            print("TIMEOUT")
            return None
        ss = s.successors()
        if len(ss) == 0:
            s = s0
        else:
            choice = random.choices(ss)
            a,s2 = choice[0]
            s=s2
    print("SOLVED (" + str(iterations) + " iterations)")
    return s

# Wrapper for the random walking, with an interface that is
# compatible with the interfaces of the A* and IDA*
# implementations.

def randomwalking(s0,goaltest,h):
    ss0 = StateSequence(s0)
    result = randomwalk(ss0,lambda ss: goaltest(ss.lastState))
    if result == None:
        return None
    return [ a for a,s in result.sequence ]


### Hill-climbing local search
#
# A basic stochastic search algorithm that tends to
# hill-climb towards better and better states, but
# can also do moves other than the seemingly best
# ones.
# This function can be called both with the exactly
# the same arguments as the A* and IDA* implementations,
# as# is uses the functions goaltext and h exactly
# the same way. However, the calls in "hillclimbing"
# below will call with StateSequence objects and
# goaltest and h that work with StateSequence.

def hillclimb(s0,goaltest,h):
    s = s0
    iterations = 0
    while not goaltest(s):
        if iterations > 100000:
            print("TIMEOUT")
            return None
        iterations = iterations + 1
        s_successors = s.successors()
        w_list = []
        for state in s_successors:
            h_val = h(s)
            if h_val != 0:
                w = 100/(h_val*h_val)
                w_list.append(w)
            else:
                w_list.append(h_val)
        rand = random.choices(s_successors,w_list)
        a,s = rand[0]
### IMPLEMENT CODE HERE
###
### 1. Get successor states s.successors().
### 2. Before randomly choosing one of the successor states
###    with e.g. random.choices, you could construct a weight
###    list for random.choices that prioritizes the lowest
###    h-values much stronger than others (e.g. weight 100 for
###    lowest f-value states, and for others some far lower weight
###    depending on how bad that f-value is.)
### 3. Then just assign that randomly chosen state to 's' and
###    continue with the search.
### You may want to experiment a little bit with the weights
### for choosing successor states. Do not expect to solve all
### of the test problems with this algorithm, but many of them
### are. The quality of the solutions is bad, though.
    print("SOLVED (" + str(iterations) + " iterations)")
    return s

# Wrapper for the Hill-Climbing algorithm, with an interface
# that is compatible with the interfaces of the A* and IDA*
# implementations.

def hillclimbing(s0,goaltest,h):
    ss0 = StateSequence(s0)
    result = hillclimb(ss0,lambda ss: goaltest(ss.lastState),lambda ss: h(ss.lastState))
    if result == None:
        return None
    return [ a for a,s in result.sequence ]


### Iterative Deepening A* -- IDA*

totalcalls = 0
#optmPath = []

        
def IDAstar(s0,goaltest,h):
    global totalcalls
    #global optmPath
    bound = h(s0)
    totalcalls = 0
    #optmPath = []
    print(s0)
    while True:
        print("Calling doDFS with bound " + str(bound))
        t = doDFS(s0,0,bound,goaltest,h,[s0])
        if t==None:
            print("TIMEOUT")
            return None
        if isinstance(t,list):
            print("SOLVED (" + str(totalcalls) + " recursive calls)")
            # Print returned list element by element
            #for action in t:
            #    print("Part of the solution path: " + str(action))
            return t
        if t == float('inf'):
            print("NOT SOLVABLE")
            return None
        bound = t

# The main subprocedure of IDA*, performing DFS up to a given
# bound.
# doDFS returns
#   None                if spent too much time
#   a list of actions   if a shortest path to goal states was found
#   a number            for the next bound otherwise
#
# The arguments are
#    s        The state to search
#    g        The g-value of 's'
#    bound    The threshold to cut off the DFS search
#    goaltest Function to test if 's' is a goal state
#    h        The function for computing the h-value of 's'
#    path     List of all states encountered in the current
#             branch. Test successors s2 of s with "s2 in path"
#             to see if the path would have a cycle (and then
#             ignore that kind of s2.)


def doDFS(s,g,bound,goaltest,h,path):
    #global optmPath
    global totalcalls
    totalcalls = totalcalls + 1
    if totalcalls > 10000000:
        return None
    f = g + h(s)
    if f > bound:
        return f
    if goaltest(s):
        optmPath = []
        for i in range(len(path) - 1):
            for a,s in path[i].successors():
                if s == path[i+1]:
                    optmPath.append(a)
        return optmPath
    min_s = float('inf')
    for a, state in s.successors():
        if not state in path:
            action = a
            path.append(state)
            cost = action.cost
            new_f = doDFS(state, g+cost, bound, goaltest, h, path)
            if isinstance(new_f,list):
                #optmPath.insert(0,action)
                return new_f
            if new_f < min_s:
                min_s = new_f
            path.pop()
    return min_s

### Write your code here
### Write your code here
### Write your code here
### Write your code here
### Write your code here
### Write your code here
### Write your code here
### Write your code here
### Write your code here
### Write your code here
### Write your code here
### Write your code here

### NOTE: The grader will call both IDAstar and doDFS, so keep
### the interfaces to these intact. (We need to check doDFS
### separately because otherwise you could plug in your A*
### implementation as an IDA* impersonation. ;-) )
