import random, config, vrep, math, time, copy, pickle
from deap import base, creator, tools
from functools import partial
import threading
#from algo import Loop
random.seed()

IHROBOT=3

def get_robot_object(clientID, n=0):
  opmode = vrep.simx_opmode_oneshot_wait
  vrep_get = partial(vrep.simxGetObjectHandle, clientID=clientID, operationMode=opmode)
  index = "" if n == 0 else "#{}".format(n - 1)
  ret_test, robotHandle = vrep_get(objectName="2W1A" + index)
  if ret_test != 0:
    _, r = vrep_get(objectName="2W1A")
    vrep.simxCopyPasteObjects(clientID, [r], opmode)
  ret1, wristHandle = vrep_get(objectName="WristMotor" + index)
  ret2, elbowHandle = vrep_get(objectName="ElbowMotor" + index)
  ret3, shoulderHandle = vrep_get(objectName="ShoulderMotor" + index)
  ret4, robotHandle = vrep_get(objectName="2W1A" + index)
  if ret1 == ret2 == ret3 == ret4 == 0:
    return wristHandle, elbowHandle, shoulderHandle, robotHandle
  raise("vrep_get error (be sure the chicken robot is there) 2W1A0{}".format(index))  

# simx_opmode_blocking
# http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm
def init_simulation(opmode = vrep.simx_opmode_oneshot_wait):
  # Close eventual old connections
  vrep.simxFinish(-1)
  # Connect to V-REP remote server              # Args could be set to False if real remote
  clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
  handlers = []
  pos_inits =  []
  
  if clientID == -1:
    raise Exception('Not Connected to remote API server')

  stop_simulation(clientID, opmode)
  handlers.append(get_robot_object(clientID))
  pos_inits.append(get_position(clientID, handlers[0][IHROBOT]))
  vrep.simxSetObjectPosition(clientID, handlers[0][IHROBOT], -1, \
                               [0.0, config.pos_x_start, 0.075], opmode)
  for i in range(1, config.nb_robots):
    handlers.append(get_robot_object(clientID, n=i))
    vrep.simxSetObjectPosition(clientID, handlers[i][IHROBOT], -1, \
                               [0.0, config.pos_x_start + (0.75 * i), 0.075], opmode)
    pos_inits.append(get_position(clientID, handlers[i][IHROBOT]))
  return clientID, handlers, pos_inits

def stop_simulation(clientID, opmode=vrep.simx_opmode_oneshot_wait):
  vrep.simxStopSimulation(clientID, opmode)
  if config.verbose:
    print("--simulation stopped--")
  time.sleep(0.5)

def start_simulation(clientID, opmode=vrep.simx_opmode_oneshot_wait):
  vrep.simxStartSimulation(clientID, opmode)
  time.sleep(0.5)
  if config.verbose:
    print("--simulation started--")

class Gene(int):
  motor = None

  def __deepcopy__(self, memo):
    r = Gene(self)
    r.motor = self.motor
    return r

  def __copy__(self):
    r = Gene(self)
    r.motor = self.motor
    return r

  def __str__(self):
    return "m[%s]:%s" % (self.motor, int(self))
  def __repr__(self):
    return str(self)
  
def l_gen():
  g = Gene(random.randint(0, 300))
  g.motor = random.randint(0, 2)
  return g

def init_deap_lib(clientID, handlers, pos_inits):
  # Set max Value for Problem resolution.
  creator.create("FitnessMax", base.Fitness, weights=(1.0,))
  # define An individue
  creator.create("Individual", list, fitness=creator.FitnessMax)
  toolbox = base.Toolbox()

#  toolbox.register("generator_gene", lambda: l_gen(motors))

  # Structure initializers
  toolbox.register("individual", tools.initRepeat, creator.Individual,
                   l_gen, config.genes_start) #toolbox.generator_gene
  toolbox.register("population", tools.initRepeat, list, toolbox.individual)

  # Register function to eval individual
  toolbox.register("evaluate", eval_pop, clientID, handlers, pos_inits)
  # CrossOver Choose by config
  toolbox.register("crossover", *config.crossover, **config.kwargs_crossover)
  # Mutation choose in configuration 
  toolbox.register("mutate", config.mutate, **config.kwargs_mutate)
  # Selection choosen in configuration 
  toolbox.register("select", config.selection, **config.kwargs_selection)
  return toolbox

def move_motor_angle(clientID, motor, angle, opmode=vrep.simx_opmode_oneshot):
  while angle > 300:
    angle /= 2
#  if config.verbose:
#    print("[%s] %s" % (motor, angle))
  vrep.simxSetJointTargetPosition(clientID, motor, math.radians(angle), opmode)

def eval_pop(clientID, handlers, pos_inits, pop):
  x = 0
  for x in range(0, len(pop), config.nb_robots):
    _pop = pop[x:x+config.nb_robots]
    start_simulation(clientID)
    # Threading
    for _handlers, pos_init, ind in zip( handlers, pos_inits, _pop):
      args=(clientID, _handlers, pos_init, ind)
      t = threading.Thread(group=None, target=eval_individual, args=args, daemon=True)
      t.start()
    # Wait until  Thread complete
    while threading.active_count() > 1:
      time.sleep(0.2)
    
    stop_simulation(clientID)
   
def eval_individual(clientID, handler, pos_init, individual):
  """
    Eval an individue.
    For that run the simulation and calculate the distance.
  """
  roboth, handler =  handler[-1], handler[:-1]
  for angle in individual:
    move_motor_angle(clientID, handler[angle.motor], copy.copy(angle))
    time.sleep(0.2)
  new_pos = get_position(clientID, roboth)
  dist = put_value_on_position(pos_init, new_pos)
  value = dist / float(len(individual))
  individual.fitness.value = (value, )
  if config.verbose:
    print("Value of this individual %s" % value)
  return None

def put_value_on_position(pos_init, new_pos):
  dist = math.sqrt((new_pos[0] - pos_init[0])**2 + (new_pos[1] - pos_init[1])**2)
  return dist

  
def get_position(clientID, handler, opmode=vrep.simx_opmode_oneshot_wait):
  ret, pos = vrep.simxGetObjectPosition(clientID, handler, -1, opmode)
  return pos

def print_ind(ind):
  s = " - ".join(["m[%s] %s" % (gene.motor, str(gene)) for gene in ind])
  print(s)

def evolution(pop, toolbox):
  print("Genetique Evolution Started")
  for i in range(config.iteration):
    # Select the next generation individuals
    print("Iteration %d/%d" % (i, config.iteration))
    check_population(pop)
    if config.verbose:
      print("Population %s" % len(pop))
      print(pop)
    new_pop = [toolbox.clone(x) for x in toolbox.select(pop, config.population_start)]
    # Apply crossover - we could also take radom ind from new_pop
    #
    
    for child1, child2 in zip(new_pop[::2], new_pop[1::2]):
      if random.random() < config.crossover_percentage:
        toolbox.crossover(child1, child2)
        del child1.fitness.values
        del child2.fitness.values

    new_pop = [n for n in new_pop if len(n) != 0]
    # apply mutation
    for mutant in new_pop:
      if random.random() < config.mutate_percentage:
        org = copy.deepcopy(mutant)
        mts = toolbox.mutate(mutant)[0]
        for i in range(len(org)):
          if not type(mts[i]) is Gene:
            mts[i] = Gene(mts[i])
            # # Try out a new motor
            mts[i].motor = org[i].motor # random.randint(0, 2) #  org[i].motor
        del mutant.fitness.values

    # Evaluate the individuals that haven't been
    invalid_ind = [ind for ind in new_pop if not ind.fitness.valid]
    toolbox.evaluate(invalid_ind)
    pop = new_pop
    if config.save_at_each_turn:
      write_population(pop)
  print("End Evolution\n%s" % pop)
  return pop

def write_population(population, path=None):
  p = path if path else config.file_save
  with open(p, "wb") as f:
    pickle.dump(population, f)
  print("State saved @%s" % p)

def load_population(toolbox, savedrobot=None):
  filen = savedrobot if type(savedrobot) is str else config.file_save
  if not savedrobot:
    return toolbox.population(n=config.population_start)
  with open(filen, "rb") as f:
    pop = pickle.load(f)
  if config.verbose:
    print("Log from file ", filen, " size population ", len(pop))
  return pop

def check_population(population):
  for ind in population:
    for i in range(len(ind)):
      if not type(ind[i]) is Gene:
        m = randon.randint(0, 2)
      else:
        m = ind[i].motor
      if ind[i] > 300:
        while ind[i] > 300:
          ind[i] /= 2
        ind[i] = Gene(ind[i])
        ind[i].motor = m

def main(opmode, clientID, handlers, pos_inits, toolbox, savedrobot=None):
  # Get initial Popluation from File or from a new population
  pop = savedrobot if savedrobot else toolbox.population(n=config.population_start)

  check_population(pop)
  toolbox.evaluate(pop)
  # Launch evolution
  population = evolution(pop, toolbox)
  if config.select_best_at_end_iteration:
    best = tools.selBest(population, config.num_best_select)
    write_population(best, path=config.filename_to_save_best)
  if config.saving:
    write_population(population)

def preload_main(savedrobot=None):
  opmode = vrep.simx_opmode_oneshot_wait
  clientID, handlers, pos_inits = init_simulation() # motors, roboth
  
#  stop_simulation(clientID, opmode)
#  pos_init = get_position(clientID, roboth, opmode=opmode)
  toolbox = init_deap_lib(clientID, handlers, pos_inits)
  savedrobot = load_population(toolbox, savedrobot=savedrobot)
  return main(opmode, clientID, handlers, pos_inits, toolbox, savedrobot=savedrobot)
  
 # l = Loop(configfile, savedrobot=savedrobot)
 # l()
  

if __name__ == '__main__':
  import sys
  savedrobot=None
  if len(sys.argv) >= 2:
    if sys.argv[1] == "False":
      savedrobot = False
    elif sys.argv[1] == "True" or sys.argv[1] == "true":
      savedrobot = True
    else:
      savedrobot = sys.argv[1]
  preload_main(savedrobot=savedrobot)


