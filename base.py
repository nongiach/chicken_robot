import random, config, vrep, math, time, copy, pickle
from deap import base, creator, tools
#from algo import Loop
random.seed()

# simx_opmode_blocking
# http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm
def init_simulation(opmode = vrep.simx_opmode_oneshot_wait):
  # Close eventual old connections
  vrep.simxFinish(-1)
  # Connect to V-REP remote server              # Args could be set to False if real remote
  clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

  if clientID == -1:
    raise Exception('Not Connected to remote API server')
  ret = []
  # Try to retrieve motors and robot handlers
  # don't rely on the automatic name adjustment mechanism
  # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectHandle
  sret, hret = vrep.simxGetObjectHandle(clientID, "WristMotor", opmode)
  if sret == -1:
    raise Exception('Can\'t Connecte to motor')
  ret.append(hret)
  sret, hret = vrep.simxGetObjectHandle(clientID, "ElbowMotor", opmode)
  if sret == -1:
    raise Exception('Can\'t Connecte to motor')
  ret.append(hret)
  sret, hret = vrep.simxGetObjectHandle(clientID, "ShoulderMotor", opmode)
  if sret == -1:
    raise Exception('Can\'t Connecte to motor')
  ret.append(hret)
  sret, robot = vrep.simxGetObjectHandle(clientID, "2W1A", opmode)
  if sret == -1:
    raise Exception('Can\'t Connecte to motor')
  return clientID, ret, robot

def stop_simulation(clientID, opmode=vrep.simx_opmode_oneshot_wait):
  vrep.simxStopSimulation(clientID, opmode)
  print("--simulation stopped--")
  time.sleep(1)

def start_simulation(clientID, opmode=vrep.simx_opmode_oneshot_wait):
  vrep.simxStartSimulation(clientID, opmode)
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
  
def l_gen(motors):
  g = Gene(random.randint(0, 300))
  g.motor = motors[random.randint(0, len(motors) - 1)]
  return g

def init_deap_lib(clientID, roboth, motors, pos_init):
  # Set max Value for Problem resolution.
  creator.create("FitnessMax", base.Fitness, weights=(1.0,))
  # define An individue
  creator.create("Individual", list, fitness=creator.FitnessMax)
  toolbox = base.Toolbox()

  toolbox.register("generator_gene", lambda: l_gen(motors))

  # Structure initializers
  toolbox.register("individual", tools.initRepeat, creator.Individual,
                   toolbox.generator_gene, config.genes_start)
  toolbox.register("population", tools.initRepeat, list, toolbox.individual)

  # Register function to eval individual
  toolbox.register("evaluate", eval_individual, clientID, roboth, pos_init)
  # CrossOver Choose by config
  toolbox.register("mate", *config.crossover)
  # Mutation choose in configuration 
  toolbox.register("mutate", config.mutate, **config.kwargs_mutate)
  # Selection choosen in configuration
  toolbox.register("select", config.selection, **config.kwargs_selection)
  return toolbox

def move_motor_angle(clientID, motor, angle, opmode=vrep.simx_opmode_oneshot_wait):
   vrep.simxSetJointTargetPosition(clientID, motor, math.radians(angle), opmode)


def eval_individual(clientID, roboth, pos_init, individual):
  """
    Eval an individue.
    For that run the simulation and calculate the distance.
  """
  start_simulation(clientID)
  for angle in individual:
    move_motor_angle(clientID, angle.motor, angle)
    time.sleep(1)
  new_pos = get_position(clientID, roboth)
  stop_simulation(clientID)
  dist = put_value_on_position(pos_init, new_pos)
  value = dist / float(len(individual))
  print("Value of this individual %s" % value)
  return (value, ) # Need to be return as an iterable ?

def put_value_on_position(pos_init, new_pos):
  dist = math.sqrt((new_pos[0] - pos_init[0])**2 + (new_pos[1] - pos_init[1])**2)
  return dist

  
def get_position(clientID, handler, opmode=vrep.simx_opmode_oneshot_wait):
  ret, pos = vrep.simxGetObjectPosition(clientID, handler, -1, opmode)
  return pos

def evolution(pop, toolbox):
  print("Genetique Evolution Started")
  for i in range(config.iteration):
    # Select the next generation individuals
    print("Population %s" % pop)
    new_pop = list(map(toolbox.clone, toolbox.select(pop, len(pop))))
    print("New Population %s" % new_pop)
    # Apply crossover - we could also take radom ind from new_pop
    for child1, child2 in zip(new_pop[::2], new_pop[1::2]):
      if random.random() < config.mate_percentage:
        mts = toolbox.mate(child1, child2)
        del child1.fitness.values
        del child2.fitness.values
    print("Mate %s" % new_pop)
    new_pop = [n for n in new_pop if len(n) != 0]
    # apply mutation
    for mutant in new_pop:
      if random.random() < config.mutate_percentage:
        time.sleep(0.5)
        org = copy.deepcopy(mutant)
        mts = toolbox.mutate(mutant)[0]
        for i in range(len(org)):
          if not type(mts[i]) is Gene:
            mts[i] = Gene(mts[i])
            mts[i].motor = org[i].motor
        del mutant.fitness.values
        time.sleep(0.5)

    # Evaluate the individuals that haven't been
    invalid_ind = [ind for ind in new_pop if not ind.fitness.valid]
    fitnesses = map(toolbox.evaluate, invalid_ind)
    for ind, fit in zip(invalid_ind, fitnesses):
      ind.fitness.values = fit
    pop = new_pop
  print("End Evolution\n%s" % pop)
  return pop

def write_population(population):
  with open(config.file_save, "wb") as f:
    pickle.dump(population, f)
def load_population():
  with open(config.file_save, "rb") as f:
    pop = pickle.load(f)
  return pop

def main(savedrobot=None):
  opmode = vrep.simx_opmode_oneshot_wait
  clientID, motors, roboth = init_simulation()
  opmode=vrep.simx_opmode_oneshot_wait
  stop_simulation(clientID, opmode)
  pos_init = get_position(clientID, roboth, opmode=opmode)
  toolbox = init_deap_lib(clientID, roboth, motors, pos_init)
  pop = toolbox.population(n=config.population_start)
  fitnesses = list(map(toolbox.evaluate, pop))
  for ind, fit in zip(pop, fitnesses):
    ind.fitness.values = fit
  population = evolution(pop, toolbox)
  write_population(population)
  print(load_population())
  return None
 # l = Loop(configfile, savedrobot=savedrobot)
 # l()
  

if __name__ == '__main__':
  import sys
  savedrobot=None
  if len(sys.argv) >= 2:
    savedrobot = sys.argv[1]
  main()


  


# print motorsHandlers['Gyro']['status']
  # print vrep.simxGetObjectGroupData(clientID, motorsHandlers['Gyro']['handler'], 0, vrep.simx_opmode_oneshot_wait)
  # print vrep.simxGetObjectGroupData(clientID, motorsHandlers['Gyro']['handler'], 1, vrep.simx_opmode_oneshot_wait)
  # print vrep.simxGetObjectGroupData(clientID, motorsHandlers['Gyro']['handler'], 2, vrep.simx_opmode_oneshot_wait)
  # print vrep.simxGetObjectOrientation(clientID, motorsHandlers['Gyro']['handler'], -1, vrep.simx_opmode_oneshot_wait)
  # data=simTubeRead(gyroCommunicationTube)
  #  if (data) 
  #    angularVariations=simUnpackFloats(data)

