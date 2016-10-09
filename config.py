from deap import tools, gp

saving = True
file_save = "rbt.pk"
verbose = True
population_start = 300
genes_start = 30
iteration = 50
select_best_at_end_iteration = False
num_best_select = 1
filename_to_save_best = "bst_rbt.pk"

# Has to be a number between 0.0 and 1.0 (because of random.random)
crossover_percentage = 0.25
# Croos Over function and arguments from deap
# Can be: 
# tools.cxUniform,
# need another argument indpb
# see http://deap.gel.ulaval.ca/doc/default/api/tools.html#deap.tools.cxUniform
# Seriously look for a better one
crossover = [
  tools.cxMessyOnePoint # cxTwoPoint
]
kwargs_crossover = {
}

# Has to be a number between 0.0 and 1.0 (because of random.random)
mutate_percentage = 0.35
mutate = tools.mutGaussian  #mutUniformInt
kwargs_mutate = {
  'indpb':0.45,
  'mu':12,
  'sigma':2
#  'low':0,
#  'up':300
}

selection = tools.selDoubleTournament # tools.selTournament
kwargs_selection = {
  'fitness_size':3,
  'parsimony_size':2,
  'fitness_first':False
#  'tournsize':3
}
