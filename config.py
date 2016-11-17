from deap import tools, gp

verbose = True
saving = True
file_save = "rbt.pk"
population_start = 30
nb_robots = 5 # don't change unless you change pos_x_start
pos_x_start = -1.75
genes_start = 40
iteration = 20
select_best_at_end_iteration = True
num_best_select = 5
filename_to_save_best = "bst_rbt1.pk"
save_at_each_turn = True

# Has to be a number between 0.0 and 1.0 (because of random.random)
crossover_percentage = 0.35
# Croos Over function and arguments from deap
# Can be: 
# tools.cxUniform,
# need another argument indpb
# see http://deap.gel.ulaval.ca/doc/default/api/tools.html#deap.tools.cxUniform
# Seriously look for a better one
crossover = [
  tools.cxUniform #cxMessyOnePoint # cxTwoPoint
]
kwargs_crossover = {
  'indpb': 0.80
}

# Has to be a number between 0.0 and 1.0 (because of random.random)
mutate_percentage = 0.85
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
