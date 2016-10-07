from deap import tools

file_save = "rbt.pk"
population_start = 4
genes_start = 4
iteration = 2

# Has to be a number between 0.0 and 1.0 (because of random.random)
mate_percentage = 0.90
# Croos Over function and arguments from deap
# Can be: 
# tools.cxUniform,
# need another argument indpb
# see http://deap.gel.ulaval.ca/doc/default/api/tools.html#deap.tools.cxUniform
# Seriously look for a better one
crossover = [
  tools.cxMessyOnePoint # cxTwoPoint
]

# Has to be a number between 0.0 and 1.0 (because of random.random)
mutate_percentage = 0.90
mutate = tools.mutUniformInt # mutFlipBit
kwargs_mutate = {
  'indpb':0.90,
  'low':0,
  'up':300
}

selection = tools.selTournament
kwargs_selection = {
  'tournsize':3
}
