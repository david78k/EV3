print('Look at this python code go!')

import sys
import math
import random
import string
import bpnn

for arg in sys.argv:
    print arg

random.seed(0)

#print 'demo'
network = bpnn.train()

inputs = sys.argv[0]
targets = sys.argv[1]

outputs = network.test([[inputs, targets]])
#outputs = bpnn.testPredefined()

"""
for output in outputs:
	print output
"""