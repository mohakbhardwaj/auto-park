__author__ = 'shivam'

import numpy as np

#Environment Parameters
env_width = 300
env_height = 360

#Create empty environment
env=[ [0 for i in range(env_width)] for j in range(env_height)]

#Todo:Add obstacles


#saving the environment
env_= np.asarray(env)
np.savetxt('env.txt',env_,fmt='%d')