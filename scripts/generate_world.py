import sys
#sys.path.append('../models/surf/python_wrapper/build')
import numpy as np
import surf_fcn_python_wrapper

pi = 3.1415

def surf(x, y):
    return surf_fcn_python_wrapper.surf_alt(x, y)

# seed
np.random.seed(205)

# read template
f = open("world.template", "r")
world_template = f.read()
f.close()

# read template
f = open("obstacle.template", "r")
obstacle_template = f.read()
f.close()

# obstacles
obstacles_str = ""

# generate obstacles tree1
N = 35
mus = [0., 0., 0.]
sigma = 200.
for i in range(N):
    model_name = "tree1"
    name = model_name + "_%s" % i
    x = mus[0] + sigma * (np.random.random() - 0.5)
    y = mus[0] + sigma * (np.random.random() - 0.5)
    z = surf(x, y)
    yaw = 2*pi*np.random.random()
    pos = "%s %s %s" % (x, y, z)
    att =  "%s %s %s" % (0, 0, yaw)
    obstacle_str = obstacle_template % {"obstacle_model": model_name, "obstacle_name": name, "obstacle_pos": pos, "obstacle_att": att}
    obstacles_str += obstacle_str
    
# generate obstacles tree2
N = 20
mus = [5., -10., 0.]
sigma = 200.
for i in range(N):
    model_name = "tree2"
    name = model_name + "_%s" % i
    x = mus[0] + sigma * (np.random.random() - 0.5)
    y = mus[0] + sigma * (np.random.random() - 0.5)
    z = surf(x, y)
    yaw = 2*pi*np.random.random()
    pos = "%s %s %s" % (x, y, z)
    att =  "%s %s %s" % (0, 0, yaw)
    obstacle_str = obstacle_template % {"obstacle_model": model_name, "obstacle_name": name, "obstacle_pos": pos, "obstacle_att": att}
    obstacles_str += obstacle_str
    
# generate obstacles rock1
N = 60
mus = [0., 0., 0.]
sigma = 200.
for i in range(N):
    model_name = "rock1"
    name = model_name + "_%s" % i
    x = mus[0] + sigma * (np.random.random() - 0.5)
    y = mus[0] + sigma * (np.random.random() - 0.5)
    z = surf(x, y)
    yaw = 2*pi*np.random.random()
    pos = "%s %s %s" % (x, y, z)
    att =  "%s %s %s" % (0, 0, yaw)
    obstacle_str = obstacle_template % {"obstacle_model": model_name, "obstacle_name": name, "obstacle_pos": pos, "obstacle_att": att}
    obstacles_str += obstacle_str
    
# generate obstacles rock2
N = 12
mus = [0., 0., 0.]
sigma = 200.
for i in range(N):
    model_name = "rock2"
    name = model_name + "_%s" % i
    x = mus[0] + sigma * (np.random.random() - 0.5)
    y = mus[0] + sigma * (np.random.random() - 0.5)
    z = surf(x, y)
    yaw = 2*pi*np.random.random()
    pos = "%s %s %s" % (x, y, z)
    att =  "%s %s %s" % (0, 0, yaw)
    obstacle_str = obstacle_template % {"obstacle_model": model_name, "obstacle_name": name, "obstacle_pos": pos, "obstacle_att": att}
    obstacles_str += obstacle_str
	
	
# write word
world_str = world_template % {"obstacles": obstacles_str}
#world_str = world_template % {"obstacles": ""}
# f = open("kin_obstacles_generated.world", "w")
f = open("../kin.world", "w")
f.write(world_str)
f.close()
