import numpy as np
pi = 3.1415

def surf(x, y):
    z = 2.0 * np.sin (x * 2.0 * 3.1415 / 25.0) +  3.0 * np.sin(y * 2.0* 3.1415 / 35.0)
    return z

def surf_n(x, y):
    xn = (2.0 * 3.1415 / 25.0) * 2.0 * np.cos (x * 2.0 * 3.1415 / 25.0)
    yn = (2.0* 3.1415 / 35.0) * 3.0 * np.cos(y * 2.0* 3.1415 / 35.0)
    zn = 1;
    n = np.array([xn, yn, zn])
    n = n / np.linalg.norm(n)
    return n

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
N = 25
mus = [0., 0., 0.]
sigma = 160.
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
N = 12
mus = [5., -10., 0.]
sigma = 160.
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
N = 40
mus = [0., 0., 0.]
sigma = 160.
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
N = 6
mus = [0., 0., 0.]
sigma = 160.
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
#world_str = world_template % {"obstacles": obstacles_str}
world_str = world_template % {"obstacles": ""}
# f = open("kin_obstacles_generated.world", "w")
f = open("../kin.world", "w")
f.write(world_str)
f.close()
