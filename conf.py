# Gain values controller
kp = 10
kd = 2

# Force field multiplication constants
G_r = 5 # repulsion from other repeaters and main drone
G_s = 5 # repulsion from obstacles and unseen areas
G_n = 0.5 # noise repulsion

# LOS parameters
wm = 2 # for distance to closest obstacle/unseen area
wl = 1 # for maximise the allowed distance from the followed drone
n_samples = 10 # number of points to evaluate. If bigger, better result but slower

# plotting
n_squares = 40 # n*n squares in the discretised space. If bigger, better simulation but slower
plot_step = 4 # update plot after n steps (bigger to speed up).

dt = 0.1

v_range = 2 # used to find the next position for a repeater.
use_sqrt = False # use square root in the optimization function or not
