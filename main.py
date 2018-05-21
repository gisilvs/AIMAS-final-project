import json
import numpy as np
from numpy.linalg import norm
import time
from shapely import geometry
import conf
import repeater
import pygame_utils
import pygame as pg

V_CONST=conf.v_range
wm = conf.wm; wl = conf.wl #for position optimization
sqrt=conf.use_sqrt


def update_map(position, squares, sh_bounding_lines,sh_obstacles):

    """
    Updates the discretized map when it is explored
    :param position: Coordinates of the main drone
    :param squares: Array with elements that are squares if they are not seen, otherwise None
    :param sh_bounding_lines: boundary of the mine in shapely format
    :param sh_obstacles: obstacles in shapely format
    :return: Updated Array with squares
    """
    for i  in range(squares.shape[0]):
        for j in range (squares.shape[1]):
            square=squares[i,j]
            if not square['seen']:
                center = square['center']
                if norm(position-center) < scanning_range:
                    seen = True
                    line = geometry.LineString([center, position])
                    if line.intersects(sh_bounding_lines):
                        seen=False

                    if seen:

                        for obstacle in sh_obstacles:
                            if line.intersects(obstacle.boundary):
                                seen=False
                                break
                            if square['square'].intersects(obstacle.boundary):  # or square['square'].touches(obstacle.boundary):
                                seen=False
                                break
                else:
                    seen=False
                if seen:
                    squares[square['i'],square['j']]['seen']=True

    return squares

def find_boundary(squares):
    """
    Function that finds the squares on the boundary between the seen and the unseen map, and the squares sorrounding obstacles
    :param squares: Array with elements that are squares if they are not seen, otherwise None
    :return: A list with center coordinates for the boundary squares
    """
    indices=[]
    for i,row in enumerate(squares):
        for j,square in enumerate (row):
            if square['seen']:
               indices.append((i,j))

    boundary=set()
    for index in indices:
        i,j=index
        xs=[]
        ys=[]
        if i >0:
            xs.append(i-1)
        if i < squares.shape[0] - 1:
            xs.append(i+1)
        if j > 0:
            ys.append(j-1)
        if j < squares.shape[1] - 1:
            ys.append(j+1)
        for x in xs:
            if not squares[x,j]['seen']:
                boundary.add((squares[x,j]['i'],squares[x,j]['j']))
            for y in ys:
                if not squares[i, y]['seen']:
                    boundary.add((squares[i,y]['i'],squares[i,y]['j']))
                if not squares[x,y]['seen']:
                    boundary.add((squares[x,y]['i'],squares[x,y]['j']))

    boundary_centers = []
    boundaries=[]
    for boundary_index in boundary:
        boundary_centers.append(squares[boundary_index]['center'])
        boundaries.append(squares[boundary_index])

    return boundary_centers,boundaries

def discretize(bounds, n_squares):

    """
    A function to discretize the whole map
    :param bounds: A tuple containing (min_x, max_x, min_y, max_y) of the bounds of the bounding polygon
    :param n_squares: Number of squares along the x-axis
    :return: Array of squares
    """

    min_x, min_y, max_x, max_y = bounds

    xs=[min_x]
    ys=[min_y]
    x=min_x
    y=min_y
    x_side=(max_x-min_x)/n_squares
    while x<max_x:
        x+=x_side
        xs.append(x)
    while y<max_y:
        y+=x_side
        ys.append(y)


    xs = [xs[0] - x_side] + xs
    xs = xs + [xs[-1] + x_side]
    ys = [ys[0] - x_side] + ys
    ys = ys + [ys[-1] + x_side]

    squares=np.empty((len(xs)-1,len(ys)-1),dtype=object)
    for i in range(len(xs)-1):
        for j in range(len(ys)-1):
            vertices = [(xs[i],ys[j]),(xs[i],ys[j+1]),(xs[i+1],ys[j+1]),(xs[i+1],ys[j])]
            square=geometry.Polygon(vertices)
            squares[i,j]={'vertices':vertices, 'center':np.array(square.centroid), 'i':i,'j':j,'seen':False, 'square':square}
    return squares

def get_bounding_lines(bounding_polygon):

    """
    Creates the bounding lines for the bounding polygon with a small opening in origin
    :param bounding_polygon: List of coords for the vertices for the bounding polygon
    :return: List of coords for the vertices, with opening
    """

    first_point = np.array(bounding_polygon[0])
    second_point = np.array(bounding_polygon[1])
    last_point = np.array(bounding_polygon[-1])
    dir1 = (second_point - first_point) / norm(second_point - first_point)
    dir2 = (last_point - first_point) / norm(last_point - first_point)

    bounding_lines = [first_point + 2*dir1]
    for point in bounding_polygon[1:]:
        point = np.array(point)
        bounding_lines.append(point)

    bounding_lines.append(first_point + 2*dir2)

    return bounding_lines


def sample_feasible_point(target_pos,repeater_pos,desired_range,sensor_range,repeater_v_max, boundary_obstacles):
    """
    this function first sample a bunch of pints from an area reacheble in few time steps, and among them picks the one that maximise
    our objective function
    :param target_pos: position of the followed drone
    :param repeater_pos: position of the repater computing its new position
    :param desired_range: desired line of sight to keep, smaller than the maximum LOS as precaution
    :param sensor_range: maximum range of the sensor (to have LOS we have to stay inside 2*sensor_range)
    :param repeater_v_max: max_velocity of repeater
    :param boundary_obstacles: boundary of unseen area and of obstacles. To take into account for line of sight
    :return: the next position the drone should point to
    """
    sh_inner=geometry.Point(target_pos).buffer(inner_range+sensor_range)
    sh_target = geometry.Point(target_pos).buffer(desired_range + sensor_range)#.difference(sh_inner)
    sh_repeater = geometry.Point(repeater_pos).buffer(V_CONST*repeater_v_max)
    desired_intersection = sh_target.intersection(sh_repeater)

    min_x, min_y, max_x, max_y = desired_intersection.bounds



    n_samples = conf.n_samples
    # Store sampled points in allowed region
    samples = []

    sample_found=False
    while not sample_found:
        while len(samples) < n_samples:

            s = np.random.uniform((min_x, min_y), (max_x, max_y))
            if geometry.Point(s).within(desired_intersection):
                #for obst in boundary_obstacles:
                samples.append(s)


        # Store the squares that are within range to even block the LOS
        dist = norm(target_pos - repeater_pos)
        squares_of_interest = []
        for square in boundary_obstacles:
            if norm(square['center'] - target_pos) < 2 * sensor_range:
                if norm(square['center'] - repeater_pos) < dist + V_CONST*repeater_v_max:
                    squares_of_interest.append(square)


        # Store sampled points in the allowed region to find ones that are in LOS
        good_samples = []
        for sample in samples:
            is_good = True
            for square in squares_of_interest:
                if geometry.LineString([target_pos, sample]).intersects(square['square']):
                    is_good = False
                    break
            if is_good:
                sample_found=True
                good_samples.append(sample)

        #if we don't find anything, we try to sample more
        if not good_samples:
            print('No point found, keep sampling')

    samples = np.array(good_samples)


    # for each sample in LOS
    sample_dline = []
    sample_dmin = []

    for sample in samples:
        # store distance from sample to target
        sample_dline.append(norm(sample - target_pos))
        dist_to_closest = np.inf
        line = geometry.LineString([target_pos, sample])
        for square in squares_of_interest:
            dist_to_line = geometry.Point(square['center']).distance(line)
            if dist_to_line < dist_to_closest:
                dist_to_closest = dist_to_line
            sample_dmin.append(dist_to_closest)


    wm = conf.wm; wl = conf.wl

    if sqrt:
        obj = lambda dmin, dline: wm*np.sqrt(dmin) + wl*np.sqrt(dline)
    else:
        obj = lambda dmin, dline: wm * dmin + wl*dline

    objective = np.array(list(map(obj, sample_dmin, sample_dline)))
    best_index = np.argmax(objective)
    best_point = samples[best_index]

    '''try:
        best_index = np.argmax(objective)
        best_point = samples[best_index]
    except:
        # todo: add a stop signal to target/main drone in this case
        print('cant find feasible point, go towards target!')
        best_point = target_pos'''

    return best_point


plotter=pygame_utils.Plotter() # contains the various plotting functions

"""Import configurations from json file"""
data = json.load(open('P25_X.json'))
traj=json.load(open('P25_26_traj.json'))

#mine polygon
bounding_polygon = data["bounding_polygon"]
#ground station position
ground_station=np.array(data["ground_station"],dtype=float)

#drones settings
sensor_range=data["sensor_range"]
desired_range=data["desired_range"]
scanning_range=data["scanning_range"]
inner_range=data["inner_range"]
a_max = data["vehicle_a_max"]
v_max = data["vehicle_v_max"]

#trajectory for main drone
traj_t=traj["t"]
traj_theta=traj["theta"]
traj_x=traj["x"]
traj_y=traj["y"]
traj_pos = np.array(list(zip(traj_x,traj_y))) * 1.4
traj_pos -= traj_pos[0]+np.array((-1,-1))

#obstacles
obstacles=[]
sh_obstacles=[]
for d in data:
    if "obstacle" in d:
        obstacle=data[d]
        obstacles.append(obstacle)
        sh_obstacles.append(geometry.Polygon(obstacle)) #obstacles in shapely format


sh_bounding_polygon=geometry.Polygon(bounding_polygon)
bounding_lines = get_bounding_lines(bounding_polygon)
sh_bounding_lines=geometry.LineString(bounding_lines)
bounds = sh_bounding_polygon.bounds
n_squares = conf.n_squares
plot_step = conf.plot_step
squares=discretize(bounds, n_squares)
obstacle_matrix = np.zeros(squares.shape)

repeaters=[]

time_step=0
start = False
done = False

while not done:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            done = True
    if not start:
        t0 = time.time()
        t1 = 0
        while (t1 - t0 < 1):
            t1 = time.time()
        start = True

    ## check if we see some square
    squares= update_map(traj_pos[time_step],squares, sh_bounding_lines,sh_obstacles)
    ## Now we look for the boundary of the unseen area, to find points to use for the force field
    boundary_centers,boundary = find_boundary(squares)
    pos_main_drone = traj_pos[time_step]

    if repeaters:
        for r in range(len(repeaters)):
            if r == 0:
                line=geometry.LineString([pos_main_drone,repeaters[r].position])
            else:
                line=geometry.LineString([repeaters[r].position,repeaters[r-1].position])
            for obstacle in boundary:
                if line.intersects(obstacle['square']):
                    print("Lost LOS")
                    exit(1)
        # if we have repeaters we move them
        if norm(repeaters[-1].position-ground_station)>=desired_range*2:
            #if the last added repeater is going out of rage from the ground station we add a new one
            repeaters.append(repeater.Repeater(ground_station.copy(),len(repeaters),a_max,v_max))
        for r in range(len(repeaters)):
            #for all the repeaters, we find the position where to move to keep line of sight with the target
            if r==0:
                target_pos=pos_main_drone
                repeater_pos=repeaters[r].position
                repeaters[r].pos_desired = sample_feasible_point(target_pos,repeater_pos,desired_range,sensor_range,v_max,boundary)
            else:
                target_pos = repeaters[r-1].position
                repeater_pos = repeaters[r].position
                repeaters[r].pos_desired = sample_feasible_point(target_pos, repeater_pos, desired_range, sensor_range, v_max,
                                                    boundary)

        # now move with all the drones (simulate decentralized and parallel decision making)
        for r in range(len(repeaters)):
            if r==0:
                repeaters[r].move(repeaters[r].pos_desired, pos_main_drone, repeaters, boundary_centers)
            else:
                target_pos = repeaters[r - 1].position
                repeaters[r].move(repeaters[r].pos_desired, target_pos,repeaters, boundary_centers)

    else:
    #if there are no repeaters, we check if the payload is on range.
    # if not, we add a repeater
        if norm(pos_main_drone-ground_station)>=desired_range*2:
            repeaters.append(repeater.Repeater(ground_station.copy(),0,a_max,v_max))




    time_step += 1

    if time_step%plot_step==0:
        plotter.set_bg(repeaters,squares,pos_main_drone,bounding_lines,obstacles,sensor_range,scanning_range,ground_station,desired_range)
        pg.display.flip()