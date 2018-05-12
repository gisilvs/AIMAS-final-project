import json
import pygame as pg
import numpy as np
from numpy.linalg import norm
import time
from shapely import geometry

class Repeater():

    """
    A class for the repeater drones
    """

    def __init__(self, pos,index):
        self.position = pos
        self.velocity = np.zeros(2)

        self.error_prev = np.zeros(2)
        self.index=index

        # For noise in the control
        self.percistance_counter = 0
        self.noise_direction = np.zeros(2)

    def move(self, pos_desired, pos_main_drone, repeaters, boundary_centers, obstacle_centers):

        """
        Updates the position and velocity of the repeater.
        :param pos_desired: Desired position to go towards
        :param repeaters: A list with all the repeater drones. To avoid collision
        :param boundary_centers: A list with the center coordinates of the boundary
        between seen and unseen area
        :return:
        """

        # Get the control signal (acceleration)
        u = self.control(pos_desired, pos_main_drone, repeaters, boundary_centers, obstacle_centers)

        # Make sure it's not too large
        if norm(u) > a_max:
            u = u / norm(u) * a_max

        # Update velocity
        self.velocity += u * dt

        # Make sure it's not too large
        if norm(self.velocity) > v_max:
            self.velocity = self.velocity / norm(self.velocity) * v_max

        # Update position
        self.position += self.velocity

    def get_repulsive(self, centers, S = 10, R = 3):

        repulsive = np.array((0.0, 0.0))

        for center in centers:
            d = norm(center - self.position)

            # If repeater is far away, don't account for it at all
            # or if we have priority
            if d > S:
                continue

            # Direction from repeater to self, normalized
            direction = (center - self.position) / d

            # If very close, large repulsive force
            if d <= R:
                repulsive += direction * 1000

            else:
                repulsive += ((S - d) / (S - R)) * direction

        return repulsive


    def get_repulsive_repeaters(self, repeaters, S = 10, R = 3):

        repulsive = np.array((0.0, 0.0))

        # Compute repulsive forces from repeaters
        if len(repeaters) > 1:  # If more repeaters than this one
            for repeater in repeaters:  # Look at all repeaters except for this one. Assign indices? d == 0?
                d = norm(repeater.position - self.position)

                # If repeater is far away, don't account for it at all
                # or if we have priority
                if d > S or d == 0 or self.index < repeater.index:
                    continue

                # Direction from repeater to self, normalized
                direction = (repeater.position - self.position) / d

                # If very close, large repulsive force
                if d <= R:
                    repulsive += direction * 1000

                else:
                    repulsive += ((S - d) / (S - R)) * direction

        return repulsive

    def get_repulsive_main(self, pos_main_drone, S = 10, R = 3):

        repulsive = np.array((0.0, 0.0))

        d = norm(pos_main_drone - self.position)

        # Direction from main drone to self, normalized
        direction = (pos_main_drone - self.position) / d

        # If very close, large repulsive force
        if d <= R:
            repulsive += direction * 1000

        else:
            repulsive += ((S - d) / (S - R)) * direction

        return repulsive

    def get_noise(self):

        if self.percistance_counter % 10 == 0:
            noise_direction = np.random.normal(0, 1, 2)
            self.noise_direction = noise_direction / norm(noise_direction)

        self.percistance_counter += 1

        return self.noise_direction

    def control(self, pos_desired, pos_main_drone, repeaters, boundary_centers, obstacle_centers):

        """
        PD controller to provide a control signal / acceleration
        :param pos_desired: Desired position to go towards
        :param repeaters: A list with all the repeater drones. To avoid collision
        :param boundary_centers: A list with the center coordinates of the boundary
        between seen and unseen area
        :return: Control signal u
        """

        # todo: choose good parameters for the gains
        # Proportional and differential gains
        kp = 10
        kd = 10

        # repulsive repeater gain, repulsive shaded gain, noise gain
        G_r = 1
        G_s = 1
        G_n = 0.5

        ### Repeaters
        repulsive_repeaters = G_r * self.get_repulsive_repeaters(repeaters)

        #### Boundary of non-seen
        repulsive_shaded = G_s * self.get_repulsive(boundary_centers)

        #### Obstacles
        repulsive_obstacles = G_s * self.get_repulsive(obstacle_centers)

        #### Main drone
        repulsive_main_drone = G_r * self.get_repulsive_main(pos_main_drone)

        ### Noise
        noise = G_n * self.get_noise()

        # Accumulated repulsive force from obstacles, other drones and unseen area.
        repulsive = - repulsive_repeaters - repulsive_shaded - repulsive_obstacles - repulsive_main_drone

        # PD controller
        error = pos_desired - self.position
        d_error = (error - self.error_prev) / dt

        # Control signal
        u = kp * error + kd * d_error + repulsive + noise

        self.error_prev = error

        return u


def update_map(position, squares, sh_bounding_lines, obstacle_matrix):

    """
    Updates the discretized map when it is explored
    :param position: Coordinates of the main drone
    :param squares: Array with elements that are squares if they are not seen, otherwise None
    :return: Updated Array with squares
    """

    for i  in range(squares.shape[0]):
        for j in range (squares.shape[1]):
            square=squares[i,j]
            if not square['seen']:
                center = square['center']
                if norm(position-center) < scanning_range:
                    seen = True

                    # Check if it intersects with obstacle / boundary
                    line = geometry.LineString([center, position])
                    if line.intersects(sh_bounding_lines):
                        obstacle_matrix[i, j] = 1

                else:
                    seen=False
                if seen:
                    squares[square['i'],square['j']]['seen']=True

    return squares, obstacle_matrix

def get_obstacle_centers(squares, obstacle_matrix):

    n, m = obstacle_matrix.shape
    centers = []
    for i in range(n):
        for j in range(m):
            if obstacle_matrix[i, j]:
                centers.append(squares[i, j]['center'])

    return centers


def find_boundary(squares):

    """
    Function that finds the squares on the boundary between the seen and the unseen map
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
    for boundary_index in boundary:
        boundary_centers.append(squares[boundary_index]['center'])

    return boundary_centers

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
            squares[i,j]={'square':vertices, 'center':np.array(square.centroid), 'i':i,'j':j,'seen':False}
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

def list_to_pygame(list_of_points):
    pg_list_of_points=[]
    for point in list_of_points:
        pg_list_of_points.append(to_pygame(point))
    return  pg_list_of_points

def to_pygame(coords):

    """
    Convert coordinates into pygame coordinates
    """

    return (int(coords[0] * 5 + width / 2 - 150), int(coords[1] * -5 + height / 2 + 200))


def set_bg(repeaters,squares,obstacle_matrix):

    """
    set initial and final position
    """

    screen.fill((255, 255, 255))
    '''for i in range(len(robots.locations)):
        pg_pos = to_pygame(robots.locations[i])
        pg.draw.circle(screen, robots.colors[i], (pg_pos[0], pg_pos[1]), 3, 0)
        if len(robots.all_locations)>1:
            for p in range(1,len(robots.all_locations)):
                pg.draw.line(screen,robots.colors[i],to_pygame(robots.all_locations[p-1][i]),to_pygame(robots.all_locations[p][i]))'''
    for i in range(1,len(traj_pos)):
        pg.draw.line(screen,(0,0,0),to_pygame(traj_pos[i-1]),to_pygame(traj_pos[i]))
    #pg.draw.polygon(screen, (0, 0, 0), pg_bounding_polygon, 1)
    '''for pos in vs.desired_pos:
        pg_pos = to_pygame(pos)
        pg.draw.circle(screen, (0, 0, 255), (pg_pos[0], pg_pos[1]), 3, 1)'''
    pg.draw.line(screen,(255,0,0),to_pygame(traj_pos[time_step]+np.array([1,1])),to_pygame(traj_pos[time_step]+np.array([-1,-1])),4)
    pg.draw.line(screen, (255, 0, 0), to_pygame(traj_pos[time_step] + np.array([-1, 1])),
                 to_pygame(traj_pos[time_step] + np.array([1, -1])),4)
    pg.draw.circle(screen,(255,0,0),to_pygame(traj_pos[time_step]),scanning_range*5,1)
    pg.draw.circle(screen,(0,153,0),to_pygame(ground_station),5)
    pg.draw.circle(screen,(0,153,0),to_pygame(ground_station),sensor_range*5,1)
    pg.draw.circle(screen, (0, 255, 0), to_pygame(ground_station), desired_range * 5, 1)
    if repeaters:
        for repeater in repeaters:
            pg.draw.line(screen, (0, 0, 255), to_pygame(repeater.position + np.array([0.8, 0.8])),
                     to_pygame(repeater.position+ np.array([-0.8, -0.8])), 3)
            pg.draw.line(screen, (0, 0, 255), to_pygame(repeater.position + np.array([-0.8, 0.8])),
                     to_pygame(repeater.position + np.array([0.8, -0.8])), 3)
    s=pg.Surface((infoObject.current_w, infoObject.current_h),pg.SRCALPHA)
    for i in range(squares.shape[0]):
        for j in range(squares.shape[1]):
            square = squares[i, j]
            if not square['seen']:
                pg.draw.polygon(s,(100,100,100,128), list_to_pygame(square['square']))
            if obstacle_matrix[i,j]==1:
                pg.draw.polygon(screen, (222, 184, 135), list_to_pygame(square['square']))

    for i in range(1,len(bounding_lines)):
        pg.draw.line(screen,(0,0,0),to_pygame(bounding_lines[i-1]),to_pygame(bounding_lines[i]))
    screen.blit(s, (0, 0))
# PyGame parameters
pg.init()
infoObject = pg.display.Info()
screen = pg.display.set_mode((infoObject.current_w, infoObject.current_h))
width = infoObject.current_w
height = infoObject.current_h
background_colour = (255, 255, 255)
screen.fill(background_colour)


data = json.load(open('P25_X.json'))
traj=json.load(open('P25_26_traj.json'))
bounding_polygon = data["bounding_polygon"]
ground_station=np.array(data["ground_station"],dtype=float)
sensor_range=data["sensor_range"]
desired_range=data["desired_range"]
scanning_range=data["scanning_range"]

a_max = data["vehicle_a_max"]
v_max = data["vehicle_v_max"]


traj_t=traj["t"]
traj_theta=traj["theta"]
traj_x=traj["x"]
traj_y=traj["y"]
traj_pos=np.array(list(zip(traj_x,traj_y)))
traj_pos-=traj_pos[0]+np.array((-1,-1))

dt=0.1

pg_bounding_polygon = []
for point in bounding_polygon:
    pg_bounding_polygon.append(to_pygame(point))

sh_bounding_polygon=geometry.Polygon(bounding_polygon)

bounding_lines = get_bounding_lines(bounding_polygon)
sh_bounding_lines=geometry.LineString(bounding_lines)



bounds = sh_bounding_polygon.bounds
n_squares = 30
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
    squares, obstacle_matrix = update_map(traj_pos[time_step],squares, sh_bounding_lines, obstacle_matrix)
    ## Now we look for the boundary of the unseen area, to find points to use for the force field
    boundary_centers = find_boundary(squares)
    obstacle_centers = get_obstacle_centers(squares, obstacle_matrix)


    pos_main_drone = traj_pos[time_step]

    if repeaters:
        # if we have repeaters we move them
        if norm(repeaters[-1].position-ground_station)>=desired_range:
            #if the last added repeater is going out of rage from the ground station we add a new one
            repeaters.append(Repeater(ground_station.copy(),len(repeaters)))
        for r in range(len(repeaters)):
            #for all the repeaters, if is the one following the payload we move towards it,
            # otherwise we move towards the next repeater in the chain
            if r==0:
                pos_desired = traj_pos[time_step] # todo: here do the optimization?
                repeaters[r].move(pos_desired, pos_main_drone, repeaters, boundary_centers, obstacle_centers)#todo: go to the boss
            else:
                repeaters[r].move(repeaters[r-1].position, pos_main_drone,
                                  repeaters, boundary_centers, obstacle_centers)#todo: go to the previous repeater


    else:
    #if there are no repeaters, we check if the payload is on range.
    # if not, we add a repeater
        if norm(traj_pos[time_step]-ground_station)>=desired_range:
            repeaters.append(Repeater(ground_station.copy(),0))




    time_step += 1

    if time_step%5==0:
        set_bg(repeaters,squares,obstacle_matrix)
        pg.display.flip()