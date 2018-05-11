import json
import pygame as pg
import numpy as np
from numpy.linalg import norm
import time
from shapely import geometry

class Repeater():

    def __init__(self, pos,index):
        self.position = pos
        self.velocity = np.array((0, 0), dtype=float)

        self.error_prev = np.array((0, 0))
        self.index=index

    def control(self, pos_desired, repeaters = [], boundary_centers = []):
        """
        PD controller to provide a control signal / acceleration
        :param pos_desired: Desired position to go towards
        :return: Control signal u
        """
        # todo: choose good parameters

        S_r = 10 # completely randomly set.
        R_r = 2 # radius of repeater (obstacle)

        repulsive_repeaters = np.array((0.0, 0.0))

        # Compute repulsive forces from repeaters
        if len(repeaters) > 1: # If more repeaters than this one
            for repeater in repeaters: # Look at all repeaters except for this one. Assign indices? d == 0?
                d = norm(repeater.position - self.position)

                # If repeater is far away, don't account for it at all
                # or if we have priority
                if d > S_r or d == 0 or self.index<repeater.index:
                    continue

                # Direction from repeater to self, normalized
                direction = (repeater.position - self.position) / d

                # If very close, large repulsive force
                if d <= R_r:
                    repulsive_repeaters += direction * 1000

                else:
                    repulsive_repeaters += ((S_r - d) / (S_r - R_r)) * direction

        S_s = 10  # completely randomly set.
        R_s = 5  # radius of repeater (obstacle) # todo: make it depend on side length

        repulsive_shaded = np.array((0.0, 0.0))

        # Compute repulsive forces from the boundary of non-seen area
        for center in boundary_centers:
            d = norm(center - self.position)

            # If repeater is far away, don't account for it at all
            # or if we have priority
            if d > S_s:
                continue

            # Direction from repeater to self, normalized
            direction = (center - self.position) / d

            # If very close, large repulsive force
            if d <= R_s:
                repulsive_shaded += direction * 1000

            else:
                repulsive_shaded += ((S_s - d) / (S_s - R_s)) * direction


        # Proportional and differential gains
        kp = 10; kd = 10

        # repulsive repeater gain, repulsive shaded gain
        G_r = 50; G_s = 50

        # PD controller
        error = pos_desired - self.position
        d_error = (error - self.error_prev) / dt
        u = kp * error + kd * d_error - G_r * repulsive_repeaters - G_s * repulsive_shaded

        self.error_prev = error

        return u

    def move(self, pos_desired, repeaters = [], boundary_centers = []):
        """
        Update the position, velocity and acceleration
        :param pos_desired:
        :return:
        """

        # Get the control signal (acceleration)
        u = self.control(pos_desired, repeaters, boundary_centers)

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




def update_map(position, squares):
    for i  in range(squares.shape[0]):
        for j in range (squares.shape[1]):
            square=squares[i,j]
            if square:
                if norm(position-square['center'])<scanning_range:
                    seen=True
                else:
                    seen=False
                if seen:
                    squares[square['i'],square['j']]=None
    return squares

def find_boundary(squares):
    indices=np.argwhere(squares==None)
    boundary=set()
    for index in  indices:
        i,j=index
        xs=[]
        ys=[]
        if i >0:
            xs.append(i-1)
        if i < squares.shape[0]-1:
            xs.append(i+1)
        if j >0:
            ys.append(j-1)
        if j < squares.shape[1]-1:
            ys.append(j+1)
        for x in xs:
            if squares[x,j]!=None:
                boundary.add((squares[x,j]['i'],squares[x,j]['j']))
            for y in ys:
                if squares[i, y] != None:
                    boundary.add((squares[i,y]['i'],squares[i,y]['j']))
                if squares[x,y] != None:
                    boundary.add((squares[x,y]['i'],squares[x,y]['j']))

    return boundary

def discretize(min_x,max_x,min_y,max_y,n_squares):
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

    squares=np.empty((len(xs)-1,len(ys)-1),dtype=object)
    for i in range(len(xs)-1):
        for j in range(len(ys)-1):
            square=geometry.Polygon([(xs[i],ys[j]),(xs[i],ys[j+1]),(xs[i+1],ys[j+1]),(xs[i+1],ys[j])])
            #squares.append({'square':square,'center':np.array(square.centroid)})
            squares[i,j]={'square':square,'center':np.array(square.centroid),'i':i,'j':j}
            a=0
    return squares

def list_to_pygame(list_of_points):
    pg_list_of_points=[]
    for point in list_of_points:
        pg_list_of_points.append(to_pygame(point))
    return  pg_list_of_points

def to_pygame(coords):
    '''Convert coordinates into pygame coordinates'''
    return (int(coords[0] * 5 + width / 2 - 150), int(coords[1] * -5 + height / 2 + 200))


def set_bg(repeaters,squares):
    '''set initial and final position'''
    screen.fill((255, 255, 255))
    '''for i in range(len(robots.locations)):
        pg_pos = to_pygame(robots.locations[i])
        pg.draw.circle(screen, robots.colors[i], (pg_pos[0], pg_pos[1]), 3, 0)
        if len(robots.all_locations)>1:
            for p in range(1,len(robots.all_locations)):
                pg.draw.line(screen,robots.colors[i],to_pygame(robots.all_locations[p-1][i]),to_pygame(robots.all_locations[p][i]))'''
    for i in range(1,len(traj_pos)):
        pg.draw.line(screen,(0,0,0),to_pygame(traj_pos[i-1]),to_pygame(traj_pos[i]))
    pg.draw.polygon(screen, (0, 0, 0), pg_bounding_polygon, 1)
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
            if square:
                pg.draw.polygon(s,(100,100,100,128),list_to_pygame(list(np.array(square['square'].exterior.coords)[:-1])))

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
traj_pos-=traj_pos[0]

dt=0.1

pg_bounding_polygon = []
for point in bounding_polygon:
    pg_bounding_polygon.append(to_pygame(point))

sh_bounding_polygon=geometry.Polygon(bounding_polygon)
min_x,min_y,max_x,max_y=sh_bounding_polygon.bounds
squares=discretize(min_x,max_x,min_y,max_y,10)

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
    squares=update_map(traj_pos[time_step],squares)  
    ## Now we look for the boundary of the unseen area, to find points to use for the force field
    boundary=find_boundary(squares)
    boundary_centers = []
    for boundary_index in boundary:
        boundary_centers.append(squares[boundary_index]['center'])



    if repeaters:
        # if we have repeaters we move them
        if norm(repeaters[-1].position-ground_station)>=desired_range:
            #if the last added repeater is going out of rage from the ground station we add a new one
            repeaters.append(Repeater(ground_station.copy(),len(repeaters)))
        for r in range(len(repeaters)):
            #for all the repeaters, if is the one following the payload we move towards it,
            # otherwise we move towards the next repeater in the chain
            if r==0:
                repeaters[r].move(traj_pos[time_step], repeaters, boundary_centers)#todo: go to the boss
            else:
                repeaters[r].move(repeaters[r-1].position, repeaters, boundary_centers)#todo: go to the previous repeater


    else:
    #if there are no repeaters, we check if the payload is on range.
    # if not, we add a repeater
        if norm(traj_pos[time_step]-ground_station)>=desired_range:
            repeaters.append(Repeater(ground_station.copy(),0))




    time_step += 1

    if time_step%10==0:
        set_bg(repeaters,squares)
        pg.display.flip()