import json
import pygame as pg
import numpy as np
from numpy.linalg import norm
import time
from shapely import geometry

class Repeater():

    def __init__(self, pos):
        self.position = pos
        self.velocity = np.array((0, 0), dtype=float)

        self.error_prev = np.array((0, 0))

    def control(self, pos_desired):
        """
        PD controller to provide a control signal / acceleration
        :param pos_desired: Desired position to go towards
        :return: Control signal u
        """

        # Proportional and differential gains # Todo: do these values make sense?
        kp = 10; kd = 10

        # PD controller
        error = pos_desired - self.position
        d_error = (error - self.error_prev) / dt
        u = kp*error + kd * d_error

        self.error_prev = error

        return u

    def move(self, pos_desired):
        """
        Update the position, velocity and acceleration
        :param pos_desired:
        :return:
        """

        # Get the control signal (acceleration)
        u = self.control(pos_desired)

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






def discretize(min_x,max_x,min_y,max_y,n_squares):
    squares=[]
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

    for i in range(len(xs)-1):
        for j in range(len(ys)-1):
            square=geometry.Polygon([(xs[i],ys[j]),(xs[i],ys[j+1]),(xs[i+1],ys[j+1]),(xs[i+1],ys[j])])
            squares.append({'square':square,'center':np.array(square.centroid)})
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


def set_bg(repeaters,not_seen):
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
    if not_seen:
        for square in not_seen:
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
ground_station=np.array(data["ground_station"])
sensor_range=data["sensor_range"]
desired_range=data["desired_range"]

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
not_seen=discretize(min_x,max_x,min_y,max_y,30)

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

    for square in not_seen:
        if norm(traj_pos[time_step]-square['center'])<10:
            seen=True
            for vertex in list(np.array(square['square'].exterior.coords))[:-1]:
                if norm(traj_pos[time_step]-square['center'])>=10:
                    seen=False
            if seen:
                not_seen.remove(square)


    if repeaters:
        if norm(repeaters[-1].position-ground_station)>=desired_range:
            repeaters.append(Repeater(ground_station))
        for r in range(len(repeaters)):
            if r==0:
                repeaters[r].move(np.array((0,0)))#todo: go to the boss
            else:
                a=0#todo: go to the previous repeater


    else:
        if norm(traj_pos[time_step]-ground_station)>=desired_range:
            repeaters.append(Repeater(ground_station))




    time_step += 1

    if time_step%20==0:
        set_bg(repeaters,not_seen)
        pg.display.flip()