import numpy as np
from numpy.linalg import norm
import pygame as pg
import json
import time
from pygame.locals import *
pg_scale = 7

dt = 0.1
white=(255,255,255)
black=(0,0,0)
red=(255,0,0)
green=(0,255,0)
blue=(0,0,255)
orange=(255, 102, 0)
dark_green=(0,153,0)

pg.init()
infoObject = pg.display.Info()
screen = pg.display.set_mode((infoObject.current_w, infoObject.current_h))
width = infoObject.current_w
height = infoObject.current_h
background_colour = (255, 255, 255)


class Drone():

    def __init__(self, pos):

        self.pos = pos
        self.vel = np.zeros(2)
        self.error_prev = np.zeros(2)
        self.u_max = 2
        self.v_max = 2

    def move(self, target):

        u = self.control(target)
        self.vel += u * dt

        if norm(self.vel) > self.v_max:
            self.vel = self.vel / norm(self.vel) * self.v_max

        self.pos += self.vel * dt

    def control(self, target):
        k_p = 10; k_d = 10
        error = target - self.pos
        d_error = (error - self.error_prev) / dt

        self.error_prev = error
        u = k_p * error + k_d * d_error
        if norm(u) > self.u_max:
            u = (u / norm(u)) * self.u_max

        return u

def to_pygame(coords):
    """
    Convert coordinates into pygame coordinates
    """
    return (int(coords[1] * -pg_scale + height / 2 + 600),int(coords[0] * pg_scale + width / 2 - 550))

def list_to_pygame(list_of_points):
    """
    Convert list of points to pygame coordinates
    :param list_of_points: list of points to collect
    :return: list of points in pygame coordinates
    """
    pg_list_of_points=[]
    for point in list_of_points:
        pg_list_of_points.append(to_pygame(point))
    return  pg_list_of_points

def set_bg(screen,main_pos, bounding_polygon, obstacles):

    screen.fill(white)

    ###UNCOMMENT TO PLOT THE TRAJECTORY OF THE MAIN DRONE
    #for i in range(1,len(traj_pos)):
        #pg.draw.line(screen,(0,0,0),to_pygame(traj_pos[i-1]),to_pygame(traj_pos[i]))

    #plot main drone and its sensors
    pg.draw.line(screen, red, to_pygame(main_pos + np.array([1, 1])), to_pygame(main_pos + np.array([-1, -1])), 4)
    pg.draw.line(screen, red, to_pygame(main_pos + np.array([-1, 1])),to_pygame(main_pos + np.array([1, -1])),4)
    [pg.draw.polygon(screen, black, list_to_pygame(obstacle), 0) for obstacle in obstacles]

    #plot cave boundaries
    pg.draw.polygon(screen,black,list_to_pygame(bounding_polygon),1)



def main():

    save_path = "data1.json"

    MOVE_LEFT=np.array([0, 1])
    MOVE_RIGHT = np.array([0,-1])
    MOVE_UP = np.array([-1,0])
    MOVE_DOWN=np.array([1, 0])



    init_pos = np.array([1.0, 1.0])
    drone = Drone(init_pos)


    data = json.load(open('P25_X.json'))
    bounding_polygon = data["bounding_polygon"]
    # obstacles
    obstacles = []
    sh_obstacles = []
    for d in data:
        if "obstacle" in d:
            obstacle = data[d]
            obstacles.append(obstacle)

    time_step = 0
    start = False
    done = False

    t=[]
    x=[]
    y=[]
    tstep=0.0
    direction=np.zeros(2)
    while not done:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                done = True
            if event.type == KEYDOWN:
                if event.key == K_LEFT:
                    direction = MOVE_LEFT
                elif event.key == K_RIGHT:
                    direction = MOVE_RIGHT
                elif event.key == K_UP:
                    direction = MOVE_UP
                elif event.key == K_DOWN:
                    direction = MOVE_DOWN
                elif event.key == K_SPACE:
                    data = {'t': t, 'x': x, 'y': y}
                    with open(save_path, 'w') as outfile:
                        json.dump(data, outfile, indent=2)
                    exit(1)


            elif event.type == KEYUP:
                if event.key == K_LEFT:
                    direction = np.zeros(2)
                elif event.key == K_RIGHT:
                    direction = np.zeros(2)
        if not start:
            t0 = time.time()
            t1 = 0
            while (t1 - t0 < 1):
                t1 = time.time()
            start = True

        drone.move(drone.pos+direction)
        tstep+=dt
        x.append(drone.pos[0])
        y.append(drone.pos[1])
        t.append(tstep)
        set_bg(screen,drone.pos,bounding_polygon, obstacles)
        pg.display.flip()



if __name__ == '__main__':
    main()
