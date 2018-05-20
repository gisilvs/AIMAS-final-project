import pygame as pg
import numpy as np

#plotting_settings
pg_scale=7

#colors
white=(255,255,255)
black=(0,0,0)
red=(255,0,0)
green=(0,255,0)
blue=(0,0,255)
purple=(76,0,153)
dark_green=(0,153,0)


class Plotter():
    def __init__(self):
        pg.init()
        self.infoObject = pg.display.Info()
        self.screen = pg.display.set_mode((self.infoObject.current_w, self.infoObject.current_h))
        self.width = self.infoObject.current_w
        self.height = self.infoObject.current_h
        self.background_colour = (255, 255, 255)


    def list_to_pygame(self,list_of_points):
        """
        Convert list of points to pygame coordinates
        :param list_of_points: list of points to collect
        :return: list of points in pygame coordinates
        """
        pg_list_of_points=[]
        for point in list_of_points:
            pg_list_of_points.append(self.to_pygame(point))
        return  pg_list_of_points

    def to_pygame(self,coords):
        """
        Convert coordinates into pygame coordinates
        """
        return (int(coords[1] * -pg_scale + self.height / 2 + 600),int(coords[0] * pg_scale + self.width / 2 - 550))

    def set_bg(self,repeaters,squares,main_pos,bounding_lines,obstacles,sensor_range,scanning_range,ground_station,desired_range):
        """
        Set various things to plot on screen
        :param repeaters: list of repeaters to plot
        :param squares: unseen discretized space
        :param main_pos: position of the main drone
        :param bounding_lines: plotting of the boundary with mine entrance
        :param obstacles: obstacles to plot
        :param sensor_range: maxmimum line of sight for one drone
        :param scanning_range: range of 'exploration' for the main drone
        :param ground_station: position of the ground station
        :param desired_range: range where to be to make sure to be in LOS
        :return:
        """

        self.screen.fill(white)

        ###UNCOMMENT TO PLOT THE TRAJECTORY OF THE MAIN DRONE
        #for i in range(1,len(traj_pos)):
            #pg.draw.line(screen,(0,0,0),to_pygame(traj_pos[i-1]),to_pygame(traj_pos[i]))

        #plot obstacles
        [pg.draw.polygon(self.screen, black, self.list_to_pygame(obstacle), 0) for obstacle in obstacles]

        #plot main drone and its sensors
        pg.draw.line(self.screen, red, self.to_pygame(main_pos + np.array([1, 1])), self.to_pygame(main_pos + np.array([-1, -1])), 4)
        pg.draw.line(self.screen, red, self.to_pygame(main_pos + np.array([-1, 1])),self.to_pygame(main_pos + np.array([1, -1])),4)

        #sensor range in red
        pg.draw.circle(self.screen, red, self.to_pygame(main_pos), sensor_range*pg_scale, 1)

        #scanning range in purple
        pg.draw.circle(self.screen,purple,self.to_pygame(main_pos),scanning_range*pg_scale,1)

        #plot ground station todo convert to ellipse
        pg.draw.circle(self.screen,dark_green,self.to_pygame(ground_station),5)
        pg.draw.circle(self.screen,dark_green,self.to_pygame(ground_station),sensor_range*pg_scale,1)
        pg.draw.circle(self.screen, green, self.to_pygame(ground_station), desired_range * pg_scale, 1)

        #plot repeaters and their sensor range
        if repeaters:
            for repeater in repeaters:
                pg.draw.line(self.screen, blue, self.to_pygame(repeater.position + np.array([0.8, 0.8])),
                             self.to_pygame(repeater.position+ np.array([-0.8, -0.8])), 3)
                pg.draw.line(self.screen, blue, self.to_pygame(repeater.position + np.array([-0.8, 0.8])),
                             self.to_pygame(repeater.position + np.array([0.8, -0.8])), 3)
                pg.draw.circle(self.screen, blue, self.to_pygame(repeater.position), sensor_range*pg_scale, 1)

        #plot unseen areas
        s=pg.Surface((self.infoObject.current_w, self.infoObject.current_h),pg.SRCALPHA)
        for i in range(squares.shape[0]):
            for j in range(squares.shape[1]):
                square = squares[i, j]
                if not square['seen']:
                    pg.draw.polygon(s,(100,100,100,128), self.list_to_pygame(square['vertices'])) #color is grey with transparency 128

        #plot cave boundaries
        for i in range(1,len(bounding_lines)):
            pg.draw.line(self.screen,black,self.to_pygame(bounding_lines[i-1]),self.to_pygame(bounding_lines[i]))
            self.screen.blit(s, (0, 0))


