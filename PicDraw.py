import pygame
import numpy as np

class PicDraw:
    def __init__(self, screen, MAS_WIDTH, MAS_HEIGHT, DRAW_WIDTH, tracking_time_length:int=1000):
        self.screen = screen
        self.MAS_WIDTH = MAS_WIDTH
        self.MAS_HEIGHT = MAS_HEIGHT
        self.DRAW_WIDTH = DRAW_WIDTH
        self.tracking_time_length = tracking_time_length
        self.tracking_time = np.arange(-tracking_time_length,0).reshape(-1,1)
        # self.tracking_time = np.zeros((tracking_time_length, 1))
        self.tracking_signal = np.zeros((self.tracking_time_length, 1))
        self.on_canvas_time = np.linspace(self.MAS_WIDTH, self.MAS_WIDTH + self.DRAW_WIDTH,tracking_time_length).reshape(-1,1)
        self.on_canvas_signal = self.MAS_HEIGHT + self.tracking_signal

    def draw_MAS_area(self):
        # Draw the MAS area
        pygame.draw.rect(self.screen, (255, 0, 0), (0, 0, self.MAS_WIDTH, self.MAS_HEIGHT), 2)

    def data_screen_mapping(self, data, axis = 0):
        # mapping for the x-axis
        if axis == 0:
            return np.interp(data, [data[0],data[-1]], [self.MAS_WIDTH, self.MAS_WIDTH + self.DRAW_WIDTH])
        else:
            result = np.interp(data, [-0.8,0.8], [self.MAS_HEIGHT,0])
            return result