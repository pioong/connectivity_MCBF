import pygame
import sys
import math
import PicDraw
from drone import Drone
from MCBF import CBF_SDP
import matplotlib.pyplot as plt
import numpy as np

time_data = []
u_x_data = []
u_y_data = []
eig_data = [] 

def plot_control(time_data,u_x_data,u_y_data):
    u_x_data = np.array(u_x_data)
    u_y_data = np.array(u_y_data)
    plt.subplot(2,1,1)
    for i in range(simulation.DRONE_COUNT):
        plt.plot(time_data, u_x_data[:, i], label=f'Drone {i}')
    plt.xlabel("Time [ticks]")
    plt.ylabel(r'$u_1$')
    plt.title(r'$u_1$ over time')
    plt.grid(True)
    plt.legend()

    plt.subplot(2,1,2)
    for i in range(simulation.DRONE_COUNT):
        plt.plot(time_data, u_y_data[:, i], label=f'Drone {i}')
    plt.xlabel("Time [ticks]")
    plt.ylabel(r'$u_2$')
    plt.title(r'$u_2$ over time')
    plt.grid(True)
    plt.legend()   

def plot_eigenvalues(time_data,eig_data):
    eig_data = np.array(eig_data)
    for i in range(simulation.DRONE_COUNT):
        plt.plot(time_data, eig_data[:, i], label=rf'$\phi_{i+1}$')
    plt.xlabel(r"Time [ticks]")
    plt.ylabel(r"$\phi_i(L(x))$")
    plt.title(r"Eigenvalues over time")
    plt.grid(True)
    plt.legend()  

class PygameSimulation():
    def __init__(self):
        # Initialize Pygame
        pygame.init()

        # Constants
        self.MAS_WIDTH, self.MAS_HEIGHT = 800, 600
        self.DRONE_SIZE = 10
        self.DRONE_COUNT = 6
        self.COMM_RANGE = 200.0  # Communication range in pixels
        self.MIN_DISTANCE_BETWEEN_DRONES = 30

        # Colors
        self.WHITE = (255, 255, 255)
        self.BLUE = (0, 0, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.YELLOW = (255, 255, 0)

        # Set up the display
        self.DRAW_WIDTH = 200
        self.screen = pygame.display.set_mode((self.MAS_WIDTH, self.MAS_HEIGHT))
        pygame.display.set_caption("Drone Swarm Simulation")

    def create_drones(self):
        drones = []
        drone_id = 0
        drone_init_position = 300.0*np.ones((6,2))
        factor = 0.5
        drone_init_position[0,:] += factor*self.COMM_RANGE*np.array([0.0,0.0])
        drone_init_position[1,:] += factor*self.COMM_RANGE*np.array([-0.707,-0.707])
        drone_init_position[2,:] += factor*self.COMM_RANGE*np.array([0.707,-0.707])
        drone_init_position[3,:] += factor*self.COMM_RANGE*np.array([-0.707,0.707])
        drone_init_position[4,:] += factor*self.COMM_RANGE*np.array([-1.707,-0.707])
        drone_init_position[5,:] += factor*self.COMM_RANGE*np.array([-1.707,0.707])
        for _ in range(self.DRONE_COUNT):
            drone = Drone(drone_id, drone_init_position[_,0], drone_init_position[_,1])
            drones.append(drone)
            drone_id = drone_id + 1

        return drones

    def draw_connections(self, drones):
        # Draw connections
        for i in range(len(drones)):
            for j in range(i + 1, len(drones)):
                if drones[i].distance_to(drones[j]) < self.COMM_RANGE:
                    pygame.draw.line(self.screen, self.GREEN, (drones[i].x, drones[i].y), (drones[j].x, drones[j].y))

    def run_simulation(self, drones):
        running = True

        # Initialize the drones
        actual_positions = [(drone.x, drone.y) for drone in drones]
        
        self.cbf_sdp = CBF_SDP(self.COMM_RANGE,self.DRONE_COUNT,self.DRONE_SIZE)
        closest_drone = drones[0]

        # Initialize the PicDraw class
        pic = PicDraw.PicDraw(self.screen, self.MAS_WIDTH, self.MAS_HEIGHT, self.DRAW_WIDTH, tracking_time_length=1000)
        # Initialize the destination vector
        destination_vector = actual_positions



        while running:
            self.screen.fill(self.WHITE)

            actual_positions = [(drone.x, drone.y) for drone in drones]

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    if pos[0] <= self.MAS_WIDTH:
                        closest_drone = min(drones, key=lambda drone: math.sqrt((drone.x - pos[0]) ** 2 + (drone.y - pos[1]) ** 2))
                        destination_vector[closest_drone.id] = (pos[0],pos[1])
            
            u_des = self.cbf_sdp.compute_desired_velocities(actual_positions, destination_vector)

            u = self.cbf_sdp.solve_sdp(actual_positions, u_des, closest_drone.id)


            for i, (drone, (ux, uy)) in enumerate(zip(drones, u)):
                drone.update_position(ux, uy)
                if i == closest_drone.id:
                    drone.draw(self.screen, self.DRONE_SIZE, self.RED)
                else:
                    drone.draw(self.screen, self.DRONE_SIZE, self.BLUE)

            self.draw_connections(drones)

            # Draw the MAS area
            pic.draw_MAS_area()

            if pygame.time.get_ticks() < 20000:
                time_data.append(pygame.time.get_ticks())
                u_x_data.append(u[:,0])
                u_y_data.append(u[:,1])
                eig_data.append(self.cbf_sdp.lambdas.value)
            pygame.display.flip()

if __name__ == "__main__":
    simulation = PygameSimulation()

    drones = simulation.create_drones()

    simulation.run_simulation(drones)

    pygame.quit()
    plt.figure(1)
    plot_control(time_data,u_x_data,u_y_data)
    plt.figure(2)
    plot_eigenvalues(time_data,eig_data)
    plt.show()
    sys.exit()


