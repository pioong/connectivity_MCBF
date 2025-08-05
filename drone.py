import pygame
import math

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)

class Drone:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.is_marked = False
        self.neighbors = []
        self.received_packets = []
        self.color = BLUE

    def update_position(self, ux, uy):
        """Update drone position based on control inputs ux and uy."""
        dt = 0.5
        self.x += dt*ux
        self.y += dt*uy

    def move_towards(self, target):
        # Move one step toward the goal to simplify
        step_size = 5  # Adjust the step size as needed.
        if self.x < target.x:
            self.x += min(step_size, target.x - self.x)
        elif self.x > target.x:
            self.x -= min(step_size, self.x - target.x)
        if self.y < target.y:
            self.y += min(step_size, target.y - self.y)
        elif self.y > target.y:
            self.y -= min(step_size, self.y - target.y)

    def update_position1(self):
        # Implement logic to decide where to move.
        # For example, move toward the center of mass of the neighbors.
        if self.neighbors:
            avg_x = sum(drone.x for drone in self.neighbors) / len(self.neighbors)
            avg_y = sum(drone.y for drone in self.neighbors) / len(self.neighbors)
            self.move_towards(Drone(avg_x, avg_y))

    def draw(self, screen, DRONE_SIZE, color):
        pygame.draw.circle(screen, color, (self.x, self.y), DRONE_SIZE)

    def move_to(self, new_x, new_y):
        self.x = new_x
        self.y = new_y

    def distance_to(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def update_neighbors(self, all_drones, communication_range):
        """Update the list of neighbors based on a communication range."""
        self.neighbors = [drone for drone in all_drones if drone != self and self.distance_to(drone) <= communication_range]

        return self.neighbors

    def marking_process(self, neighbors, COMM_RANGE):
        """ Marks the drone as part of the E-CDS based on specific criteria """
        marker = False

        if self.is_gateway == True:
            marker = True
            return marker

        for i in range(len(neighbors)):
            for j in range(i + 1, len(neighbors)):
                if neighbors[i].distance_to(neighbors[j]) > COMM_RANGE:
                    marker = True

        return marker
    
    def can_reach(self, target, exclude, marked_drones, visited=None):
        """
        Determines whether there is a path from the current drone (self) to a specified target drone (target)
        while avoiding a particular drone (exclude) and considering only drones that are in a specified list 
        of marked drones (marked_drones)
        """
        if visited is None:
            visited = set()
        if self == target:
            return True
        visited.add(self)
        for neighbor in self.neighbors:
            if neighbor != exclude and neighbor in marked_drones and neighbor not in visited:
                if neighbor.can_reach(target, exclude, marked_drones, visited):
                    return True
        return False
    
    def can_be_pruned(self, drone, marked_drones):
        """
        Determine if a drone can be pruned. A drone can be pruned if at least
        two of its marked neighbors can connect directly or through other marked neighbors.
        """
        if self.is_gateway == True:
            return False
        
        for neighbor in self.neighbors:
            if len(neighbor.neighbors) <= 1:
                return False
            if neighbor in marked_drones and neighbor != drone:
                # Check if this neighbor can still reach other neighbors without this drone
                other_marked_neighbors = []
                for n in neighbor.neighbors:
                    if n != drone and n in marked_drones:
                        other_marked_neighbors.append(n)

                reachable = False
                for other in other_marked_neighbors:
                    if neighbor.can_reach(other, drone, marked_drones):
                        reachable = True
                    else:
                        reachable = False
                        break

                if not reachable:
                    return False
            else:
                return False
    
        return True

    

    
    
