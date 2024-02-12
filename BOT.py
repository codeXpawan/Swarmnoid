from astar import astar
import numpy as np
import math
import paho.mqtt.client as mqtt

class BOT():
    
    ARENA_WIDTH = 300
    ARENA_HEIGHT = 300
    GRID_SIZE = 2  # Adjust based on your setup
    robot_id = None
    target_waste_ids = None
    position = set()
    centre_position = set()
    targeted_waste_position = set()
    targeted_waste_id = None
    obstacles = set()
    path = None
    KP = 0.2
    KI = 0.2
    KD = 0.01
    pre_error_x = 0
    pre_error_y = 0
    dt = 0.3
    MQTT_BROKER = "192.168.1.172"
    MQTT_PORT = 1883
    client = mqtt.Client()
    
    def __init__(self,robot_id, position,target_waste_ids):
        self.position = position
        self.robot_id = robot_id
        self.target_waste_ids = target_waste_ids
        self.connect_mqtt()
        pass  
    
    def connect_mqtt(self):
        self.client.connect(self.MQTT_BROKER, self.MQTT_PORT, 60)
        self.client.loop_start()
    
    def plan_path(self,path=None):
        """Wrapper for the A* pathfinding."""
        # print(self.position)
        start_grid = self.convert_to_grid_coordinates(self.position)
        goal_grid = self.convert_to_grid_coordinates(self.targeted_waste_position)
        path_obstacles = set()
        obstacles = self.convert_obstacles_to_grid()
        if path:
            path_obstacles = self.convert_path_to_grid()
        all_obstacles = obstacles | path_obstacles
        # Assuming your grid size is the width/height of the arena divided by GRID_SIZE
        grid_size = self.ARENA_WIDTH // self.GRID_SIZE

        self.path = astar(start_grid, goal_grid, all_obstacles, grid_size)
        # return path  # This will be a list of grid coordinates representing the path

    def convert_path_to_grid(obstacles,cell_size=15):
        grid_obstacles = set()
        for position in obstacles:
            if not isinstance(position, tuple) or len(position) != 2:
                raise ValueError("Each position must be a tuple of (x, y).")
            grid_x = int(position[0] / cell_size)
            grid_y = int(position[1] / cell_size)
            grid_obstacles.add((grid_x, grid_y))
        return grid_obstacles
    
    def convert_to_grid_coordinates(self,position, cell_size=15):
        """Converts position to grid coordinates."""
        # print(position)
        # print("3")
        if not isinstance(position, tuple) or len(position) != 2:
            raise ValueError("Position must be a tuple of (x, y).")
        grid_x = int(position[0] / cell_size)
        grid_y = int(position[1] / cell_size)
        return (grid_x, grid_y)
    
    def convert_obstacles_to_grid(self, cell_size=15):
        """Converts a set of positions to grid coordinates."""
        grid_obstacles = set()
        for position in self.obstacles:
            if not isinstance(position, tuple) or len(position) != 2:
                raise ValueError("Each position must be a tuple of (x, y).")
            grid_x = int(position[0] / cell_size)
            grid_y = int(position[1] / cell_size)
            grid_obstacles.add((grid_x, grid_y))
        return grid_obstacles
    
    def update_position(self,position):
        self.position = position
        
    def update_centre_position(self,position):
        self.centre_position = position
        
    def update_target_waste_ids(self,target_waste_ids):
        self.target_waste_ids = target_waste_ids
    
    def find_nearest_waste(self,markers):
        nearest_waste_dist = float("inf")
        for marker_id, marker_data_list in markers.items():
            if (marker_id in self.target_waste_ids):
                for marker_data in marker_data_list:
                    corners = marker_data["corners"]
                    tl, tr = corners[0], corners[1]
                    head_center = ((tl[0] + tr[0]) / 2, (tl[1] + tr[1]) / 2)
                    self.obstacles.add(head_center)
                    distance = np.linalg.norm(
                        np.array(self.position) - np.array(head_center)
                    )
                    if distance < nearest_waste_dist:
                        # nearest_waste_id = marker_id
                        midpoints = [
                                        (
                                            (corners[0][0] + corners[1][0]) / 2,
                                            (corners[0][1] + corners[1][1]) / 2,
                                        ),  # Top edge
                                        (
                                            (corners[1][0] + corners[2][0]) / 2,
                                            (corners[1][1] + corners[2][1]) / 2,
                                        ),  # Right edge
                                        (
                                            (corners[2][0] + corners[3][0]) / 2,
                                            (corners[2][1] + corners[3][1]) / 2,
                                        ),  # Bottom edge
                                        (
                                            (corners[3][0] + corners[0][0]) / 2,
                                            (corners[3][1] + corners[0][1]) / 2,
                                        ),  # Left edge
                                    ]
                        min_distance = float("inf")
                        for midpoint in enumerate(midpoints):
                            midpoint = midpoint[1]
                            # print(midpoint)
                            # distance = math.hypot(
                            #     midpoint[0] - self.position[0], midpoint[1] - self.position[1]
                            # )
                            distance = np.linalg.norm(
                                np.array(self.position) - np.array(midpoint)
                            )
                            if distance < min_distance:
                                # min_distance = distance
                                self.targeted_waste_position = midpoint
                                self.targeted_waste_id = marker_id
        if self.targeted_waste_position:
            self.obstacles.discard(self.targeted_waste_position)
        
        return self.targeted_waste_id
                        

    def send_mqtt_command(self,topic, command):
        self.client.publish(topic, command)
    
    def move_towards_goal(self):
        for next_position in self.path:
            position_reached = False
            while not position_reached:
                error_x = next_position[0] - self.position[0]
                error_y = next_position[1] - self.position[1]
                
                if abs(error_x) > abs(next_position[0] - self.centre_position[0]):
                    self.send_mqtt_command(f"/robot{self.robot_id}_y", 0.2)
                elif abs(error_y) > abs(next_position[1] - self.centre_position[1]):
                    self.send_mqtt_command(f"/robot{self.robot_id}_y", 0.2)
                else:
                    if error_x < 5 and error_y > 5:
                        error_x = error_y
                        error_y = 0
                    # Compute PID output
                    output_x = self.KP * error_x + self.KI * (error_x+self.pre_error_x)*self.dt + (self.KD * (error_x - self.pre_error_x))/self.dt
                    output_y = self.KP * error_y + self.KI * (error_y+self.pre_error_y)*self.dt + (self.KD * (error_y - self.pre_error_y))/self.dt
                    
                    self.pre_error_x = error_x
                    self.pre_error_y = error_y
                    
                    self.send_mqtt_command(f"/robot{self.robot_id}_x", output_x)
                    self.send_mqtt_command(f"/robot{self.robot_id}_y", output_y)
                if (
                    self.position
                    and math.hypot(
                        self.position[0] - next_position[0],
                        self.position[1] - next_position[1],
                    )
                    < 30
                ):
                    position_reached = True