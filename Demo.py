import pygame
import math
import numpy as np
import time
class Enviroment(object):
    def __init__(self, dimensions: tuple, mapImg):
        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.orange = (255, 165, 0)
        self.yellow = (255, 255, 0)
        
        # map dims
        self.height = dimensions[0]
        self.width = dimensions[1]
        
        
        # window settings
        pygame.display.set_caption("Simulator Robot")
        self.map = pygame.display.set_mode((self.width, self.height))
        
        # font settings
        pygame.font.init()  # Initialize the font module
        self.font = pygame.font.SysFont('Arial', 25)
        self.text = None
        self.textRect = None
        
        # trail settings
        self.trailSet = []
        
        self.add_goal = None
    
    def robot_fame(self, pos, rotation):
        n = 80
        cx, cy = pos
        x_axis = (cx + n * math.cos(rotation), cy + n * math.sin(rotation))
        y_axis = (cx + n * math.cos(rotation + math.pi / 2), cy + n * math.sin(rotation + math.pi / 2))
        pygame.draw.line(self.map, self.red, pos, x_axis, 3)
        pygame.draw.line(self.map, self.green, pos, y_axis, 3)
        
    def write_info(self, vl, vr, theta):
        txt = f'vl = {vl:.2f} | vr = {vr:.2f} | theta = {int(math.degrees(theta))}'
        self.text = self.font.render(txt, True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.map.blit(self.text, self.textRect)
        
    def trail(self, pos):
        for i in range(0, len(self.trailSet) - 1):
            pygame.draw.line(self.map, self.orange, (self.trailSet[i][0], self.trailSet[i][1]), (self.trailSet[i + 1][0], self.trailSet[i + 1][1]))
        if self.trailSet.__sizeof__() > 100000:
            self.trailSet.pop(0)
        self.trailSet.append(pos)
        
    def sensor_info(self, sensor_data):
        txt = f'Sensor: {sensor_data}'
        self.text = self.font.render(txt, True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.map.blit(self.text, self.textRect)
    
    def Goal(self):
        txt = f'Đa đen đich'
        self.text = self.font.render(txt, True, self.red, self.white)
        self.textRect = self.text.get_rect()
        self.map.blit(self.text, self.textRect)    
    def robot_sensor_data(self,pos, sensor_data):
        for point  in sensor_data:    
            pygame.draw.line(self.map, self.blue, pos, point, 1)
            pygame.draw.circle(self.map, self.blue, point, 5)
    
    
    
    def draw_map(self):
        self.map.blit(self.mapImg, (0, 0))
    
# robot class
class Robot:
    def __init__(self, start_pos, robot_img, width):
        self.m2p = 3779.527559 # meter to pixel
        # robot dims
        self.sp = 0
        self.w = width
        self.x = start_pos[0]
        self.y = start_pos[1]
        self.theta = -np.pi / 2
        self.vl = self.sp
        self.vr = -self.sp
        self.max_speed = 200
        self.min_speed = -200
        #robot Values
        self.r = 8
        self.L = 3
        
        # robot velocity
        self.vx = 0
        self.vy = 0
        self.vtheta = 0
        
        
        
        # robot image
        self.img = pygame.image.load(robot_img)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.img = pygame.image.load(robot_img)
        # self.img = pygame.transform.scale(self.img, (50, 50))
        self.img = pygame.transform.scale(self.img, (75, 75)) #map6
        
        # time
        self.dt = 0
        self.lastTime = pygame.time.get_ticks()
        
    
        self.sensor = [0, 0, 0, 0, 0, 0]
        self.edges = []

        self.theta_target = 0
    def draw(self, map):
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(-self.theta), 1)
        map.blit(self.rotated, self.rect)
        
    def handle_keys(self, event):
        if event:
            if event.type == pygame.KEYDOWN:
                # change vl
                if event.key == pygame.K_1:
                    self.vl += 0.0001 * self.m2p
                if event.key == pygame.K_2:
                    self.vl -= 0.0001 * self.m2p
                # change vr
                if event.key == pygame.K_3:
                    self.vr += 0.0001 * self.m2p
                if event.key == pygame.K_4:
                    self.vr -= 0.0001 * self.m2p
    
    def move(self):
        self.vr = min(self.vr, self.max_speed)
        self.vr = max(self.vr, self.min_speed)
        self.vl = min(self.vl, self.max_speed)
        self.vl = max(self.vl, self.min_speed)
        self.dt = (pygame.time.get_ticks() - self.lastTime) / 1000
        self.lastTime = pygame.time.get_ticks()
        
        #kinematic model
        R = np.array([
            [np.cos(self.theta), np.sin(self.theta), 0],
            [-np.sin(self.theta), np.cos(self.theta), 0],
            [0, 0, 1]
        ])
        
        j1f = np.array([[1, 0, -self.L],    
                        [-1, 0, -self.L]])
        j2 = np.array([[self.r, 0],
                       [0, self.r]])
        
        vv = np.linalg.inv(R) @ np.linalg.pinv(j1f) @ j2 @ np.array([[self.vl], [self.vr]])
        vx, vy, omega = vv.flatten()
        
        new_x = self.x + vx * self.dt
        new_y = self.y + vy * self.dt
        new_theta = self.theta + omega * self.dt
        #limit theta
        new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi
        
        
        #update position
        self.x = new_x
        self.y = new_y
        self.theta = new_theta 
        
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
    
    def sensor_data(self, map):
        angles = [
            self.theta,
            3*np.pi/2 + self.theta, #trai
            np.pi/2 + self.theta,   #phai
            5*np.pi/3 + self.theta, #XTRai
            np.pi/3 + self.theta,   #XPhai
            np.pi/2 + self.theta
        ]    
        
        edges = []
        distances = []
        
        for angle in angles:
            distance = 0
            x = int(self.x)
            y = int(self.y)
            while True:
                x = int(self.x + distance * np.cos(angle))
                y = int(self.y +distance * np.sin(angle))
                distance += 1
                
                if x < 0 or x >= 1200 or y < 0 or y >= 800:
                    break
                b,g,r,a = map.get_at((x, y))
                if r < 110 and g < 110 and b < 110:
                    break
            edges.append((x, y))
            distances.append(distance)
        self.sensor = distances
        self.edges = edges

def emumerate(iterable):
    count = 0
    for item in iterable:
        yield count, item
        count += 1
   
        
  
                        
    

            
if __name__ == "__main__":        
    # initialization
    pygame.init()
    
    my_map = pygame.image.load(r"maps/map-0.png")
    

    # start position
    
    
    # start = [300, 50] #5x5
    # start = [500, 750] #6x6
    
    # start = [450, 750]# Bám Phải map 6
    
    #-----------------------Bot
    # start = [350, 780] #5x5  map8  Bot
    # END = [500,80]
    
    #-----------------------Bot
    start = [450, 750] #5x5  map8  Bot
    END = [600,80]
    # END = [350,80]
   
    # END = [500,80]
    # END = [500, 450]
    
    
    
    # -----------------------Top
    # start = [400, 50] #5x5 TOP
    # END = [550, 780]   
    
    
    
    # dimensions
    dims = [800, 900]

    cnt_distance = +10
    target_distance = 40 + cnt_distance
    target_distance_font = 65 + cnt_distance
    
    # target_distance = 25
    # target_distance_font = 25
    
    # target_distance = 65
    # the environment
    environment = Enviroment(dims,my_map)

    # the robot
    robot = Robot(start, r"RBF1.png", 0.01)
    
    number_of_robot = 20
    Robots = [] # Quần thể robot
    
    for i in range(number_of_robot):
        robot = Robot(start, r"RBF1.png", 0.01)
        Robots.append(robot)

    # simulation loop
    running = True

    # cnt = 35
    cnt = 5
    
    # mô phỏng thêm 20 robot
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            robot.handle_keys(event)    

        pygame.display.update
        # environment.map.fill(environment.white)
        environment.map.blit(my_map, (0, 0))
        
    
        robot.move()
        robot.sensor_data(environment.map.copy())
        environment.robot_fame((robot.x, robot.y), robot.theta)
        
        right_sensor = robot.sensor[2]
        left_sensor = robot.sensor[1]
        front_sensor = robot.sensor[0]
        X_left_sensor = robot.sensor[3]
        X_right_sensor = robot.sensor[4]
        
        # #Trai
        # if front_sensor < target_distance_font and left_sensor > target_distance :
        #     robot.vl = 5 +cnt
        #     robot.vr = 0
        # elif front_sensor > target_distance_font and right_sensor < target_distance or X_right_sensor  < target_distance:
        #     robot.vl = 5+cnt
        #     robot.vr = -0
         
        # else:
        #     robot.vl = 2+cnt
        #     robot.vr = -5-cnt
        # #     robot.vl =8+cnt
        # #     robot.vr = -25-cnt
        
        #Phai
        if front_sensor < target_distance_font and right_sensor > target_distance :
            robot.vl = 0
            robot.vr = -5-cnt
        elif front_sensor > target_distance_font and left_sensor < target_distance or X_left_sensor < target_distance:
            robot.vl = 0
            robot.vr = -5-cnt
        else:
            robot.vl = 5+cnt
            robot.vr = -2-cnt
            # robot.vl =65+cnt
            # robot.vr = -32-cnt
       
        
                
        robot.draw(environment.map)
        environment.trail((robot.x, robot.y))
        # environment.write_info(robot.vl, robot.vr, robot.theta)
        environment.robot_sensor_data((robot.x, robot.y), robot.edges)
        
        environment.sensor_info(robot.sensor)
        
        if robot.x > 400 and  robot.x <= END[0] and robot.y <= END[1]:
            robot.vl = 0
            robot.vr = 0
            environment.Goal()
            
            # robot.move()
        pygame.display.flip()
        
        
        