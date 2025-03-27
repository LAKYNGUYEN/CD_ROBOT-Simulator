import pygame, math, numpy as np

class Noron(object):
    def __init__(self):
        pass
    def function(self, x):
        return np.tanh(x)
    def NN(self, x, w, v):
        net_h = w.T @ x
        y_h = self.function(net_h)
        return v.T @ y_h

class Maze(object):
    def __init__(self, image: str):
        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.orange = (255, 165, 0)
        self.yellow = (255, 255, 0)
        # Load image
        self.image = pygame.image.load(image)
        # Map dimensions
        self.height = self.image.get_height()
        self.width = self.image.get_width()
        # Window settings
        pygame.display.set_caption("Simulator Robot")
        self.map = pygame.display.set_mode((self.width, self.height))
        # font
        pygame.font.init()  # Initialize the font module
        self.font = pygame.font.SysFont('Arial', 25)
        self.text = None
        self.textRect = None
    def frame(self, pos, rotation):
        n = 80
        cx, cy = pos
        x_axis = (cx + n * math.cos(rotation), cy + n * math.sin(rotation))
        y_axis = (cx + n * math.cos(rotation + math.pi / 2), cy + n * math.sin(rotation + math.pi / 2))
        pygame.draw.line(self.map, self.red, (cx, cy), x_axis, 3)
        pygame.draw.line(self.map, self.green, (cx, cy), y_axis, 3)
    def sensor_info(self, data):
        txt = f'Sensor: {data}'
        self.text = self.font.render(txt, True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.map.blit(self.text, self.textRect)
    def sensor_data(self, position, data):
        for point  in data:    
            pygame.draw.line(self.map, self.blue, position, point, 1)
            pygame.draw.circle(self.map, self.blue, point, 5)
    def draw(self):
        self.map.blit(self.image, (0, 0))

class Robot(object):
    def __init__(self, x, y, theta, image):
        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.orange = (255, 165, 0)
        self.yellow = (255, 255, 0)
        # 
        self.m2p = 3779.527559
        self.x = x
        self.y = y
        self.theta = theta  # Góc quay (radian)
        self.v1 = 0.5 * self.m2p
        self.v2 = -0.5 * self.m2p
        self.speed = [-self.m2p * 1e-2, self.m2p * 1e-2] # min - max 
        self.r = 10  # Bán kính bánh xe
        self.l = 25  # Khoảng cách giữa 2 bánh xe
        self.alpha = [math.pi / 2, math.pi / 2]
        self.beta = [0, 0]
        self.gamma = [0, 0]
        # Load hình ảnh robot
        self.img = pygame.image.load(image)
        self.img = pygame.transform.scale(self.img, (50, 50))
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(x, y))
        self.dataTime = 0
        self.lastTime = pygame.time.get_ticks()
        # 
        self.sensor = [0, 0, 0, 0, 0]
        self.edges = []
        # crash
        self.crash = False
        self.costfunction = 0
        self.IT = 0
        self.last_position = (self.x, self.y)
        self.total_distance = 0 
        self.frames_count = 0 
    def move(self):
        self.v1 = min(self.v1, self.speed[1])
        self.v2 = min(self.v2, self.speed[1])
        self.v1 = max(self.v1, self.speed[0])
        self.v2 = max(self.v2, self.speed[0])
        # Cập nhật thời gian
        self.dataTime = (pygame.time.get_ticks() - self.lastTime) / 1000
        self.lastTime = pygame.time.get_ticks()
        # Ma trận quay R
        R = np.array([
                [np.cos(self.theta), np.sin(self.theta), 0], 
                [-np.sin(self.theta), np.cos(self.theta), 0], 
                [0, 0, 1]])
        j1f = np.array([[1, 0, -self.l], 
                        [-1, 0, -self.l]])
        j2 = np.array([[self.r, 0], 
                    [0, self.r]])
        vv = np.linalg.inv(R) @ np.linalg.pinv(j1f) @ j2 @ np.array([[self.v1], [self.v2]])
        vx, vy, omega = vv.flatten()
        new_x = self.x + vx * self.dataTime
        new_y = self.y + vy * self.dataTime
        new_theta = self.theta + omega * self.dataTime
        # limit theta
        new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi
        # Cập nhật hình ảnh robot
        self.x = new_x
        self.y = new_y
        self.theta = new_theta
        self.rotated = pygame.transform.rotozoom(self.img, -math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center = (self.x, self.y))
    def sensors(self, map, maze: Maze):
        angles = [
            self.theta,
            3*np.pi/2 + self.theta, #trai
            np.pi/2 + self.theta,   #phai
            5*np.pi/3 + self.theta, #XTRai
            np.pi/3 + self.theta   #XPhai
        ]    
        edges = []
        distances = []
        for angle in angles:
            distance = 0
            step = 0.5
            x = int(self.x)
            y = int(self.y)
            while True:
                x = int(self.x + distance * np.cos(angle))
                y = int(self.y +distance * np.sin(angle))
                distance += step
                if x < 0 or x >= maze.width or y < 0 or y >= maze.height:
                    break
                if x < 0 or x >= 800 or y < 0 or y >= 900:
                    break
                r, g, b, a = map.get_at((x, y))
                if (r, g, b) == (0, 0, 0):
                    break
                if (r + g + b) < 10:
                    break
                if r < 20 and g < 20 and b < 20:
                    break
                if r < 10 and g < 10 and b < 10:
                    break
            edges.append((x, y))
            distances.append(distance)
        self.sensor = distances
        self.edges = edges
    def draw(self, screen):
        screen.blit(self.rotated, self.rect)
    def draw_circle(self, screen):
        pygame.draw.circle(screen, self.green, self.rect.center, 35, 2)
    def check(self, screen):
        for angle in np.linspace(0, 2 * np.pi, 36):
            x = int(self.x + 35 * np.cos(angle))
            y = int(self.y + 35 * np.sin(angle))
            if 0 <= x < screen.get_width() and 0 <= y < screen.get_height():
                r, g, b, a = screen.get_at((x, y))
                if r < 20 and g < 20 and b < 20: 
                    self.crash = True
                    return True
        self.crash = False
        return False


if __name__ == "__main__":
    pygame.init()
    maze = Maze("maps/map-0.png")
    noron = Noron()
    running = True
    numbers = 10
    robots = []
    SPEED = 2.5  # Speed of the robot
    
    # PSO parameters
    pop_size = numbers
    npar = 55
    min_max = [-5, 5]
    w_pso = 0.9
    c1_pso = 0.5
    c2_pso = 0.5
    max_iteration = 30
    current_iteration = 0
    
    
    # Initialize PSO variables
    Pbest_fitness = np.full(pop_size, np.inf)
    Gbest_fitness = np.inf
    Pbest_position = np.zeros((pop_size, npar))
    Gbest_position = np.zeros(npar)
    fitness_values = np.zeros(max_iteration)
    fitness_mean = np.zeros(max_iteration)
    P_pso = np.random.uniform(min_max[0], min_max[1], (pop_size, npar))
    V_pso = np.zeros((pop_size, npar))
    
    # Create initial robots
    for _ in range(numbers):
        robots.append(Robot(400, 90, math.pi / 2, "E:/HK8/ChuyenDe-Robot/RBF1.png"))
    
    clock = pygame.time.Clock()
    
    while running and current_iteration < max_iteration:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
        num_robot_available = numbers
        
        # maze.draw()
        
        # Update each robot
        # check nếu 10S thì tạo lại quần thể
        last_time = pygame.time.get_ticks()
        runnTime = 0
        while num_robot_available > 0 and runnTime < 10 :
            for idx, robot in enumerate(robots):
                 # Skip crashed robots
                    
                ex, ey = 450 - robot.x, 800 - robot.y
                etheta = (math.pi / 2) - robot.theta
                robot.costfunction += ex**2 + ey**2 + etheta**2
                robot.IT += 1
                
                X_nn = np.array([
                    robot.sensor[0], robot.sensor[1], robot.sensor[2],
                    robot.x, robot.y, robot.theta, 450, 800, (math.pi / 2)
                ])
                
                W = P_pso[idx, :45].reshape(9, 5)
                V = P_pso[idx, 45:].reshape(5, 2)
                v1, v2 = noron.NN(X_nn, W, V).flatten()
                robot.v1, robot.v2 = v1 * SPEED, v2 * SPEED
                robot.move()
                # 
                
                # dx = robot.x - robot.last_position[0]
                # dy = robot.y - robot.last_position[1]
                # distance_moved = (dx**2 + dy**2) ** 0.5 
                # robot.total_distance += distance_moved 
                # robot.frames_count += 1
                
                
                # if robot.frames_count >= 600:
                #     min_distance_threshold = 5 
                #     if robot.total_distance < min_distance_threshold:
                #         robot.crash = True  
                #     robot.total_distance = 0
                #     robot.frames_count = 0
                # robot.last_position = (robot.x, robot.y)
                
                if robot.crash:
                    num_robot_available -= 1
                    continue 
                # 
                if robot.check(maze.map):
                    robot.costfunction /= robot.IT
                
                robot.sensors(maze.map, maze)
                robot.draw(maze.map)
                robot.draw_circle(maze.map)
                maze.frame((robot.x, robot.y), robot.theta)
                maze.sensor_data((robot.x, robot.y), robot.edges)
                
            # nếu đủ 10s thì break
            runnTime = (pygame.time.get_ticks() - last_time) / 1000
            pygame.display.update()
            maze.draw()
            clock.tick(60)
            
        if all(robot.crash for robot in robots):
                    print(f'Iteration {current_iteration} ended. Restarting with new robots.')
                    current_iteration += 1
                    if current_iteration >= max_iteration:
                        break
                    
                    # # Reset lại PSO
                    # P_pso = np.random.uniform(min_max[0], min_max[1], (pop_size, npar))
                    # V_pso = np.zeros((pop_size, npar))
                    # Pbest_fitness.fill(np.inf)
                    # Gbest_fitness = np.inf
                    # Pbest_position.fill(0)
                    # Gbest_position.fill(0)
                    
                    # Tạo lại danh sách robots
                    robots = [Robot(400, 70, math.pi / 2, "E:/HK8/ChuyenDe-Robot/RBF1.png") for _ in range(numbers)]
                    continue
        # Kiểm tra nếu tất cả robot bị crash thì restart iteration
        
        
        # Update PSO
        for idx, robot in enumerate(robots):
            if robot.crash:
                continue
            J = robot.costfunction
            if J < Pbest_fitness[idx]:
                Pbest_fitness[idx] = J
                Pbest_position[idx] = P_pso[idx]
            if J < Gbest_fitness:
                Gbest_fitness = J
                Gbest_position = P_pso[idx]
        
        fitness_values[current_iteration] = Gbest_fitness
        fitness_mean[current_iteration] = np.mean([robot.costfunction for robot in robots if not robot.crash])
        
        V_pso = (w_pso * V_pso + c1_pso * np.random.rand() * (Pbest_position - P_pso) 
                 + c2_pso * np.random.rand() * (Gbest_position - P_pso))
        P_pso += V_pso
        
        print(f'Iteration {current_iteration}: {Gbest_fitness}')
