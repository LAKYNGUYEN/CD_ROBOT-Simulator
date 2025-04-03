# import pygame, math, numpy as np
# import pandas as pd
# import os
# import csv
# class Noron(object):
#     def __init__(self):
#         pass
#     def function(self, x):
#         return np.tanh(x)
#     def NN(self, x, w, v):
#         net_h = w.T @ x
#         y_h = self.function(net_h)
#         return v.T @ y_h
# # class Noron(object):
# #     def __init__(self):
# #         pass
    
# #     def function(self, x):
# #         return np.tanh(x)

# #     def NN(self, x, w, v):
# #         # Thêm bias vào đầu vào (x)
# #         x = np.append(x, 1)  # Bias đầu vào
        
# #         # Tính toán lớp ẩn
# #         net_h = w.T @ x  
# #         y_h = self.function(net_h)
# #         y_h = np.append(y_h, 1)  # Thêm bias vào lớp ẩn
        
# #         # Tính toán lớp đầu ra
# #         output = v.T @ y_h  
# #         return output



# class Maze(object):
#     def __init__(self, image: str):
#         # colors
#         self.black = (0, 0, 0)
#         self.white = (255, 255, 255)
#         self.green = (0, 255, 0)
#         self.blue = (0, 0, 255)
#         self.red = (255, 0, 0)
#         self.orange = (255, 165, 0)
#         self.yellow = (255, 255, 0)
#         # Load image
#         self.image = pygame.image.load(image)
#         # Map dimensions
#         self.height = self.image.get_height()
#         self.width = self.image.get_width()
#         # Window settings
#         pygame.display.set_caption("Simulator Robot")
#         self.map = pygame.display.set_mode((self.width, self.height))
#         # font
#         pygame.font.init()  # Initialize the font module
#         self.font = pygame.font.SysFont('Arial', 25)
#         self.text = None
#         self.textRect = None
#     def frame(self, pos, rotation):
#         n = 80
#         cx, cy = pos
#         x_axis = (cx + n * math.cos(rotation), cy + n * math.sin(rotation))
#         y_axis = (cx + n * math.cos(rotation + math.pi / 2), cy + n * math.sin(rotation + math.pi / 2))
#         pygame.draw.line(self.map, self.red, (cx, cy), x_axis, 3)
#         pygame.draw.line(self.map, self.green, (cx, cy), y_axis, 3)
#     def sensor_info(self, data):
#         txt = f'Sensor: {data}'
#         self.text = self.font.render(txt, True, self.black, self.white)
#         self.textRect = self.text.get_rect()
#         self.map.blit(self.text, self.textRect)
#     def sensor_data(self, position, data):
#         for point  in data:    
#             pygame.draw.line(self.map, self.blue, position, point, 1)
#             pygame.draw.circle(self.map, self.blue, point, 5)
#     def draw(self):
#         self.map.blit(self.image, (0, 0))

# class Robot(object):
#     def __init__(self, x, y, theta, image):
#         # colors
#         self.black = (0, 0, 0)
#         self.white = (255, 255, 255)
#         self.green = (0, 255, 0)
#         self.blue = (0, 0, 255)
#         self.red = (255, 0, 0)
#         self.orange = (255, 165, 0)
#         self.yellow = (255, 255, 0)
#         # 
#         self.m2p = 3779.527559
#         self.x = x
#         self.y = y
#         self.theta = math.pi / 2   # Góc quay (radian)
#         self.v1 = 0.5 * self.m2p
#         self.v2 = -0.5 * self.m2p
#         self.speed = [-self.m2p * 1e-3, self.m2p * 1e-3] # min - max 
#         self.r = 10  # Bán kính bánh xe
#         self.l = 25  # Khoảng cách giữa 2 bánh xe
#         self.alpha = [math.pi / 2, math.pi / 2]
#         self.beta = [0, 0]
#         self.gamma = [0, 0]
#         # Load hình ảnh robot
#         self.img = pygame.image.load(image)
#         self.img = pygame.transform.scale(self.img, (40, 40))
#         self.rotated = self.img
#         self.rect = self.rotated.get_rect(center=(x, y))
#         self.dataTime = 0
#         self.lastTime = pygame.time.get_ticks()
#         # 
#         self.sensor = [0, 0, 0, 0, 0]
#         self.edges = []
#         # crash
#         self.crash = False
#         self.costfunction = 0
#         self.IT = 0
        
#         self.last_position = (self.x, self.y)  # Lưu vị trí trước đó
#         self.total_distance = 0 
#         self.frames_count = 0 
        
#         # Lưu trữ lịch sử vận tốc
#         # self.velocity_history = []
#         # self.counter = 0
        
#         self.previous_theta = 0  # Lưu góc trước đó để tính phạt xoay vòng
#         self.IT = 0  # Số bước di chuyển
#         self.start_position = (self.x, self.y)  # Lưu vị trí xuất phát
        
#         self.max_distance = 0  # Khoảng cách xa nhất đã đi được
#         self.min_distance = 0  # Khoảng cách gần nhất đến mục tiêu
#         # lưu giá trị tốt nhất
        
        
#         self.history = []  # Danh sách lưu lịch sử vị trí
#         self.max_history_size = 800  # Số lượng khung hình tối đa để kiểm tra
#         self.local_stuck_threshold = 40  # Ngưỡng phát hiện mắc kẹt (đơn vị pixel)

#         self.Penalty_Crash = 0  # Phạt nếu robot va vào tường
        
#     def move(self):
#         self.v1 = min(self.v1, self.speed[1])
#         self.v2 = min(self.v2, self.speed[1])
#         self.v1 = max(self.v1, self.speed[0])
#         self.v2 = max(self.v2, self.speed[0])
#         # Cập nhật thời gian
#         self.dataTime = (pygame.time.get_ticks() - self.lastTime) / 1000
#         self.lastTime = pygame.time.get_ticks()
#         # Ma trận quay R
#         R = np.array([
#                 [np.cos(self.theta), np.sin(self.theta), 0], 
#                 [-np.sin(self.theta), np.cos(self.theta), 0], 
#                 [0, 0, 1]])
#         j1f = np.array([[1, 0, -self.l], 
#                         [-1, 0, -self.l]])
#         j2 = np.array([[self.r, 0], 
#                     [0, self.r]])
#         vv = np.linalg.inv(R) @ np.linalg.pinv(j1f) @ j2 @ np.array([[self.v1], [self.v2]])
#         vx, vy, omega = vv.flatten()
#         new_x = self.x + vx * self.dataTime
#         new_y = self.y + vy * self.dataTime
#         new_theta = self.theta + omega * self.dataTime
#         # limit theta
#         new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi
#         # Cập nhật hình ảnh robot
#         self.x = new_x
#         self.y = new_y
#         self.theta = new_theta
#         self.rotated = pygame.transform.rotozoom(self.img, -math.degrees(self.theta), 1)
#         self.rect = self.rotated.get_rect(center = (self.x, self.y))
#     def sensors(self, map, maze: Maze):
#         angles = [
#             self.theta,
#             3*np.pi/2 + self.theta, #trai
#             np.pi/2 + self.theta,   #phai
#             5*np.pi/3 + self.theta, #XTRai
#             np.pi/3 + self.theta   #XPhai
#         ]    
#         edges = []
#         distances = []
#         for angle in angles:
#             distance = 0
#             step = 0.5
#             x = int(self.x)
#             y = int(self.y)
#             while True:
#                 x = int(self.x + distance * np.cos(angle))
#                 y = int(self.y +distance * np.sin(angle))
#                 distance += step
#                 if x < 0 or x >= maze.width or y < 0 or y >= maze.height:
#                     break
#                 if x < 0 or x >= 900 or y < 0 or y >= 800:
#                     break
#                 r, g, b, a = map.get_at((x, y))
#                 if r < 20 and g < 20 and b < 20:
#                     break
                
#             edges.append((x, y))
#             distances.append(distance)
#         self.sensor = distances
#         self.edges = edges
#     def draw(self, screen):
#         screen.blit(self.rotated, self.rect)
#     def draw_circle(self, screen):
#         pygame.draw.circle(screen, self.blue, self.rect.center, 20, 2)
#     def check(self, screen):
#         for angle in np.linspace(0, 2 * np.pi, 36):
#             x = int(self.x + 20 * np.cos(angle))
#             y = int(self.y + 20 * np.sin(angle))
#             if 0 <= x < screen.get_width() and 0 <= y < screen.get_height():
#                 r, g, b, a = screen.get_at((x, y))
#                 if r < 20 and g < 20 and b < 20: 
#                     self.crash = True
#                     return True
#         self.crash = False
#         return False



# def fitness_function(robot, goal_x=450, goal_y=800):
#     # Khoảng cách đến đích
#     distance_to_goal = np.sqrt((goal_x - robot.x) ** 2 + (goal_y - robot.y) ** 2)
    
#     # Phạt nếu robot va vào tường
#     collision_penalty = 200 if robot.crash else 0  

#     # Phạt nếu robot xoay vòng liên tục
#     rotation_penalty = abs(robot.theta - robot.previous_theta) * 0.05  

#     # Phạt dựa trên số bước di chuyển
#     # step_penalty = robot.IT * 0.03
#     # phạt nếu robot chạm tường
#     step_penalty = robot.IT * 2 if robot.crash else 0

#     # Phần thưởng nếu tiến gần đến mục tiêu
#     progress_reward = -250 if distance_to_goal < np.sqrt((goal_x - robot.last_position[0])**2 + (goal_y - robot.last_position[1])**2) else 0

#     # Thưởng cho khoảng cách xa nhất từ điểm xuất phát
#     start_x, start_y = robot.start_position
#     current_distance = np.sqrt((robot.x - start_x) ** 2 + (robot.y - start_y) ** 2)
#     if current_distance > robot.max_distance:
#         robot.max_distance = current_distance
#     max_distance_reward = robot.max_distance * 0.1  # Thưởng theo khoảng cách xa nhất đã đi được

#     # Thưởng nếu đạt khoảng cách gần nhất so với mục tiêu
#     if distance_to_goal < robot.min_distance:
#         robot.min_distance = distance_to_goal
#     min_distance_reward = (500 - robot.min_distance) * 0.05  # Thưởng khi đến gần mục tiêu hơn trước

#     # Tổng chi phí đánh giá
#     fitness_score = (distance_to_goal + collision_penalty  + rotation_penalty +
#                     step_penalty + robot.Penalty_Crash + progress_reward - max_distance_reward - min_distance_reward)
    
#     return fitness_score





# def is_stuck(robot):
   
#     if len(robot.history) < robot.max_history_size:
#         return False  # Chưa đủ dữ liệu để kiểm tra

#     # Lấy vị trí gần đây nhất
#     recent_positions = robot.history[-robot.max_history_size:]
#     x_positions = [pos[0] for pos in recent_positions]
#     y_positions = [pos[1] for pos in recent_positions]

#     # Tính khoảng cách giữa vị trí xa nhất và gần nhất trong lịch sử
#     max_x_diff = max(x_positions) - min(x_positions)
#     max_y_diff = max(y_positions) - min(y_positions)

#     # Nếu robot gần như đứng yên trong vùng nhỏ, thì coi là mắc kẹt
#     return max_x_diff < robot.local_stuck_threshold and max_y_diff < robot.local_stuck_threshold





# def position_file(iteration, Gbest_fitness, robots, filename="results.csv"):
#     import csv
#     # Tìm robot có fitness tốt nhất trong quần thể (fitness thấp nhất)
#     best_robot = min(robots, key=lambda r: fitness_function(r))
#     best_fitness = fitness_function(best_robot)
    
#     # Ghi thông tin robot tốt nhất vào file CSV
#     with open(filename, mode="a", newline="") as file:
#         writer = csv.writer(file)
#         # Nếu file mới hoặc chưa có dữ liệu, ghi header
#         if file.tell() == 0:
#             writer.writerow(["Iteration", "Gbest_Fitness", "V1", "V2", "X", "Y", "Robot_Fitness"])
#         writer.writerow([iteration, Gbest_fitness, best_robot.v1, best_robot.v2, best_robot.x, best_robot.y, best_fitness])


# if __name__ == "__main__":
#     pygame.init()
#     maze = Maze("maps/map-0.png")
#     noron = Noron()
#     running = True
#     numbers = 50 # Số lượng robot
#     robots = []
#     # SPEED =   # Speed of the robot
    
#     # PSO parameters
#     pop_size = numbers
#     npar = 65 # Number of parameters in the neural network
#     min_max = [-5, 5]
#     w_pso = 0.5
   
#     c1_pso = 2.5
#     c2_pso = 1.5
#     max_iteration = 1300
#     current_iteration = 0
    
    
#     # Initialize PSO variables
#     Pbest_fitness = np.full(pop_size, np.inf)
#     Gbest_fitness = np.inf
#     Pbest_position = np.zeros((pop_size, npar))
#     Gbest_position = np.zeros(npar)
    
#     fitness_values = np.zeros(max_iteration)
#     fitness_mean = np.zeros(max_iteration)
#     P_pso = np.random.uniform(min_max[0], min_max[1], (pop_size, npar))
#     V_pso = np.zeros((pop_size, npar))
#     JJ = np.zeros(pop_size)
#     # Create initial robots
#     for _ in range(numbers):
#         robots.append(Robot(450, 90, 4*math.pi/2, "E:/HK8/ChuyenDe-Robot/RBF1.png"))
    
#     clock = pygame.time.Clock()
#     cnti = 0
    
#     while running and current_iteration < max_iteration:
        
#         num_robot_available = numbers
#         # check nếu 10S thì tạo lại quần thể
#         last_time = pygame.time.get_ticks()
#         runnTime = 0
        
#         for robot in robots:
#             robots = [Robot(450, 70, 4*math.pi / 2, "E:/HK8/ChuyenDe-Robot/RBF1.png") for _ in range(numbers)]
#             robot.crash = False
#             robot.v1 = 0
#             robot.v2 = 0
#             robot.costfunction = 0
#             robot.IT = 0
#             # robot.check(maze.map)
#             robot.sensors(maze.map, maze)
#             robot.previous_theta = math.pi / 2
#             fitness_function(robot) == 0
#             robot.last_position = (robot.x, robot.y)  # Lưu vị trí hiện tại
#             robot.start_position = (robot.x, robot.y)  # Lưu vị trí xuất phát
            
#         while num_robot_available >0  and runnTime < 10:
#             for idx, robot in enumerate(robots):
#                     # Skip crashed robots
#                 if robot.crash == False:        
#                     ex, ey = 450 - robot.x, 800 - robot.y
#                     etheta = (math.pi / 2) - robot.theta
                    
#                     # robot.costfunction += ex**2 + ey**2 + etheta**2 + 0.1 
#                     fitness_function(robot)
#                     robot.IT += 1
#                     X_nn = np.array([robot.sensor[0], robot.sensor[1], robot.sensor[2], robot.sensor[3], robot.sensor[4],     
#                                     robot.x, robot.y, robot.theta, 450, 800, (4*math.pi / 2)  
#                                 ])

                    
#                     W = P_pso[idx, :55].reshape(11, 5)  # 12 đầu vào (bao gồm bias), 5 neuron ẩn
#                     V = P_pso[idx, 55:].reshape(5, 2)   # 6 đầu vào (5 neuron + bias), 2 đầu ra

                
#                     v1, v2 = noron.NN(X_nn, W, V).flatten()
                    
#                     # Cập nhật vận tốc và di chuyển robot
#                     robot.v1, robot.v2 = v1 , v2 
#                     robot.move()
#                     robot.history.append([robot.x, robot.y])
                    
#                     # nếu robot đứng cục bộ trong 1 vị trí nhất định  thì crash
                    
                    
#                     if robot.check(maze.map):
#                         num_robot_available -= 1
#                         robot.last_position = (robot.x, robot.y)  # Lưu vị trí hiện tại
                        
#                     # Cập nhật lịch sử vị trí
#                     robot.history.append((robot.x, robot.y))
#                     if len(robot.history) > robot.max_history_size:
#                         robot.history.pop(0)  # Giữ lịch sử giới hạn trong max_history_size

#                     # Nếu robot bị mắc kẹt, đánh dấu nó là crash
#                     if is_stuck(robot):
#                         print(f"Robot {idx} bị kẹt tại vị trí ({robot.x:.2f}, {robot.y:.2f})! Đánh dấu CRASH.")
#                         robot.crash = True
#                         robot.Penalty_Crash += 500  # Phạt nếu robot bị kẹt
#                         num_robot_available -= 1

                    
                    
#                     robot.sensors(maze.map, maze)
#                     robot.draw(maze.map)
#                     robot.draw_circle(maze.map)
#                     maze.frame((robot.x, robot.y), robot.theta)
#                     # maze.sensor_data((robot.x, robot.y), robot.edges)
#             # nếu đủ 10s thì break
#             pygame.display.update()
#             maze.draw()
#             runnTime = (pygame.time.get_ticks() - last_time) / 1000
#             # print(runnTime)
#             clock.tick(60)
            

#         # Gbest_position = load_best_positions()
        
#         # Update PSO
#         for idx, robot in enumerate(robots):
#             # J = robot.costfunction
#             J = fitness_function(robot)
           
#             JJ[idx] = J
#             if J < Pbest_fitness[idx]:
#                 Pbest_fitness[idx] = J
#                 Pbest_position[idx] = P_pso[idx]
#             if J < Gbest_fitness:
#                 Gbest_fitness = J
#                 Gbest_position = P_pso[idx]
#                 # if current_iteration == 50:
                
               
#         fitness_values[current_iteration] = Gbest_fitness
#         fitness_mean[current_iteration] = np.mean(JJ)
        
#         w_pso = 0.5 - (current_iteration / max_iteration) * (0.5 - 0.4)
 
#         # Update velocity and position
#         V_pso = (w_pso * V_pso + c1_pso * np.random.rand() * (Pbest_position - P_pso) 
#                  + c2_pso * np.random.rand() * (Gbest_position - P_pso))
#         # print(V_pso)
#         P_pso += V_pso
        
#         position_file(current_iteration, Gbest_fitness, robots)

        
#         print(f'Iteration {current_iteration}: {Gbest_fitness}')
#         current_iteration += 1
        
   
#     # Lưu kết quả vào file CSV

        
    
    
import pygame, math, numpy as np
import pandas as pd
import os
import csv


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
        # Màu sắc
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.orange = (255, 165, 0)
        self.yellow = (255, 255, 0)
        # Load image và thông tin map
        self.image = pygame.image.load(image)
        self.height = self.image.get_height()
        self.width = self.image.get_width()
        pygame.display.set_caption("Simulator Robot")
        self.map = pygame.display.set_mode((self.width, self.height))
        pygame.font.init()
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
        for point in data:    
            pygame.draw.line(self.map, self.blue, position, point, 1)
            pygame.draw.circle(self.map, self.blue, point, 5)
    def draw(self):
        self.map.blit(self.image, (0, 0))


class Robot(object):
    def __init__(self, x, y, theta, image):
        # Màu sắc
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.orange = (255, 165, 0)
        self.yellow = (255, 255, 0)
        # Thông số robot
        self.m2p = 3779.527559
        self.x = x
        self.y = y
        self.theta = math.pi / 2  # Góc quay (radian)
        self.v1 = 0.5 * self.m2p
        self.v2 = -0.5 * self.m2p
        self.speed = [-self.m2p * 1e-3, self.m2p * 1e-3]  # min - max 
        self.r = 10   # Bán kính bánh xe
        self.l = 25   # Khoảng cách giữa 2 bánh xe
        self.alpha = [math.pi / 2, math.pi / 2]
        self.beta = [0, 0]
        self.gamma = [0, 0]
        # Load hình ảnh robot
        self.img = pygame.image.load(image)
        self.img = pygame.transform.scale(self.img, (40, 40))
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(x, y))
        self.dataTime = 0
        self.lastTime = pygame.time.get_ticks()
        # Cảm biến
        self.sensor = [0, 0, 0, 0, 0]
        self.edges = []
        # Các biến trạng thái
        self.crash = False
        self.costfunction = 0
        self.IT = 0
        self.last_position = (self.x, self.y)  # Vị trí trước đó
        
        self.total_distance = 0 
        self.frames_count = 0 
        self.previous_theta = 0  # Dùng để tính phạt xoay vòng
        self.start_position = (self.x, self.y)
        self.max_distance = 0   # Khoảng cách xa nhất đã đi được
        self.min_distance = 0   # Khoảng cách gần nhất đến mục tiêu
        
        
        # Lịch sử vị trí (để kiểm tra robot bị kẹt)
        self.history = []
        self.max_history_size = 800
        self.local_stuck_threshold = 40
        self.Penalty_Crash = 0

    def move(self):
        self.v1 = min(self.v1, self.speed[1])
        self.v2 = min(self.v2, self.speed[1])
        self.v1 = max(self.v1, self.speed[0])
        self.v2 = max(self.v2, self.speed[0])
        self.dataTime = (pygame.time.get_ticks() - self.lastTime) / 1000
        self.lastTime = pygame.time.get_ticks()
        # Ma trận quay R
        R = np.array([
            [np.cos(self.theta), np.sin(self.theta), 0], 
            [-np.sin(self.theta), np.cos(self.theta), 0], 
            [0, 0, 1]
        ])
        j1f = np.array([[1, 0, -self.l], [-1, 0, -self.l]])
        j2 = np.array([[self.r, 0], [0, self.r]])
        vv = np.linalg.inv(R) @ np.linalg.pinv(j1f) @ j2 @ np.array([[self.v1], [self.v2]])
        vx, vy, omega = vv.flatten()
        new_x = self.x + vx * self.dataTime
        new_y = self.y + vy * self.dataTime
        new_theta = self.theta + omega * self.dataTime
        new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi
        self.x = new_x
        self.y = new_y
        self.theta = new_theta
        self.rotated = pygame.transform.rotozoom(self.img, -math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def sensors(self, map, maze: Maze):
        angles = [
            self.theta,
            3*np.pi/2 + self.theta,  # trái
            np.pi/2 + self.theta,    # phải
            5*np.pi/3 + self.theta,  # xiên trái
            np.pi/3 + self.theta     # xiên phải
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
                y = int(self.y + distance * np.sin(angle))
                distance += step
                if x < 0 or x >= maze.width or y < 0 or y >= maze.height:
                    break
                if x < 0 or x >= 900 or y < 0 or y >= 800:
                    break
                r, g, b, a = map.get_at((x, y))
                if r < 20 and g < 20 and b < 20:
                    break
            edges.append((x, y))
            distances.append(distance)
        self.sensor = distances
        self.edges = edges

    def draw(self, screen):
        screen.blit(self.rotated, self.rect)

    def draw_circle(self, screen):
        pygame.draw.circle(screen, self.blue, self.rect.center, 20, 2)

    def check(self, screen):
        for angle in np.linspace(0, 2 * np.pi, 36):
            x = int(self.x + 20 * np.cos(angle))
            y = int(self.y + 20 * np.sin(angle))
            if 0 <= x < screen.get_width() and 0 <= y < screen.get_height():
                r, g, b, a = screen.get_at((x, y))
                if r < 20 and g < 20 and b < 20:
                    self.crash = True
                    return True
        self.crash = False
        return False


def fitness_function(robot, goal_x=450, goal_y=800):
    distance_to_goal = np.sqrt((goal_x - robot.x) ** 2 + (goal_y - robot.y) ** 2)
    
    collision_penalty = 100 if robot.crash else 0
    
    rotation_penalty = abs(robot.theta - robot.previous_theta) * 0.05
    
    step_penalty = robot.IT * 0.03 if robot.crash else 0
    progress_reward = -50 if distance_to_goal < np.sqrt((goal_x - robot.last_position[0])**2 + (goal_y - robot.last_position[1])**2) else 0

    start_x, start_y = robot.start_position
    current_distance = np.sqrt((robot.x - start_x) ** 2 + (robot.y - start_y) ** 2)
    if current_distance > robot.max_distance:
        robot.max_distance = current_distance
    max_distance_reward = robot.max_distance * 0.1

    if distance_to_goal < robot.min_distance:
        robot.min_distance = distance_to_goal
    min_distance_reward = (500 - robot.min_distance) * 0.05

    fitness_score = (distance_to_goal + collision_penalty + rotation_penalty +
                     step_penalty + robot.Penalty_Crash + progress_reward -
                     max_distance_reward - min_distance_reward)
    return fitness_score


def is_stuck(robot):
    if len(robot.history) < robot.max_history_size:
        return False
    recent_positions = robot.history[-robot.max_history_size:]
    x_positions = [pos[0] for pos in recent_positions]
    y_positions = [pos[1] for pos in recent_positions]
    max_x_diff = max(x_positions) - min(x_positions)
    max_y_diff = max(y_positions) - min(y_positions)
    return max_x_diff < robot.local_stuck_threshold and max_y_diff < robot.local_stuck_threshold


def load_best_positions(filename="best_position.csv"):
    if os.path.exists(filename) and os.path.getsize(filename) > 0:
        df = pd.read_csv(filename, header=None)
        return df.values.flatten()  # Trả về mảng 1D
    else:
        return None

def save_best_positions(Gbest_position, filename="best_position.csv"):
    df = pd.DataFrame(Gbest_position.reshape(1, -1))
    df.to_csv(filename, header=False, index=False)

def position_file(iteration, Gbest_fitness, robots, filename="results.csv"):
    # Tìm robot có fitness tốt nhất
    best_robot = min(robots, key=lambda r: fitness_function(r))
    best_fitness = fitness_function(best_robot)
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow(["Iteration", "Gbest_Fitness", "V1", "V2", "X", "Y", "Robot_Fitness"])
        writer.writerow([iteration, Gbest_fitness, best_robot.v1, best_robot.v2,
                         best_robot.x, best_robot.y, best_fitness])

# re-train  
def retrain_with_gbest(pop_size, npar, min_max):
    Gbest = load_best_positions()
    if Gbest is None:
        print("Không tìm thấy Gbest đã lưu, khởi tạo quần thể ngẫu nhiên!")
        return np.random.uniform(min_max[0], min_max[1], (pop_size, npar))
    else:
        # Khởi tạo quần thể xung quanh Gbest (thêm nhiễu nhỏ)
        return np.array([Gbest + np.random.uniform(-0.5, 0.5, npar) for _ in range(pop_size)])


if __name__ == "__main__":
    pygame.init()
    maze = Maze("maps/map-0.png")
    noron = Noron()
    running = True
    numbers = 50  # Số lượng robot
    robots = []
    
    # PSO parameters
    pop_size = numbers
    npar = 65  # Số lượng tham số của NN
    min_max = [-5, 5]
    w_pso = 0.5
    c1_pso = 2.5
    c2_pso = 1.45
    max_iteration = 1300
    current_iteration = 0

    # Khởi tạo các biến PSO
    Pbest_fitness = np.full(pop_size, np.inf)
    Gbest_fitness = np.inf
    Pbest_position = np.zeros((pop_size, npar))
    # Sử dụng hàm retrain_with_gbest để khởi tạo P_pso dựa trên Gbest đã lưu (nếu có)
    P_pso = retrain_with_gbest(pop_size, npar, min_max)
    V_pso = np.zeros((pop_size, npar))
    JJ = np.zeros(pop_size)
    fitness_values = np.zeros(max_iteration)
    fitness_mean = np.zeros(max_iteration)

    # Tạo danh sách robot ban đầu
    for _ in range(numbers):
        robots.append(Robot(450, 90, 4*math.pi/2, "E:/HK8/ChuyenDe-Robot/RBF1.png"))
    
    clock = pygame.time.Clock()
    
    while running and current_iteration < max_iteration:
        num_robot_available = numbers
        last_time = pygame.time.get_ticks()
        runnTime = 0

        # Reset lại các robot cho mỗi iteration
        robots = [Robot(450, 70, 4*math.pi/2, "E:/HK8/ChuyenDe-Robot/RBF1.png") for _ in range(numbers)]
        for robot in robots:
            robot.crash = False
            robot.v1 = 0
            robot.v2 = 0
            robot.costfunction = 0
            robot.IT = 0
            robot.sensors(maze.map, maze)
            robot.previous_theta = math.pi / 2
            robot.last_position = (robot.x, robot.y)
            robot.start_position = (robot.x, robot.y)
            robot.history = []  # reset lịch sử

        # Vòng lặp di chuyển trong 10 giây
        while num_robot_available > 0 and runnTime < 60:
            for idx, robot in enumerate(robots):
                if not robot.crash:
                    # Cập nhật số bước
                    robot.IT += 1
                    X_nn = np.array([robot.sensor[0], robot.sensor[1], robot.sensor[2], 
                                      robot.sensor[3], robot.sensor[4], robot.x, robot.y, 
                                      robot.theta, 450, 800, (4*math.pi/2)])
                    W = P_pso[idx, :55].reshape(11, 5)
                    V = P_pso[idx, 55:].reshape(5, 2)
                    v1, v2 = noron.NN(X_nn, W, V).flatten()
                    
                    robot.v1, robot.v2 = v1*0.2, v2 *0.2
                    robot.move()
                    robot.history.append((robot.x, robot.y))
                    
                    if robot.check(maze.map):
                        num_robot_available -= 1
                        robot.last_position = (robot.x, robot.y)
                    
                    if len(robot.history) > robot.max_history_size:
                        robot.history.pop(0)
                    
                    if is_stuck(robot):
                        print(f"Robot {idx} bị kẹt tại vị trí ({robot.x:.2f}, {robot.y:.2f})! Đánh dấu CRASH.")
                        robot.crash = True
                        robot.Penalty_Crash += 500
                        num_robot_available -= 1
                    
                    robot.sensors(maze.map, maze)
                    robot.draw(maze.map)
                    robot.draw_circle(maze.map)
                    maze.frame((robot.x, robot.y), robot.theta)
                    
            pygame.display.update()
            maze.draw()
            runnTime = (pygame.time.get_ticks() - last_time) / 1000
            clock.tick(60)
        
        # Cập nhật PSO
        for idx, robot in enumerate(robots):
            J = fitness_function(robot)
            JJ[idx] = J
            if J < Pbest_fitness[idx]:
                Pbest_fitness[idx] = J
                Pbest_position[idx] = P_pso[idx]
            if J < Gbest_fitness:
                Gbest_fitness = J
                Gbest_position = P_pso[idx]
                save_best_positions(Gbest_position)  # Lưu Gbest mới nếu có

        fitness_values[current_iteration] = Gbest_fitness
        fitness_mean[current_iteration] = np.mean(JJ)
        
        w_pso = 0.5 - (current_iteration / max_iteration) * (0.5 - 0.4)
        V_pso = (w_pso * V_pso + c1_pso * np.random.rand() * (Pbest_position - P_pso) +
                 c2_pso * np.random.rand() * (Gbest_position - P_pso))
        P_pso += V_pso
        
        position_file(current_iteration, Gbest_fitness, robots)
        print(f'Iteration {current_iteration}: {Gbest_fitness}')
        current_iteration += 1
        
    pygame.quit()
