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