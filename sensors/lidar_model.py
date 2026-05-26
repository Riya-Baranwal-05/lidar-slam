import numpy as np


def simulate_lidar_scan(pose, walls, max_range, num_beams, noise_std):
    """
    Simulates a 2D LiDAR scan from a robot's current pose.
    
    Inputs:
        pose: x,y and heading angle
        walls: obstacle positions
        max_range: Lidar's max range
        num_beams: 360 beams 
        noise_std: gaussian normal distribution
    
    Returns:
        It returns array of distances with added gaussian noise , one distance per beam showing 
        distance of robot from the obstacle.
    
    How it works:
        It uses ray casting , to return minimum distanced wall from the robot.
    """
    x,y,theta =  pose
    distance = []
    for i in range(0,num_beams):
        beam_angle = theta + (2*np.pi)/num_beams *i
        min_dist = float('inf')
        for wall_type, wall_val in walls:
            if wall_type == 'h':
                if abs(np.sin(beam_angle)) < 1e-10:
                        continue
                d = (wall_val - y) / np.sin(beam_angle)
              
            elif wall_type == 'v':
               
                d = (wall_val - x) / np.cos(beam_angle)
            
            if d > 0:
                min_dist = min(min_dist, d)
        if min_dist == float('inf') or min_dist>max_range:
                continue
        else:
            distance.append(min_dist+np.random.normal(0,noise_std))
    return distance
        
if __name__ == "__main__":
    pose = (2.0, 2.0, 0.0)
    walls = [
        ('h', 0.0), ('h', 4.0),
        ('v', 0.0), ('v', 4.0)
    ]
    scan = simulate_lidar_scan(pose, walls, 
                               max_range=10.0, 
                               num_beams=360, 
                               noise_std=0.01)
    print(f"Number of beams returned: {len(scan)}")
    print(f"Min distance: {min(scan):.2f}")
    print(f"Max distance: {max(scan):.2f}")
    print(scan)