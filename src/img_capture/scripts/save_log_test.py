#!/usr/bin/env python3

## Code description:
# prints flight log to see if it is saved correctly

import numpy as np

try: 
    # data = np.read_csv('/home/username/ros2_ws/flight_logs/flight_odom_20250303_091230.csv')
    data = np.load('/home/username/ros2_ws/flight_logs/therm_images_20250303_092728.npy')
    
    # print(type(data))
    print(data.shape)
    print(data)

except Exception as e:
    print(f"Error loading file: {e}")