import os
import time
import signal
import bota_driver
import sys
from collections import deque

# Flag for graceful shutdown
stop_flag = False

def signal_handler(signum, frame):
    global stop_flag
    stop_flag = True

# Register signal handler for graceful shutdown
signal.signal(signal.SIGINT, signal_handler)

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#################
## CONFIG FILE ##
#################

# Bota Serial Socket
config_path = os.path.join("/home/hans/projects/openarm/examples/Bota_Socket_SN001310.json")

##################
## DRIVER USAGE ##
##################

try:
    # Create driver instance
    bota_ft_sensor_driver = bota_driver.BotaDriver(config_path)
    
    # Transition driver from UNCONFIGURED to INACTIVE state
    if not bota_ft_sensor_driver.configure():
        raise RuntimeError("Failed to configure driver")

    # Uncomment to tare the sensor
    if not bota_ft_sensor_driver.tare():
        raise RuntimeError("Failed to tare sensor")

    # Transition driver from INACTIVE to ACTIVE state
    if not bota_ft_sensor_driver.activate():
        raise RuntimeError("Failed to activate driver")
    
    ########################
    ## CONTROL LOOP START ##
    ########################

    # Define the example duration
    EXAMPLE_DURATION = 10.0  # seconds

    start_time = time.perf_counter()  # High-resolution start time


    while time.perf_counter() - start_time < EXAMPLE_DURATION and not stop_flag:
        # Read frame
        bota_frame = bota_ft_sensor_driver.read_frame_blocking()
        
        # Extract the data from the bota_frame
        status = bota_frame.status
        force = bota_frame.force  
        torque = bota_frame.torque
        timestamp = bota_frame.timestamp
        temperature = bota_frame.temperature
        acceleration = bota_frame.acceleration
        angular_rate = bota_frame.angular_rate

        #################################
        ## YOUR CONTROL LOOP CODE HERE ##
        #################################

        print(f"Status: {status}, Force: {force}, Torque: {torque}, Timestamp: {timestamp}, Temperature: {temperature}, Acceleration: {acceleration}, Angular Rate: {angular_rate}")

    # Transition driver from ACTIVE to INACTIVE state
    if not bota_ft_sensor_driver.deactivate():
        raise RuntimeError("Failed to deactivate driver")
    
    # Shutdown the driver
    if not bota_ft_sensor_driver.shutdown():
        raise RuntimeError("Failed to shutdown driver")
    
    print("Completion WITHOUT errors.")

except Exception as e:
    print(f"FATAL: {e}")
    print("Completion WITH errors.")
    
finally:
    print("EXITING PROGRAM")
    sys.exit(0)

