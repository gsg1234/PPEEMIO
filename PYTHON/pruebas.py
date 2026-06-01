from emioapi import EmioMotors, EmioCamera
import time
import numpy as np
import random as rnd

np.set_printoptions(suppress=True,linewidth=None)

camera = EmioCamera(track_markers=True)
camera_connected = camera.open()

motors = EmioMotors()
motors_connected = motors.open()

"""
if motors_connected:
    while True:
        # Update motor angles
        angulos = [0, rnd.uniform(-np.pi/2, np.pi/2), 0, rnd.uniform(-np.pi/2, np.pi/2)]
        print("Setting motor angles to:", angulos)
        motors.angles = angulos
        time.sleep(1)
else:
    print("Failed to connect to the motors.")

"""
if camera_connected:
    if camera.open():
        while camera.is_running:
            # Update camera frames and tracking
            camera.update()

            # Access tracker positions
            positions = np.array(camera.trackers_pos)
            print("Tracker positions shape:", positions.shape)  # Print the shape of the positions array
            print(f"x = {positions[0, 0]:.2f} mm, y = {positions[0, 1]:.2f} mm")

            time.sleep(1)
else:
    print("Failed to connect to the camera.")
