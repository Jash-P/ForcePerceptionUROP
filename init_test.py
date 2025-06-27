import rtde_control

# NOTE: add 0.4 to the z-axis value on pendant

def main():
    velocity = 0.5
    acceleration = 0.5
    blend_1 = 0.0
    blend_2 = 0.02
    blend_3 = 0.0
    path_pose1 = [-0.143, -0.435, 0.20, -0.001, 3.12, 0.04, velocity, acceleration, blend_1]
    path_pose2 = [-0.143, -0.435, 0.30, -0.001, 3.12, 0.04, velocity, acceleration, blend_2]
    path_pose3 = [0.250, -0.610, 0.170, 1.327, -2.865, -0.022, velocity, acceleration, blend_3]
    path = [path_pose1, path_pose2, path_pose3]

    # Send a linear path with blending in between - (currently uses separate script)
    rtde_c.moveL(path)
    rtde_c.stopScript()

try:
    rtde_c = rtde_control.RTDEControlInterface("169.254.150.50")
    print("Connected")
    main()
except Exception as e:
    print(f"Exception occurred: {e}")
finally:
    rtde_c.disconnect()