import rtde_control

# NOTE: add 0.4 to the z-axis value on pendant

def main():
    velocity = 0.1
    acceleration = 0.1
    blend_1 = 0.0
    blend_2 = 0.02
    blend_3 = 0.0
    path_pose1 = [-0.080, -0.600, 0.550, 0.27, -1.60, 0.40, velocity, acceleration, blend_1]
    path_pose2 = [-0.014, -0.400, 0.166, 0.616, 2.578, 1.727, velocity, acceleration, blend_2]
    path_pose3 = [0.263, -0.587, 0.172, 1.254, 1.754, 1.219, velocity, acceleration, blend_3]
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