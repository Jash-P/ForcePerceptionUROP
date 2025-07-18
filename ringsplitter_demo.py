import rtde_control

# NOTE: add 0.4 to the z-axis value on pendant

def main():
    velocity = 0.1
    acceleration = 0.1
    blend_1 = 0.0
    blend_2 = 0.02
    blend_3 = 0.02
    blend_4 = 0.0
    path_pose1 = [-0.300, -0.325, 0.700, 0.88, 4.45, -0.96, velocity, acceleration, blend_1]
    path_pose2 = [-0.185, -0.532, 0.230, 0.37, 2.38, 1.60, velocity, acceleration, blend_2]
    path_pose3 = [-0.193, -0.705, 0.107, 0.989, -2.94, -1.19, velocity, acceleration, blend_3]
    path_pose4 = [-0.1034, -0.6310, 0.1454, 0.043, -2.592, -1.673, velocity, acceleration, blend_4]
    path = [path_pose1, path_pose2, path_pose3, path_pose4]

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