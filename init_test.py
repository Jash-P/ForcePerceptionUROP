import rtde_control

try:
    rtde_c = rtde_control.RTDEControlInterface("169.254.150.50")
    print("Connected")
    rtde_c.moveJ([0.256, -0.597, -0.233, 0.797, -2.333, -1.452], 0.5, 0.3)
except Exception as e:
    print(f"Exception occurred: {e}")
finally:
    rtde_c.disconnect()