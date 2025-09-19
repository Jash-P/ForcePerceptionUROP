from maestro_wifi import MaestroWiFiClient
from ar10_compat import AR10Compat

ESP_IP = "192.168.4.1"

with MaestroWiFiClient(ESP_IP, 9000) as m:
    # optional: configure polling for live telemetry while you run motions
    m.set_poll(6, 100); m.set_autopoll(True)
    print("polling enabled")

    hand = AR10Compat(m, calibration_file="calibration_file")
    print("hand initialized")

    hand.change_speed(20)          # same semantics as legacy
    hand.change_acceleration(10)
    print("speed and acceleration set")

    hand.open_hand()
    print("hand opened")
    hand.flex_finger(2)
    print("finger 2 flexed")
    hand.close_hand()
    print("hand closed")
