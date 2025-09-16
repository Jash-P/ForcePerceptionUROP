from maestro_wifi import MaestroWiFiClient
from ar10_compat import AR10Compat

ESP_IP = "192.168.1.123"

with MaestroWiFiClient(ESP_IP, 9000) as m:
    # optional: configure polling for live telemetry while you run motions
    m.set_poll(6, 100); m.set_autopoll(True)

    hand = AR10Compat(m, calibration_file="calibration_file")

    hand.change_speed(20)          # same semantics as legacy
    hand.change_acceleration(10)

    hand.open_hand()
    hand.flex_finger(2)
    hand.close_hand()
