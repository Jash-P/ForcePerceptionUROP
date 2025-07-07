import socket

HOST = "192.168.4.1"
PORT = 8000

def send_to_nano(command):
    """
    Sends a raw command (bytes or str) to the Nano's TCP server,
    which forwards it to the Maestro via TTL serial.
    
    :param command: bytes or str containing the command bytes
    """
    # Convert string to bytes if necessary
    if isinstance(command, str):
        packet = command.encode('latin-1')
    else:
        packet = command

    with socket.create_connection((HOST, PORT), timeout=5) as s:
        s.sendall(packet)
        print(f"Sent packet: {[hex(b) for b in packet]}")
        s.shutdown(socket.SHUT_WR)
    print("Socket closed")

# Example usage:
if __name__ == "__main__":
    # Suppose self.pololu_command is the two-byte prefix: 0xAA, deviceNumber (e.g. 12)
    pololu_command = bytes([0xAA, 12])
    channel = 0
    target_quarter_us = 6000  # e.g., 1500 µs
    lsb = target_quarter_us & 0x7F
    msb = (target_quarter_us >> 7) & 0x7F

    # Build the full command packet: prefix + 0x07 + channel + lsb + msb
    command = pololu_command + bytes([0x07, channel, lsb, msb])
    send_to_nano(command)
