import socket

HOST = "192.168.4.1"
#HOST = "127.0.0.1"
PORT = 8000


def send_to_nano(command):
    """
    Sends a raw command (bytes or str) to the Nano's TCP server,
    which forwards it to the Maestro via TTL serial.
    Ensures that only whole Compact-protocol packets (4-byte) are sent,
    and that each packet is flushed immediately with TCP_NODELAY.

    :param command: bytes or str containing one or more concatenated 4-byte packets
    :raises ValueError: if the length of the packet isn’t a multiple of 4
    """
    # Convert string to bytes if necessary
    if isinstance(command, str):
        packet = command.encode('latin-1')
    else:
        packet = command

    # Validate we have whole 6-byte Pololu packets
    length = len(packet)
    if length % 6 != 0:
        raise ValueError(f"Invalid packet length {length}; must be a multiple of 4 bytes")

    # Open socket and disable Nagle so packets go out immediately
    with socket.create_connection((HOST, PORT), timeout=5) as s:
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.sendall(packet)
        print(f"Sent {length//4} packet(s): {[hex(b) for b in packet]}")
        s.shutdown(socket.SHUT_WR)
    print("Socket closed")

# Example usage:
if __name__ == "__main__":
    # Suppose self.pololu_command is the two-byte prefix: 0xAA, deviceNumber (e.g. 12)
    pololu_command = bytes([0xAA, 0])
    '''
    REMEMBER
    servos are on channels 10 onwards (1-9 are 'inputs')
    '''
    channel = 10
    target_quarter_us = 5000  # e.g., 1500 µs
    lsb = target_quarter_us & 0x7F
    msb = (target_quarter_us >> 7) & 0x7F

    # Build the full command packet: prefix + 0x04 + channel + lsb + msb
    command = pololu_command + bytes([0x04, channel, lsb, msb])
    send_to_nano(command)
