import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


class LabviewUdpBridgeNode(Node):
    """Republishes raw UDP datagrams sent by LabVIEW as ROS 2 messages."""

    def __init__(self):
        super().__init__('labview_udp_bridge_node')

        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('bind_port', 5006)
        self.declare_parameter('buffer_size', 4096)

        bind_ip = self.get_parameter('bind_ip').value
        bind_port = self.get_parameter('bind_port').value
        self._buffer_size = self.get_parameter('buffer_size').value

        self._publisher = self.create_publisher(UInt8MultiArray, 'labview/raw', 10)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((bind_ip, bind_port))
        # Timeout lets the recv thread wake up periodically to check
        # _running instead of blocking forever, so destroy_node() can
        # join it cleanly on shutdown.
        self._socket.settimeout(1.0)

        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        self.get_logger().info(f'Listening for LabVIEW UDP datagrams on {bind_ip}:{bind_port}')

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self._socket.recvfrom(self._buffer_size)
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().debug(f'Received {len(data)} bytes from {addr}: {data.hex()}')

            msg = UInt8MultiArray()
            msg.data = list(data)
            self._publisher.publish(msg)

    def destroy_node(self):
        self._running = False
        self._recv_thread.join(timeout=2.0)
        self._socket.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LabviewUdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
