import rclpy
from rclpy.node import Node
# from rclpy.clock import Clock # Clock().now().to_msg()
from std_msgs.msg import String
from colorama import Fore
import psutil
from rtf_interfaces.msg import Memory, Disk, CpuMemory

class rtf_Top(Node):
    """
    """

    def __init__(self,cmtime=0.5,dtime=5.0,path="/"):
        """
        """
        super().__init__('ros_psutil')
        self.cpu_mem_publisher = self.create_publisher(CpuMemory, 'cpu_memory', 10)
        self.disk_publisher = self.create_publisher(Disk, 'disk', 10)
        self.timer_period = cmtime  # seconds
        self.cmtimer = self.create_timer(cmtime, self.cpu_mem_callback)
        self.dtimer = self.create_timer(dtime, self.disk_callback)
        self.path = path

    def __del__(self):
        """
        """
        print(f"{Fore.GREEN}pub delete{Fore.RESET}")

    def cpu_mem_callback(self):
        """
        """
        c = psutil.cpu_percent(interval=self.timer_period*0.75, percpu=True)
        cpumem = CpuMemory()
        cpumem.header.stamp = self.get_clock().now().to_msg()
        cpumem.header.frame_id = "kevin"
        cpumem.cpu = c
        # self.get_logger().info(f">> CPU: {info}")

        m = psutil.virtual_memory()
        mem = Memory()
        mem.total = m.total
        mem.available = m.available
        mem.used = m.used
        mem.free = m.free
        mem.active = m.active
        mem.buffers = m.buffers
        mem.cached = m.cached
        mem.shared = m.shared
        cpumem.memory = mem

        # self.get_logger().info("\n-----------------")
        # self.get_logger().info(f">> CpuMemory: {cpumem}")
        print(".", end="", flush=True)

        self.cpu_mem_publisher.publish(cpumem)

    def disk_callback(self):
        """
        """
        d = psutil.disk_usage(self.path)
        disk = Disk()
        disk.path = self.path
        disk.total = d.total
        disk.used = d.used
        disk.free = d.free
        self.disk_publisher.publish(disk)

def main(args=None):
    print('Starting rtf_top.')
    try:
        rclpy.init(args=args)
        top = rtf_Top()
        rclpy.spin(top)

    except KeyboardInterrupt:
        print('Ctrl-C')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
