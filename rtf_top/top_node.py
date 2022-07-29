##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
# from rclpy.clock import Clock # Clock().now().to_msg()
from std_msgs.msg import String
from colorama import Fore
import psutil
from rtf_interfaces.msg import Memory, Disk, CpuMemory



class rtf_psutil(Node):
    """
    """

    def __init__(self, cmtime=0.5, dtime=5.0, path="/"):
        """
        Contructor

        - cmtime: seconds, publish period for CPU/memory message, default 0.5
        - dtime: seconds, publish period for disk message, default 5.0
        - path: file path string, default '/' (root)
        """
        super().__init__('ros_psutil')
        self.cpu_mem_publisher = self.create_publisher(CpuMemory, 'cpu_memory', 10)
        self.disk_publisher = self.create_publisher(Disk, 'disk', 10)

        self.declare_parameter('cmtime', 0.5) # seconds
        self.cmtime = self.get_parameter('cmtime').value
        self.cmtimer = self.create_timer(self.cmtime, self.cpu_mem_callback)
        print(f">> {self.cmtimer.timer_period_ns}")

        self.declare_parameter('dtime', 5.0) # seconds
        self.dtime = self.get_parameter('dtime').value
        self.dtimer = self.create_timer(self.dtime, self.disk_callback)

        self.declare_parameter('path', '/') # seconds
        self.path = self.get_parameter('path').value

        self.add_on_set_parameters_callback(self.parameter_callback)

    def __del__(self):
        """
        Destroys dynamically created objects like Timers on exit
        """
        # print(f"{Fore.GREEN}pub delete{Fore.RESET}")
        self.destroy_timer(self.dtimer)
        self.destroy_timer(self.cmtimer)

    def parameter_callback(self, params):
        """
        Handles ROS2 parameter updates.

        ros2 param <get|set> <node> <parameter_name> [value]
        """
        for param in params:
            if param.name == 'cmtime' and param.type_ == Parameter.Type.DOUBLE:
                self.cmtime = param.value
                self.destroy_timer(self.cmtimer)
                self.cmtimer = self.create_timer(self.cmtime, self.cpu_mem_callback)
            elif param.name == 'dtime' and param.type_ == Parameter.Type.DOUBLE:
                self.dtime = param.value
                self.destroy_timer(self.dtimer)
                self.dtimer = self.create_timer(self.dtime, self.disk_callback)
            elif param.name == 'path' and param.type_ == Parameter.Type.STRING:
                self.path = param.value
            else:
                self.get_logger().warning(f"Invalid parameter: {param.name}[{param.type_}]")
        return SetParametersResult(successful=True)

    def cpu_mem_callback(self):
        """
        Publishes the CPU/memory of the system every self.cmtime
        """
        # c = psutil.cpu_percent(interval=self.cmtime*0.75, percpu=True)
        c = psutil.cpu_percent(interval=0.1, percpu=True)
        cpumem = CpuMemory()
        cpumem.header.stamp = self.get_clock().now().to_msg()
        cpumem.header.frame_id = "psutil"
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
        Publishes the disk info of self.path every self.dtime
        """
        d = psutil.disk_usage(self.path)
        disk = Disk()
        disk.path = self.path
        disk.total = d.total
        disk.used = d.used
        disk.free = d.free
        self.disk_publisher.publish(disk)

def main(args=None):
    print('Starting rtf_psutil.')
    try:
        rclpy.init(args=args)
        top = rtf_psutil()
        rclpy.spin(top)

    except KeyboardInterrupt:
        print(f'\n{Fore.CYAN}>> Received Ctrl-C ... shutting down{Fore.RESET}')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
