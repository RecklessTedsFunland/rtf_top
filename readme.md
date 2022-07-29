![](header.webp)

# `rtf_psutil`

Like the powerful [F-ray][1], this reports performance of system.

- Message
    - CPU: [core0, core1, ...]
    - Memory: (used, max)
    - Disk space: (used, max)
    - Hostname: string
    - IPv4: string

```bash
>>> ros2 topic list
/cpu_memory
/disk
/parameter_events
/rosout
```

```bash
>>> ros2 node list
/ros_psutil
```

```bash
>>> ros2 node info /ros_psutil
/ros_psutil
  Subscribers:

  Publishers:
    /cpu_memory: rtf_interfaces/msg/CpuMemory
    /disk: rtf_interfaces/msg/Disk
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /ros_psutil/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ros_psutil/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ros_psutil/get_parameters: rcl_interfaces/srv/GetParameters
    /ros_psutil/list_parameters: rcl_interfaces/srv/ListParameters
    /ros_psutil/set_parameters: rcl_interfaces/srv/SetParameters
    /ros_psutil/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

```bash
>>> ros2 topic echo /cpu_memory
header:
  stamp:
    sec: 1659113794
    nanosec: 751915772
  frame_id: psutil
cpu:
- 30.0
- 0.0
- 0.0
- 90.0
memory:
  total: 1934405632
  available: 1456324608
  used: 281284608
  free: 739332096
  active: 483934208
  inactive: 0
  buffers: 83759104
  cached: 830029824
  shared: 7352320
```

```bash
>>> ros2 topic echo /disk                       
path: /
total: 15391461376
used: 3636686848
free: 11077103616
```

# Todo

- [ ] temperature info
- [ ] network info
- [ ] power/battery info?

# MIT License

**Copyright (c) 2020 Reckless Ted's Funland**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

[1]: https://futurama.fandom.com/wiki/F-Ray
