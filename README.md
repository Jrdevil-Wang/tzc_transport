# tzc_transport
An efficient IPC for ROS with partial serialization

We propose an efficient IPC technique called TZC (Towards Zero-Copy). As a core component of TZC, we design a novel algorithm called partial serialization. Our formulation can generate messages that can be divided into two parts. During message transmission, one part is transmitted through a socket and the other part uses shared memory. The part within shared memory is never copied or serialized during its lifetime. Our tests show that when the message size is 4MB, TZC can reduce the overhead of ROS IPC from tens of milliseconds to hundreds of microseconds.

A more detailed report can be found: https://arxiv.org/pdf/1810.00556.pdf

Directory description:
/example:

    Code examples using TZC.

/include/tzc_transport:

    Core TZC code.

/include/sensor_msgs: 

    Example of generated message header file.

/scripts:

    The TZC message generator.

/templates:

    The template file used by message generator.
