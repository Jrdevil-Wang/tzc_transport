# tzc_transport
An efficient IPC for ROS with partial serialization

We propose an efficient IPC technique called TZC (Towards Zero-Copy). As a core component of TZC, we design a novel algorithm called partial serialization. Our formulation can generate messages that can be divided into two parts. During message transmission, one part is transmitted through a socket and the other part uses shared memory. The part within shared memory is never copied or serialized during its lifetime. Our tests show that when the message size is 4MB, TZC can reduce the overhead of ROS IPC from tens of milliseconds to hundreds of microseconds.

A more detailed report can be found: https://arxiv.org/abs/1810.00556
A video of applications is at https://github.com/Jrdevil-Wang/tzc_transport/blob/master/TZC.mp4

Directory description:
<table>
  <tr> <td>/example</td> <td>Code examples using TZC.</td> </tr>
  <tr> <td>/include/tzc_transport</td> <td>Core TZC code.</td> </tr>
  <tr> <td>/include/sensor_msgs</td> <td>Example of generated message header file.</td> </tr>
  <tr> <td>/scripts</td> <td>The TZC message generator.</td> </tr>
  <tr> <td>/templates</td> <td>The template file used by message generator.</td> </tr>
</table>
