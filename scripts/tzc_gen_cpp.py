#!/usr/bin/env python

## ROS-TZC message source code generation for C++
##
## Converts ROS .msg files in a package into C++ source code implementations.

import sys
import os
import genmsg.template_tools

msg_template_map = { 'tzc_msg.hpp.template':'tzc_@NAME@.hpp' }
srv_template_map = { } # TODO handle .srv files

if __name__ == "__main__":
    genmsg.template_tools.generate_from_command_line_options(sys.argv, msg_template_map, srv_template_map)

