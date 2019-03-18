# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import re
import sys

import em

import ament_index_python.packages

class ROS2Type:
    def __init__(self, ns, ros_type, lower_type):
        self.ns = ns
        self.ros_type = ros_type
        self.lower_type = lower_type

# Copied from rosidl_cmake
def convert_camel_case_to_lower_case_underscore(value):
    # insert an underscore before any upper case letter
    # which is not followed by another upper case letter
    value = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', value)
    # insert an underscore before any upper case letter
    # which is preseded by a lower case letter or number
    value = re.sub('([a-z0-9])([A-Z])', r'\1_\2', value)
    return value.lower()

def uniquify(inlist):
    seen = {}
    result = []
    for item in inlist:
        if item in seen:
            continue
        seen[item] = 1
        result.append(item)
    return result

MARKER_START = '// with input from '

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--packages', help='Space-separated list of packages to generate code for', nargs='*', default=[])
    parser.add_argument('--ros2-msgs', help='Space-separated list of ROS 2 messages to generate code for', nargs='*', default=[])
    parser.add_argument('--print-outputs', help='Print a semicolon-separated list of the files that *would* be generated', action='store_true')
    parser.add_argument('template_dir', help='Path to template directory')
    parser.add_argument('output_dir', help='Path to output directory')
    args = parser.parse_args()

    idl_files = []
    for p in args.packages:
        directory = os.path.join(ament_index_python.packages.get_package_share_directory(p), 'msg')
        idl_files.extend([os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.idl')])

    for t in args.ros2_msgs:
        split = t.strip().split('/')
        if len(split) != 2:
            print("Invalid ros2 message type; must be of the form <package>/<msg>")
            sys.exit(1)
        package = split[0]
        message = split[1]
        directory = os.path.join(ament_index_python.packages.get_package_share_directory(package), 'msg')
        found = False
        for f in os.listdir(directory):
            if not f.endswith('.idl'):
                continue

            if f[:-4] == message:
                idl_files.append(os.path.join(directory, f))
                found = True
                break

        if not found:
            print("Failed to find message '%s' in package '%s'; quitting" % (message, package))
            sys.exit(1)

    # Uniquify the list to only generate code for each message once.
    idl_files = uniquify(idl_files)

    em_globals = {'ros2_types': []}
    outputs_to_print = []
    for f in idl_files:
        with open(f, 'r') as infp:
            marker_line = None
            for line in infp:
                # The IDL files that are generated from rosidl_fastrtps_cpp all
                # have a two-line comment header; the second line looks like:
                #
                # // with input from std_msgs/msg/String.msg
                #
                # We look through the passed in IDL file for lines that begin
                # with that string; once we find it, we parse the line apart to
                # the the 3-tuple containing the namespace, the string 'msg',
                # and the name.  The namespace and 'msg' are used in the output
                # verbatim, but we have to do a bit of munging on the name.
                # In particular, we remove the .msg, then do a conversion of
                # camel case to lower case with underscores to get the filename
                # that things are stored in.  Once we have all of that
                # information, we can create the strings that are necessary for
                # creating the ros2_topics.hpp file.

                if not line.startswith(MARKER_START):
                    continue

                marker_line = line
                break

            if marker_line is None:
                print("Failed to find marker '%s' in '%s'; quitting" % (MARKER_START, f))
                sys.exit(3)

            # We found the name of the original file; we can do conversions on it now
            split = marker_line[len(MARKER_START):].strip().split('/')
            if len(split) != 3:
                print("Failed to find proper marker '%s' in '%s'; quitting" % (MARKER_START, f))
                sys.exit(2)

            ns = split[0]
            msg = split[1]
            name = split[2][:-4]  # This removes the '.msg' off the back
            lowername = convert_camel_case_to_lower_case_underscore(name)

            ros2_type = ROS2Type(ns, name, lowername)

            em_globals['ros2_types'].append(ros2_type)

            cpp_tmpl = os.path.join(args.template_dir, 'pub_sub_type.cpp.em')
            cpp_output = os.path.join(args.output_dir, ns + '_' + lowername + '_pub_sub_type.cpp')
            hpp_tmpl = os.path.join(args.template_dir, 'pub_sub_type.hpp.em')
            hpp_output = os.path.join(args.output_dir, ns + '_' + lowername + '_pub_sub_type.hpp')

            if args.print_outputs:
                outputs_to_print.append(cpp_output)
                outputs_to_print.append(hpp_output)
                continue

            em_cpp_locals = {'ros2_type': ros2_type}
            with open(cpp_output, 'w') as outfp:
                interpreter = em.Interpreter(output=outfp, globals=em_cpp_locals,
                                             options={em.RAW_OPT: True, em.BUFFERED_OPT: True})
                with open(cpp_tmpl, 'r') as infp:
                    interpreter.file(infp)

                interpreter.shutdown()

            em_hpp_locals = {'ros2_type': ros2_type}
            with open(hpp_output, 'w') as outfp:
                interpreter = em.Interpreter(output=outfp, globals=em_hpp_locals,
                                             options={em.RAW_OPT: True, em.BUFFERED_OPT: True})
                with open(hpp_tmpl, 'r') as infp:
                    interpreter.file(infp)

                interpreter.shutdown()

    ros2_topics_tmpl = os.path.join(args.template_dir, 'ros2_topics.hpp.em')
    ros2_topics_output = os.path.join(args.output_dir, 'ros2_topics.hpp')
    if args.print_outputs:
        outputs_to_print.append(ros2_topics_output)
        print(';'.join(outputs_to_print))
    else:
        with open(ros2_topics_output, 'w') as outfp:
            interpreter = em.Interpreter(output=outfp, globals=em_globals,
                                         options={em.RAW_OPT: True, em.BUFFERED_OPT: True})
            with open(ros2_topics_tmpl, 'r') as infp:
                interpreter.file(infp)

            interpreter.shutdown()
