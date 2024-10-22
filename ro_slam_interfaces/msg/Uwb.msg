# UWB message.
#
# Lorenzo Bianchi <lnz.bnc@gmail.com>
#
# January 24, 2023

# This is free software.
# You can redistribute it and/or modify this file under the
# terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This file is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this file; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

# ROS 2 message header (with timestamp from ROS 2 real-time clock)
std_msgs/Header header

# UWB id
int8 id
string id_str

# Distance between rover and anchor (meters)
float32 dist

# DEBUG: UWB real position (meters)
float32 x
float32 y
float32 z
