# UWB array message.
#
# Lorenzo Bianchi <lnz.bnc@gmail.com>
#
# March 9, 2023

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

# UWB number
uint8 anchor_num

# Quality factor
int8 quality_factor

# Estimated position
float32 x_est
float32 y_est
float32 z_est

# UWB msgs
ro_slam_interfaces/Uwb[] uwbs


