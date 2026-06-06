############################################################################
#
#   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_romfs_files(
	1010_gazebo-classic_iris_opt_flow
	1010_gazebo-classic_iris_opt_flow.post
	1011_gazebo-classic_iris_irlock
	1012_gazebo-classic_iris_rplidar
	1013_gazebo-classic_iris_vision
	1013_gazebo-classic_iris_vision.post
	1015_gazebo-classic_iris_obs_avoid
	1015_gazebo-classic_iris_obs_avoid.post
	1017_gazebo-classic_iris_opt_flow_mockup
	1019_gazebo-classic_iris_dual_gps

	3010_jsbsim_quadrotor_x
	3011_jsbsim_hexarotor_x

	4001_gz_x500
	4002_gz_x500_depth
	4005_gz_x500_vision
	4006_gz_px4vision

	4400_gazebo-classic_huaqiccc

	6011_gazebo-classic_typhoon_h480
	6011_gazebo-classic_typhoon_h480.post

	10015_gazebo-classic_iris
	10016_none_iris
	10017_jmavsim_iris
	10018_gazebo-classic_iris_foggy_lidar
	10019_gazebo-classic_omnicopter
	10030_gazebo-classic_px4vision

	10040_sihsim_quadx
)
