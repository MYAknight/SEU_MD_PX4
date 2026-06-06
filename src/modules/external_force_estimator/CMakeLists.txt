############################################################################
#
#   Copyright (c) 2024 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__external_force_estimator
	MAIN external_force_estimator
	SRCS
		external_force_estimator.cpp
		external_force_estimator.h
	DEPENDS
		px4_work_queue
	)
