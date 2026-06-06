############################################################################
#
#   Copyright (c) 2024 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_library(AdvancedPositionControl
	AdvancedPositionControl.cpp
	AdvancedPositionControl.hpp
)

target_link_libraries(AdvancedPositionControl PUBLIC mathlib)
