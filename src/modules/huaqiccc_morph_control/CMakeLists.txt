px4_add_module(
	MODULE modules__huaqiccc_morph_control
	MAIN huaqiccc_morph_control
	SRCS
		HuaqicccMorphControl.cpp
		HuaqicccMorphControl.hpp
	DEPENDS
		px4_work_queue
)
