px4_add_module(
	MODULE modules__hauv_water_control
	MAIN hauv_water_control
	SRCS
		HAUVWaterControl.cpp
	DEPENDS
		px4_work_queue
)
