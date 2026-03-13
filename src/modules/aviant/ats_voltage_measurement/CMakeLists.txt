px4_add_module(
	MODULE modules__aviant_ats_voltage_measurement
	MAIN ats_voltage_measurement
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
	SRCS
		AtsVoltageMeasurement.cpp
	DEPENDS
		px4_work_queue
	)
