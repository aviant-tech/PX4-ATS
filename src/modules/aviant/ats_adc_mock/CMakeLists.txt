px4_add_module(
	MODULE modules__aviant_ats_adc_mock
	MAIN ats_adc_mock
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
	SRCS
		AtsAdcMock.cpp
	DEPENDS
		px4_work_queue
	)
