set(ONLY_LIBRARY ONLY_LIBRARY)

add_library(core
			${CMAKE_CURRENT_SOURCE_DIR}/core.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/ring_buffer.c)

target_link_libraries(core
		msftgsl
		Devices
		CommandParser
)

target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(external)
add_subdirectory(Devices)
add_subdirectory(CommandParser)

