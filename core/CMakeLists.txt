file(GLOB_RECURSE SOURCES
	"*.c"
	"*.cpp"
)

add_library(core
	${SOURCES})

target_link_libraries(core
		msftgsl
		Devices
)

target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(external)
add_subdirectory(Devices)
