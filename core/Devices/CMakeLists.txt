add_library(Devices BM280.cpp
            ICM20948.cpp
            bhy.cpp)

target_include_directories(Devices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(Devices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)