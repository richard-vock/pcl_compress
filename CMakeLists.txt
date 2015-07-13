cmake_minimum_required(VERSION 2.8.8)
project(jbig2)

set (jbig2_VERSION_MAJOR 0)
set (jbig2_VERSION_MINOR 1)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
#set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
    if (NOT WIN32)
        add_definitions(-fPIC)
    else()
        add_definitions(-DM_LOG2E=1.44269504088896340736)
    endif()
	add_definitions(-O3)
	add_definitions(-g)
	add_definitions(-std=c++14)
    add_definitions(-Wall)
    add_definitions(-Wno-unused-function)
    add_definitions(-Wno-unused-local-typedefs)
    add_definitions(-Wno-deprecated-declarations)
endif()

find_package(OpenCV REQUIRED core highgui)
find_package(Boost COMPONENTS system filesystem)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
if (OPENCV_CORE_FOUND AND OPENCV_HIGHGUI_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})

	add_library(jbig2 SHARED ${obj})
	target_link_libraries(jbig2 ${OpenCV_LIBS} "dl")

    if (Boost_FOUND)
        include_directories(${PROJECT_SOURCE_DIR}/tests/include)

        include_directories(${Boost_INCLUDE_DIRS})
        add_executable(naive_test "tests/naive_test.cpp")
        target_link_libraries(naive_test ${Boost_LIBRARIES} jbig2)
    endif()

	# install binary
    if (WIN32)
        install (FILES "${PROJECT_BINARY_DIR}/libjbig2.dll.a" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/libjbig2.dll" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/libjbig2.dll" DESTINATION bin)
    else()
        install (TARGETS jbig2 DESTINATION lib)
    endif()
	# install header
	install (DIRECTORY include/ DESTINATION include/jbig2)
endif()

