cmake_minimum_required(VERSION 2.8.8)
project(pcl_compress)

set (pcl_compress_VERSION_MAJOR 0)
set (pcl_compress_VERSION_MINOR 1)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

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
    add_definitions(-Wno-write-strings)
endif()

find_package(OpenCV REQUIRED core highgui imgproc)
#find_package(Boost COMPONENTS system filesystem program_options)
find_package(PCL COMPONENTS common io)
find_package(Leptonica)
find_package(JBig2Enc)
find_package(JBig2Dec)
find_package(OpenJPEG)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
if (OPENCV_CORE_FOUND AND OPENCV_HIGHGUI_FOUND AND LEPTONICA_FOUND AND JBIG2ENC_FOUND AND JBIG2DEC_FOUND AND OPENJPEG_FOUND AND PCL_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	include_directories(${LEPTONICA_INCLUDE_DIRS})
	include_directories(${JBIG2ENC_INCLUDE_DIRS})
	include_directories(${JBIG2DEC_INCLUDE_DIRS})
	include_directories(${OPENJPEG_INCLUDE_DIRS})
	include_directories(${PCL_INCLUDE_DIRS})

    find_package(Boost COMPONENTS system filesystem program_options regex)
	add_library(pcl_compress SHARED ${obj})
	target_link_libraries(pcl_compress ${OpenCV_LIBS} ${LEPTONICA_LIBRARIES} ${JBIG2ENC_LIBRARIES} ${JBIG2DEC_LIBRARIES} ${OPENJPEG_LIBRARIES} ${PCL_LIBRARIES} "dl")

    if (Boost_FOUND)
        include_directories(${PROJECT_SOURCE_DIR}/tests/include)

        include_directories(${Boost_INCLUDE_DIRS})

        # jbig2 test
        add_executable(jbig2_test "tests/jbig2_test.cpp")
        target_link_libraries(jbig2_test ${Boost_LIBRARIES} pcl_compress ${JBIG2DEC_LIBRARIES})
        # jpeg2000 test
        add_executable(jpeg2000_test "tests/jpeg2000_test.cpp")
        target_link_libraries(jpeg2000_test ${Boost_LIBRARIES} pcl_compress ${OPENJPEG_LIBRARIES})
        # patch test
        add_executable(patch_test "tests/patch_test.cpp")
        target_link_libraries(patch_test ${Boost_LIBRARIES} pcl_compress ${OPENJPEG_LIBRARIES})
    endif()

	# install binary
    if (WIN32)
        install (FILES "${PROJECT_BINARY_DIR}/libpcl_compress.dll.a" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/libpcl_compress.dll" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/libpcl_compress.dll" DESTINATION bin)
    else()
        install (TARGETS pcl_compress DESTINATION lib)
    endif()
	# install header
	install (DIRECTORY include/ DESTINATION include/pcl_compress)
endif()

