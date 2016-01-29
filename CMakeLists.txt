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
    add_definitions(-DBOOST_USE_VALGRIND)
endif()

find_package(OpenCV REQUIRED core highgui imgproc)
#find_package(Boost COMPONENTS system filesystem program_options)
find_package(PCL COMPONENTS common io kdtree)
find_package(Leptonica)
find_package(JBig2Enc)
find_package(JBig2Dec)
find_package(OpenJPEG)
#find_package(PrimitiveDetection)
find_package(ZLib)
#find_package(E57PCL)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
if (OPENCV_CORE_FOUND AND OPENCV_HIGHGUI_FOUND AND LEPTONICA_FOUND AND JBIG2ENC_FOUND AND JBIG2DEC_FOUND AND OPENJPEG_FOUND AND PCL_FOUND  AND ZLIB_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	include_directories(${LEPTONICA_INCLUDE_DIRS})
	include_directories(${JBIG2ENC_INCLUDE_DIRS})
	include_directories(${JBIG2DEC_INCLUDE_DIRS})
	include_directories(${OPENJPEG_INCLUDE_DIRS})
	include_directories(${PCL_INCLUDE_DIRS})
#include_directories(${PRIMITIVE_DETECTION_INCLUDE_DIRS})
	include_directories(${ZLIB_INCLUDE_DIRS})
#include_directories(${E57PCL_INCLUDE_DIRS})

    find_package(Boost COMPONENTS system filesystem program_options regex)
	add_library(pcl_compress SHARED ${obj})
	target_link_libraries(pcl_compress ${OpenCV_LIBS} ${LEPTONICA_LIBRARIES} ${JBIG2ENC_LIBRARIES} ${JBIG2DEC_LIBRARIES} ${OPENJPEG_LIBRARIES} ${PCL_LIBRARIES} ${ZLIB_LIBRARIES} "dl")

#include_directories(${Boost_INCLUDE_DIRS})

    ## apps
    # compress_octree
    #add_executable(compress_octree "apps/compress_octree.cpp")
    #target_link_libraries(compress_octree ${Boost_LIBRARIES} pcl_compress ${JBIG2DEC_LIBRARIES})
    # compress_primitives
    #add_executable(compress_primitives "apps/compress_primitives.cpp")
    #target_link_libraries(compress_primitives ${Boost_LIBRARIES} pcl_compress ${JBIG2DEC_LIBRARIES})
    # compress_e57n
    #add_executable(compress_e57n "apps/compress_e57n.cpp")
    #target_link_libraries(compress_e57n ${Boost_LIBRARIES} pcl_compress ${JBIG2DEC_LIBRARIES} ${E57PCL_LIBRARIES})
    # decompress_patches
    #add_executable(decompress_patches "apps/decompress_patches.cpp")
    #target_link_libraries(decompress_patches ${Boost_LIBRARIES} pcl_compress ${JBIG2DEC_LIBRARIES})
    # merge_compressed
    #add_executable(merge_compressed "apps/merge_compressed.cpp")
    #target_link_libraries(merge_compressed ${Boost_LIBRARIES} pcl_compress)

    # stats_decomp
    #add_executable(stats_decomp "apps/stats_decomp.cpp")
    #target_link_libraries(stats_decomp ${Boost_LIBRARIES} pcl_compress ${E57PCL_LIBRARIES})
    # compress_rmse
    #add_executable(compress_rmse "apps/compress_rmse.cpp")
    #target_link_libraries(compress_rmse ${Boost_LIBRARIES} pcl_compress ${E57PCL_LIBRARIES} ${PCL_LIBRARIES})
    # cloud_stats
    #add_executable(cloud_stats "apps/cloud_stats.cpp")
    #target_link_libraries(cloud_stats ${Boost_LIBRARIES} pcl_compress ${E57PCL_LIBRARIES} ${PCL_LIBRARIES})

    ## unittests
    #file (GLOB TEST_UNITS RELATIVE "${PROJECT_SOURCE_DIR}" "tests/*.cpp")
    #add_executable(unittest ${TEST_UNITS})
    #target_link_libraries(unittest ${Boost_LIBRARIES} pcl_compress ${JBIG2DEC_LIBRARIES} 'unittest' 'pthread')

    #include_directories(${PROJECT_SOURCE_DIR}/tests/include)
    # jbig2 test
    #add_executable(jbig2_test "tests/jbig2_test.cpp")
    #target_link_libraries(jbig2_test ${Boost_LIBRARIES} pcl_compress ${JBIG2DEC_LIBRARIES})
    # jpeg2000 test
    #add_executable(jpeg2000_test "tests/jpeg2000_test.cpp")
    #target_link_libraries(jpeg2000_test ${Boost_LIBRARIES} pcl_compress ${OPENJPEG_LIBRARIES})
    # patch test
    #add_executable(patch_test "tests/patch_test.cpp")
    #target_link_libraries(patch_test ${Boost_LIBRARIES} pcl_compress ${OPENJPEG_LIBRARIES})
    # serialize_octree_decomp
    #add_executable(serialize_octree_decomp "tests/serialize_octree_decomp.cpp")
    #target_link_libraries(serialize_octree_decomp ${Boost_LIBRARIES} pcl_compress ${OPENJPEG_LIBRARIES})
    # serialize_primitive_decomp
    #add_executable(serialize_primitive_decomp "tests/serialize_primitive_decomp.cpp")
    #target_link_libraries(serialize_primitive_decomp ${Boost_LIBRARIES} pcl_compress ${OPENJPEG_LIBRARIES})
    # quadtree_test
    #add_executable(quadtree_test "tests/quadtree_test.cpp")
    #target_link_libraries(quadtree_test ${Boost_LIBRARIES} pcl_compress ${OPENJPEG_LIBRARIES})

	# install binary
    if (WIN32)
        install (FILES "${PROJECT_BINARY_DIR}/libpcl_compress.dll.a" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/libpcl_compress.dll" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/libpcl_compress.dll" DESTINATION bin)
    else()
        install (TARGETS pcl_compress DESTINATION lib)
    endif()
    #install (TARGETS compress_octree DESTINATION bin)
    #install (TARGETS compress_primitives DESTINATION bin)
    #install (TARGETS compress_e57n DESTINATION bin)
    #install (TARGETS decompress_patches DESTINATION bin)
    #install (TARGETS merge_compressed DESTINATION bin)
    #install (TARGETS stats_decomp DESTINATION bin)
    #install (TARGETS compress_rmse DESTINATION bin)
    #install (TARGETS cloud_stats DESTINATION bin)
	# install header
	install (DIRECTORY include/ DESTINATION include/pcl_compress)
    # install find module
    file (GLOB find_modules RELATIVE "${PROJECT_SOURCE_DIR}" "cmake/modules/self/*.cmake")
    install (FILES ${find_modules} DESTINATION share/PCLCompress)
endif()

