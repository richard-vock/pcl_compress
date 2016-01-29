###############################################################################
# Find PCLCompress
#
# This sets the following variables:
# PCLCOMPRESS_FOUND - True if PCLCompress was found.
# PCLCOMPRESS_INCLUDE_DIRS - Directories containing the PCLCompress include files.
# PCLCOMPRESS_LIBRARY_DIRS - Directories containing the PCLCompress library.
# PCLCOMPRESS_LIBRARIES - PCLCompress library files.

if(WIN32)
    find_path(PCLCOMPRESS_INCLUDE_DIR pcl_compress PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(PCLCOMPRESS_LIBRARY_PATH pcl_compress PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${PCLCOMPRESS_LIBRARY_PATH})
        get_filename_component(PCLCOMPRESS_LIBRARY ${PCLCOMPRESS_LIBRARY_PATH} NAME)
        find_path(PCLCOMPRESS_LIBRARY_DIR ${PCLCOMPRESS_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(PCLCOMPRESS_INCLUDE_DIR pcl_compress PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(PCLCOMPRESS_LIBRARY_PATH pcl_compress PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${PCLCOMPRESS_LIBRARY_PATH})
        get_filename_component(PCLCOMPRESS_LIBRARY ${PCLCOMPRESS_LIBRARY_PATH} NAME)
        find_path(PCLCOMPRESS_LIBRARY_DIR ${PCLCOMPRESS_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(PCLCOMPRESS_INCLUDE_DIRS ${PCLCOMPRESS_INCLUDE_DIR})
set(PCLCOMPRESS_LIBRARY_DIRS ${PCLCOMPRESS_LIBRARY_DIR})
set(PCLCOMPRESS_LIBRARIES ${PCLCOMPRESS_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCLCompress DEFAULT_MSG PCLCOMPRESS_INCLUDE_DIR PCLCOMPRESS_LIBRARY PCLCOMPRESS_LIBRARY_DIR)

mark_as_advanced(PCLCOMPRESS_INCLUDE_DIR)
mark_as_advanced(PCLCOMPRESS_LIBRARY_DIR)
mark_as_advanced(PCLCOMPRESS_LIBRARY)
mark_as_advanced(PCLCOMPRESS_LIBRARY_PATH)
