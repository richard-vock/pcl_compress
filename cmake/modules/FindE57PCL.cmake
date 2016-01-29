###############################################################################
# Find E57PCL
#
# This sets the following variables:
# E57PCL_FOUND - True if E57PCL was found.
# E57PCL_INCLUDE_DIRS - Directories containing the E57PCL include files.
# E57PCL_LIBRARY_DIRS - Directories containing the E57PCL library.
# E57PCL_LIBRARIES - E57PCL library files.

if(WIN32)
    find_path(E57PCL_INCLUDE_DIR e57_pcl PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(E57PCL_LIBRARY_PATH e57_pcl PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${E57PCL_LIBRARY_PATH})
        get_filename_component(E57PCL_LIBRARY ${E57PCL_LIBRARY_PATH} NAME)
        find_path(E57PCL_LIBRARY_DIR ${E57PCL_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(E57PCL_INCLUDE_DIR e57_pcl PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(E57PCL_LIBRARY_PATH e57_pcl PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${E57PCL_LIBRARY_PATH})
        get_filename_component(E57PCL_LIBRARY ${E57PCL_LIBRARY_PATH} NAME)
        find_path(E57PCL_LIBRARY_DIR ${E57PCL_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(E57PCL_INCLUDE_DIRS ${E57PCL_INCLUDE_DIR})
set(E57PCL_LIBRARY_DIRS ${E57PCL_LIBRARY_DIR})
set(E57PCL_LIBRARIES ${E57PCL_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(E57PCL DEFAULT_MSG E57PCL_INCLUDE_DIR E57PCL_LIBRARY E57PCL_LIBRARY_DIR)

mark_as_advanced(E57PCL_INCLUDE_DIR)
mark_as_advanced(E57PCL_LIBRARY_DIR)
mark_as_advanced(E57PCL_LIBRARY)
mark_as_advanced(E57PCL_LIBRARY_PATH)
