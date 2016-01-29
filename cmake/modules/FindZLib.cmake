###############################################################################
# Find ZLib
#
# This sets the following variables:
# ZLIB_FOUND - True if ZLib was found.
# ZLIB_INCLUDE_DIRS - Directories containing the ZLib include files.
# ZLIB_LIBRARY_DIRS - Directories containing the ZLib library.
# ZLIB_LIBRARIES - ZLib library files.

if(WIN32)
    find_path(ZLIB_INCLUDE_DIR "zlib.h" PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(ZLIB_LIBRARY_PATH z PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${ZLIB_LIBRARY_PATH})
        get_filename_component(ZLIB_LIBRARY ${ZLIB_LIBRARY_PATH} NAME)
        find_path(ZLIB_LIBRARY_DIR ${ZLIB_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(ZLIB_INCLUDE_DIR "zlib.h" PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(ZLIB_LIBRARY_PATH z PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${ZLIB_LIBRARY_PATH})
        get_filename_component(ZLIB_LIBRARY ${ZLIB_LIBRARY_PATH} NAME)
        find_path(ZLIB_LIBRARY_DIR ${ZLIB_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR})
set(ZLIB_LIBRARY_DIRS ${ZLIB_LIBRARY_DIR})
set(ZLIB_LIBRARIES ${ZLIB_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZLib DEFAULT_MSG ZLIB_INCLUDE_DIR ZLIB_LIBRARY ZLIB_LIBRARY_DIR)

mark_as_advanced(ZLIB_INCLUDE_DIR)
mark_as_advanced(ZLIB_LIBRARY_DIR)
mark_as_advanced(ZLIB_LIBRARY)
mark_as_advanced(ZLIB_LIBRARY_PATH)
