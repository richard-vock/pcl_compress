###############################################################################
# Find OpenJPEG
#
# This sets the following variables:
# OPENJPEG_FOUND - True if OpenJPEG was found.
# OPENJPEG_INCLUDE_DIRS - Directories containing the OpenJPEG include files.
# OPENJPEG_LIBRARY_DIRS - Directories containing the OpenJPEG library.
# OPENJPEG_LIBRARIES - OpenJPEG library files.

if(WIN32)
    find_path(OPENJPEG_INCLUDE_DIR openjpeg-2.1 PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(OPENJPEG_LIBRARY_PATH openjp2 PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${OPENJPEG_LIBRARY_PATH})
        get_filename_component(OPENJPEG_LIBRARY ${OPENJPEG_LIBRARY_PATH} NAME)
        find_path(OPENJPEG_LIBRARY_DIR ${OPENJPEG_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(OPENJPEG_INCLUDE_DIR openjpeg-2.1 PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(OPENJPEG_LIBRARY_PATH openjp2 PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${OPENJPEG_LIBRARY_PATH})
        get_filename_component(OPENJPEG_LIBRARY ${OPENJPEG_LIBRARY_PATH} NAME)
        find_path(OPENJPEG_LIBRARY_DIR ${OPENJPEG_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(OPENJPEG_INCLUDE_DIRS ${OPENJPEG_INCLUDE_DIR})
set(OPENJPEG_LIBRARY_DIRS ${OPENJPEG_LIBRARY_DIR})
set(OPENJPEG_LIBRARIES ${OPENJPEG_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenJPEG DEFAULT_MSG OPENJPEG_INCLUDE_DIR OPENJPEG_LIBRARY OPENJPEG_LIBRARY_DIR)

mark_as_advanced(OPENJPEG_INCLUDE_DIR)
mark_as_advanced(OPENJPEG_LIBRARY_DIR)
mark_as_advanced(OPENJPEG_LIBRARY)
mark_as_advanced(OPENJPEG_LIBRARY_PATH)
