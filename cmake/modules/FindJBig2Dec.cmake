###############################################################################
# Find JBig2Dec
#
# This sets the following variables:
# JBIG2DEC_FOUND - True if JBig2Dec was found.
# JBIG2DEC_INCLUDE_DIRS - Directories containing the JBig2Dec include files.
# JBIG2DEC_LIBRARY_DIRS - Directories containing the JBig2Dec library.
# JBIG2DEC_LIBRARIES - JBig2Dec library files.

if(WIN32)
    find_path(JBIG2DEC_INCLUDE_DIR jbig2dec PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(JBIG2DEC_LIBRARY_PATH jbig2dec PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${JBIG2DEC_LIBRARY_PATH})
        get_filename_component(JBIG2DEC_LIBRARY ${JBIG2DEC_LIBRARY_PATH} NAME)
        find_path(JBIG2DEC_LIBRARY_DIR ${JBIG2DEC_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(JBIG2DEC_INCLUDE_DIR jbig2dec PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(JBIG2DEC_LIBRARY_PATH jbig2dec PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${JBIG2DEC_LIBRARY_PATH})
        get_filename_component(JBIG2DEC_LIBRARY ${JBIG2DEC_LIBRARY_PATH} NAME)
        find_path(JBIG2DEC_LIBRARY_DIR ${JBIG2DEC_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(JBIG2DEC_INCLUDE_DIRS ${JBIG2DEC_INCLUDE_DIR})
set(JBIG2DEC_LIBRARY_DIRS ${JBIG2DEC_LIBRARY_DIR})
set(JBIG2DEC_LIBRARIES ${JBIG2DEC_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(JBig2Dec DEFAULT_MSG JBIG2DEC_INCLUDE_DIR JBIG2DEC_LIBRARY JBIG2DEC_LIBRARY_DIR)

mark_as_advanced(JBIG2DEC_INCLUDE_DIR)
mark_as_advanced(JBIG2DEC_LIBRARY_DIR)
mark_as_advanced(JBIG2DEC_LIBRARY)
mark_as_advanced(JBIG2DEC_LIBRARY_PATH)
