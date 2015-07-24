###############################################################################
# Find JBig2Enc
#
# This sets the following variables:
# JBIG2ENC_FOUND - True if JBig2Enc was found.
# JBIG2ENC_INCLUDE_DIRS - Directories containing the JBig2Enc include files.
# JBIG2ENC_LIBRARY_DIRS - Directories containing the JBig2Enc library.
# JBIG2ENC_LIBRARIES - JBig2Enc library files.

if(WIN32)
    find_path(JBIG2ENC_INCLUDE_DIR jbig2enc PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(JBIG2ENC_LIBRARY_PATH jbig2enc PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${JBIG2ENC_LIBRARY_PATH})
        get_filename_component(JBIG2ENC_LIBRARY ${JBIG2ENC_LIBRARY_PATH} NAME)
        find_path(JBIG2ENC_LIBRARY_DIR ${JBIG2ENC_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(JBIG2ENC_INCLUDE_DIR jbig2enc PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(JBIG2ENC_LIBRARY_PATH jbig2enc PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${JBIG2ENC_LIBRARY_PATH})
        get_filename_component(JBIG2ENC_LIBRARY ${JBIG2ENC_LIBRARY_PATH} NAME)
        find_path(JBIG2ENC_LIBRARY_DIR ${JBIG2ENC_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(JBIG2ENC_INCLUDE_DIRS ${JBIG2ENC_INCLUDE_DIR})
set(JBIG2ENC_LIBRARY_DIRS ${JBIG2ENC_LIBRARY_DIR})
set(JBIG2ENC_LIBRARIES ${JBIG2ENC_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(JBig2Enc DEFAULT_MSG JBIG2ENC_INCLUDE_DIR JBIG2ENC_LIBRARY JBIG2ENC_LIBRARY_DIR)

mark_as_advanced(JBIG2ENC_INCLUDE_DIR)
mark_as_advanced(JBIG2ENC_LIBRARY_DIR)
mark_as_advanced(JBIG2ENC_LIBRARY)
mark_as_advanced(JBIG2ENC_LIBRARY_PATH)
