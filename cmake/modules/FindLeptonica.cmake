###############################################################################
# Find Leptonica
#
# This sets the following variables:
# LEPTONICA_FOUND - True if Leptonica was found.
# LEPTONICA_INCLUDE_DIRS - Directories containing the Leptonica include files.
# LEPTONICA_LIBRARY_DIRS - Directories containing the Leptonica library.
# LEPTONICA_LIBRARIES - Leptonica library files.

if(WIN32)
    find_path(LEPTONICA_INCLUDE_DIR leptonica PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(LEPTONICA_LIBRARY_PATH lept PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${LEPTONICA_LIBRARY_PATH})
        get_filename_component(LEPTONICA_LIBRARY ${LEPTONICA_LIBRARY_PATH} NAME)
        find_path(LEPTONICA_LIBRARY_DIR ${LEPTONICA_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(LEPTONICA_INCLUDE_DIR leptonica PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(LEPTONICA_LIBRARY_PATH lept PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${LEPTONICA_LIBRARY_PATH})
        get_filename_component(LEPTONICA_LIBRARY ${LEPTONICA_LIBRARY_PATH} NAME)
        find_path(LEPTONICA_LIBRARY_DIR ${LEPTONICA_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(LEPTONICA_INCLUDE_DIRS ${LEPTONICA_INCLUDE_DIR})
set(LEPTONICA_LIBRARY_DIRS ${LEPTONICA_LIBRARY_DIR})
set(LEPTONICA_LIBRARIES ${LEPTONICA_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Leptonica DEFAULT_MSG LEPTONICA_INCLUDE_DIR LEPTONICA_LIBRARY LEPTONICA_LIBRARY_DIR)

mark_as_advanced(LEPTONICA_INCLUDE_DIR)
mark_as_advanced(LEPTONICA_LIBRARY_DIR)
mark_as_advanced(LEPTONICA_LIBRARY)
mark_as_advanced(LEPTONICA_LIBRARY_PATH)
