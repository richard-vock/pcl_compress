###############################################################################
# Find PrimitiveDetection
#
# This sets the following variables:
# PRIMITIVE_DETECTION_FOUND - True if PrimitiveDetection was found.
# PRIMITIVE_DETECTION_INCLUDE_DIRS - Directories containing the PrimitiveDetection include files.
# PRIMITIVE_DETECTION_LIBRARY_DIRS - Directories containing the PrimitiveDetection library.
# PRIMITIVE_DETECTION_LIBRARIES - PrimitiveDetection library files.

find_path(PRIMITIVE_DETECTION_INCLUDE_DIR primitive_detection
    HINTS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}")

if (WIN32)
    find_library(PRIMITIVE_DETECTION_LIBRARY_PATH primitive_detection HINTS "/usr/x86_64-w64-mingw32/lib")
else()
    find_library(PRIMITIVE_DETECTION_LIBRARY_PATH primitive_detection HINTS "/usr/lib" "/usr/local/lib")
endif()

if(EXISTS ${PRIMITIVE_DETECTION_LIBRARY_PATH})
get_filename_component(PRIMITIVE_DETECTION_LIBRARY ${PRIMITIVE_DETECTION_LIBRARY_PATH} NAME)
find_path(PRIMITIVE_DETECTION_LIBRARY_DIR ${PRIMITIVE_DETECTION_LIBRARY} HINTS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib/")
endif()

set(PRIMITIVE_DETECTION_INCLUDE_DIRS ${PRIMITIVE_DETECTION_INCLUDE_DIR} "${PRIMITIVE_DETECTION_INCLUDE_DIR}/primitive_detection/pcshapes")
set(PRIMITIVE_DETECTION_LIBRARY_DIRS ${PRIMITIVE_DETECTION_LIBRARY_DIR})
set(PRIMITIVE_DETECTION_LIBRARIES ${PRIMITIVE_DETECTION_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PRIMITIVE_DETECTION DEFAULT_MSG PRIMITIVE_DETECTION_INCLUDE_DIR PRIMITIVE_DETECTION_LIBRARY PRIMITIVE_DETECTION_LIBRARY_DIR)

mark_as_advanced(PRIMITIVE_DETECTION_INCLUDE_DIR)
mark_as_advanced(PRIMITIVE_DETECTION_LIBRARY_DIR)
mark_as_advanced(PRIMITIVE_DETECTION_LIBRARY)
mark_as_advanced(PRIMITIVE_DETECTION_LIBRARY_PATH)
