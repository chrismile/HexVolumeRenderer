find_path(CORK_INCLUDE_DIR NAMES cork.h)
find_library(CORK_LIBRARIES NAMES cork)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CORK DEFAULT_MSG CORK_LIBRARIES CORK_INCLUDE_DIR)

mark_as_advanced(CORK_INCLUDE_DIR CORK_LIBRARIES)
