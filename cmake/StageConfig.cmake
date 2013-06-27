# - Try to find Stage
#
# Once done this will define
#
#  Stage_FOUND - system has Stage
#  Stage_INCLUDE_DIRS - the Stage include directory
#  Stage_LIBRARIES - the Stage libraries
#
# Written by William Woodall <william@osrfoundation.org>
#

find_package(PkgConfig)
pkg_check_modules(PC_Stage stage)

find_library(Stage_LIBRARIES
  NAMES ${PC_Stage_LIBRARIES}
  PATHS ${PC_Stage_LIBRARY_DIRS}
)

set(Stage_INCLUDE_DIRS ${PC_Stage_INCLUDE_DIRS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Stage
	DEFAULT_MSG
	Stage_LIBRARIES Stage_INCLUDE_DIRS
)

mark_as_advanced(Stage_LIBRARIES Stage_INCLUDE_DIRS)

set(Stage_FOUND ${STAGE_FOUND})
