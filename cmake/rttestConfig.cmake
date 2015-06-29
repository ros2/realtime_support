# rttest_FOUND
# rttest_INCLUDE_DIRS
# rttest_LIBRARIES
# rttest_DEFINITIONS

find_path(rttest_INCLUDE_DIR rttest/rttest.h)
find_library(rttest_LIBRARY_NAMES rttest)

set(rttest_LIBRARIES ${rttest_LIBRARY_NAMES})
set(rttest_INCLUDE_DIRS ${rttest_INCLUDE_DIR})

mark_as_advanced(rttest_LIBRARY_NAMES rttest_INCLUDE_DIR)
