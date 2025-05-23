# - Try to find G2O
# Once done this will define
#
#  G2O_FOUND - system has G2O
#  G2O_INCLUDE_DIRS - the G2O include directories
#  G2O_LIBRARIES - The libraries needed to use G2O

if(G2O_INCLUDE_DIRS AND G2O_LIBRARIES)
  # Already in cache, be silent
  set(G2O_FIND_QUIETLY TRUE)
endif()

# Default search path for g2o (can be overridden by CMAKE_PREFIX_PATH or G2O_ROOT_DIR)
set(G2O_DEFAULT_SEARCH_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o/install)
if(DEFINED ENV{G2O_ROOT_DIR})
    set(G2O_DEFAULT_SEARCH_PATH $ENV{G2O_ROOT_DIR})
endif()
if(G2O_ROOT_DIR)
    set(G2O_DEFAULT_SEARCH_PATH ${G2O_ROOT_DIR})
endif()


find_path(G2O_INCLUDE_DIR
  NAMES g2o/core/optimizable_graph.h # A distinctive header file
  PATHS ${G2O_DEFAULT_SEARCH_PATH}/include
  NO_DEFAULT_PATH
)

# List of g2o libraries ORB_SLAM2 typically needs
set(G2O_COMPONENT_LIBRARIES core stuff types_sba types_sim3) # Add solver_eigen if you use it

set(G2O_LIBRARIES_TEMP "")
foreach(COMPONENT ${G2O_COMPONENT_LIBRARIES})
  find_library(G2O_${COMPONENT}_LIBRARY
    NAMES g2o_${COMPONENT} # Assumes library names like libg2o_core.so
    PATHS ${G2O_DEFAULT_SEARCH_PATH}/lib
    NO_DEFAULT_PATH
  )
  if(G2O_${COMPONENT}_LIBRARY)
    list(APPEND G2O_LIBRARIES_TEMP ${G2O_${COMPONENT}_LIBRARY})
  else()
    message(WARNING "Could not find g2o component library: ${COMPONENT}")
  endif()
endforeach()

set(G2O_LIBRARIES ${G2O_LIBRARIES_TEMP})
set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G2O
  REQUIRED_VARS G2O_LIBRARIES G2O_INCLUDE_DIRS
  FAIL_MESSAGE "Could not find all required G2O libraries and include directory. Please specify G2O_ROOT_DIR or ensure g2o is in CMAKE_PREFIX_PATH."
)

mark_as_advanced(G2O_INCLUDE_DIR G2O_LIBRARIES)