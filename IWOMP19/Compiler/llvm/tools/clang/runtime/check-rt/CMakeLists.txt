
set(SOURCES
  check-rt.cpp
  MemoryProfiling.cpp
  DebugInfo.cpp
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(BUILD_SHARED_LIBS ON)
add_llvm_library( check_rt-shared ${SOURCES} )
set_target_properties( check_rt-shared
  PROPERTIES
  OUTPUT_NAME "check-rt" )

target_link_libraries( check_rt-shared
  bfd
  elf
  z
  dl
  iberty
)

if(INTERNAL_INSTALL_PREFIX)
  set(LIBCHECK_RT_HEADERS_INSTALL_DESTINATION "${INTERNAL_INSTALL_PREFIX}/include")
else()
  set(LIBCHECK_RT_HEADERS_INSTALL_DESTINATION include)
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMPONENT libcheck-rt-headers
  DESTINATION "${LIBCHECK_RT_HEADERS_INSTALL_DESTINATION}"
  FILES_MATCHING
  PATTERN "check-rt.h"
  PATTERN ".svn" EXCLUDE
  )

if (NOT CMAKE_CONFIGURATION_TYPES) # don't add this for IDE's.
  add_custom_target(install-libcheck-rt-headers
    DEPENDS
    COMMAND "${CMAKE_COMMAND}"
            -DCMAKE_INSTALL_COMPONENT=libcheck-rt-headers
            -P "${CMAKE_BINARY_DIR}/cmake_install.cmake")
endif()
