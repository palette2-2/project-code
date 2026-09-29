find_package(Threads REQUIRED)

set(DDSC_LIBRARY
        ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libddsc.so.0)
set(DDSCXX_LIBRARY
        ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libddscxx.so.0)

if (NOT EXISTS ${DDSC_LIBRARY})
    message(FATAL_ERROR "CycloneDDS C library not found: ${DDSC_LIBRARY}")
endif ()

if (NOT EXISTS ${DDSCXX_LIBRARY})
    message(FATAL_ERROR "CycloneDDS C++ library not found: ${DDSCXX_LIBRARY}")
endif ()

message(STATUS "Importing: ${DDSC_LIBRARY}")

add_library(ddsc SHARED IMPORTED GLOBAL)
set_target_properties(ddsc PROPERTIES
        IMPORTED_LOCATION ${DDSC_LIBRARY})
target_link_libraries(ddsc INTERFACE Threads::Threads)
target_link_directories(ddsc INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
target_include_directories(ddsc INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

message(STATUS "Importing: ${DDSCXX_LIBRARY}")

add_library(ddscxx SHARED IMPORTED GLOBAL)
set_target_properties(ddscxx PROPERTIES
        IMPORTED_LOCATION ${DDSCXX_LIBRARY})
target_link_libraries(ddscxx INTERFACE Threads::Threads)
target_link_directories(ddscxx INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
target_include_directories(ddscxx INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ddscxx>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/ddscxx>)

## Install targets
file(GLOB DDS_LIBRARIES "${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/*")

install(FILES ${DDS_LIBRARIES} DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
