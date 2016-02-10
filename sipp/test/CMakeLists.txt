project(sipp_test)

cmake_minimum_required(VERSION 2.4)


if (WIN32)
	set(EXTRA_LIBS winspool ws2_32)
else (WIN32)
	if (APPLE)
		find_library(COREFOUNDATION_LIBRARY CoreFoundation)
		find_library(IOKIT_LIBRARY IOKit)
		mark_as_advanced(COREFOUNDATION_LIBRARY)
		mark_as_advanced(IOKIT_LIBRARY)
		set(EXTRA_LIBS ${COREFOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
	else (APPLE)
		find_package(PkgConfig)
		pkg_check_modules(HAL hal)
		include_directories(${HAL_INCLUDE_DIRS})
		set(EXTRA_LIBS ${HAL_LIBRARIES} m pthread rt)
		add_definitions(-D_REENTRANT -D__USE_POSIX199309 -D_GNU_SOURCE)
	endif (APPLE)
endif (WIN32)

#dashel
find_path(DASHEL_INCLUDE_DIR dashel/dashel.h
	/usr/include
	/usr/local/include
)
find_library(DASHEL_LIBRARY dashel
	/usr/lib
	/usr/local/lib
)

add_executable(sipp_test sipp_test.cpp ../sipp.c)
include_directories(. ../ ${DASHEL_INCLUDE_DIR} ${EXTRA_INCLUDE_DIRS})
target_link_libraries(sipp_test ${DASHEL_LIBRARY} ${EXTRA_LIBS})
