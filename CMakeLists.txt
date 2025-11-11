cmake_minimum_required(VERSION 3.20)
project(O3F_Lite CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(O3F_BUILD_STATIC "Build with static linkage where possible" OFF)

find_package(SFML 2.5 COMPONENTS system window graphics REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

file(GLOB O3F_SOURCES CONFIGURE_DEPENDS
	${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_executable(o3f_lite ${O3F_SOURCES})

target_link_libraries(o3f_lite PRIVATE sfml-system sfml-window sfml-graphics)

if(MSVC)
	target_compile_options(o3f_lite PRIVATE /W4 /permissive-)
else()
	target_compile_options(o3f_lite PRIVATE -Wall -Wextra -Wpedantic)
	if(O3F_BUILD_STATIC)
		target_link_options(o3f_lite PRIVATE -static)
	endif()
endif()

if(WIN32)
	add_custom_command(TARGET o3f_lite POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E echo "Ensure SFML DLLs are on PATH or next to the exe."
	)
endif()
