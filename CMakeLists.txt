cmake_minimum_required(VERSION 3.0)

project(cog-ai VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS "-O0 -ggdb")
set(CMAKE_C_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_C_FLAGS_RELEASE "-O0 -ggdb")
set(CMAKE_CXX_FLAGS "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O0 -ggdb")

add_compile_definitions(ENABLE_GUI)
add_compile_definitions(ENABLE_GLFW)

set(SOURCE_FILES
	src/CogAIModule.cpp
	src/CogAgent.cpp
	src/CogAlgorithm.cpp
)

set(EXTERNAL_INCLUDE_DIRS
	external/steersuite/steerlib/include
	external/steersuite/util/include
	external/steersuite/external
	external/steersuite/external/imgui
	/usr/include
	/usr/local/include
)

set(EXTERNAL_LIBS
	steerlib
	util
)

link_directories(external/steersuite/build/lib)
include_directories(SYSTEM ${EXTERNAL_INCLUDE_DIRS})

add_library(cog-ai SHARED ${SOURCE_FILES})

target_include_directories(cog-ai PUBLIC include)

target_link_libraries(cog-ai ${EXTERNAL_LIBS})
