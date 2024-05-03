cmake_minimum_required(VERSION 3.0)

project(cog-ai VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS "-O2 -ggdb -Wall")
set(CMAKE_CXX_FLAGS "-O2 -ggdb -Wall")

add_compile_definitions(ENABLE_GUI)
add_compile_definitions(ENABLE_GLFW)

set(SOURCE_FILES
	src/CogAgent.cpp
	src/CogAIModule.cpp
	src/CogAlgorithm.cpp
	src/CogConfig.cpp
)

set(EXTERNAL_INCLUDE_DIRS
	external/steersuite/external
	external/steersuite/external/imgui
	external/steersuite/steerlib/include
	external/steersuite/util/include
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
