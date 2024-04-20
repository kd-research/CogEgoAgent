cmake_minimum_required(VERSION 3.0)

project(cog-ai VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(ENABLE_GUI)

set(SOURCE_FILES
	src/CogAIModule.cpp
	src/CogAgent.cpp
)

set(EXTERNAL_INCLUDE_DIRS
	external/steersuite/steerlib/include
	external/steersuite/util/include
	external/steersuite/external
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
