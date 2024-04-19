cmake_minimum_required(VERSION 3.0)

project(cog-agent VERSION 0.1.0)

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

add_library(cog-agent SHARED ${SOURCE_FILES})

target_include_directories(cog-agent PUBLIC include)
target_include_directories(cog-agent PRIVATE ${EXTERNAL_INCLUDE_DIRS})

target_link_libraries(cog-agent ${EXTERNAL_LIBS})
