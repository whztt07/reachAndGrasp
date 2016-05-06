//include_directories(include)
//include_directories(${PARTICLEUNIVERSE_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OGRE_SOURCE_DIR}/PlugIns/ParticleUniverse/include)
	

set (PARTICLEUNIVERSE_DEMO_NAME "particleuniversedemo")

file(GLOB PROJECT_SOURCES
	src/*.cpp
)

# OIS
find_package(OIS REQUIRED)
include_directories(${OIS_INCLUDE_DIRS})

add_executable(${PARTICLEUNIVERSE_DEMO_NAME} ${PROJECT_SOURCES})
target_link_libraries(${PARTICLEUNIVERSE_DEMO_NAME} ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${PARTICLEUNIVERSE_LIBRARIES})

# Copy resources.cfg to the build folder
#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/resources.cfg ${ParticleUniverse_BINARY_DIR}/resources.cfg COPYONLY)
