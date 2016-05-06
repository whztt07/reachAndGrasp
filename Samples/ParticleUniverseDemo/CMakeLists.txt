# Configure ParticleUniverse demo build

set(HEADER_FILES include/ParticleUniverseDemo.h)
set(SOURCE_FILES src/ParticleUniverseDemo.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OGRE_SOURCE_DIR}/PlugIns/ParticleUniverse/include)

add_library(Sample_ParticleUniverseDemo ${OGRE_LIB_TYPE} ${SOURCE_FILES} ${HEADER_FILES})

#target_link_libraries(Sample_ParticleUniverseDemo ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${PARTICLEUNIVERSE_LIBRARIES})
target_link_libraries(Sample_ParticleUniverseDemo ${OGRE_LIBRARIES} ${OIS_LIBRARIES} Plugin_ParticleUniverse)


ogre_config_sample_lib(Sample_ParticleUniverseDemo)

if (APPLE AND NOT OGRE_BUILD_PLATFORM_APPLE_IOS)
    # Set the INSTALL_PATH so that Samples can be installed in the application package
    set_target_properties(Sample_ParticleUniverseDemo
       PROPERTIES BUILD_WITH_INSTALL_RPATH 1
       INSTALL_NAME_DIR "@executable_path/../Plugins"
    )
endif()

