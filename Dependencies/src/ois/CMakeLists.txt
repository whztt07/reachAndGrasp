cmake_minimum_required(VERSION 2.8.11)
project(OIS)

include_directories(includes)

# derived from msvs2010 project
option(OIS_BUILD_DEMOS "Build demo applications" FALSE)

set(OIS_HEADERS
  includes/OIS.h
  includes/OISConfig.h
  includes/OISEffect.h
  includes/OISEvents.h
  includes/OISException.h
  includes/OISFactoryCreator.h
  includes/OISForceFeedback.h
  includes/OISInputManager.h
  includes/OISInterface.h
  includes/OISJoyStick.h
  includes/OISKeyboard.h
  includes/OISMouse.h
  includes/OISMultiTouch.h
  includes/OISObject.h
  includes/OISPrereqs.h
  src/extras/LIRC/OISLIRC.h
  src/extras/LIRC/OISLIRCFactoryCreator.h
  src/extras/LIRC/OISLIRCRingBuffer.h
)
set(OIS_SOURCES
  src/OISEffect.cpp
  src/OISException.cpp
  src/OISForceFeedback.cpp
  src/OISInputManager.cpp
  src/OISJoyStick.cpp
  src/OISKeyboard.cpp
  src/OISObject.cpp
  src/extras/LIRC/OISLIRC.cpp
  src/extras/LIRC/OISLIRCFactoryCreator.cpp
)
if (WIN32)
  set(OIS_PLATFORM win32)
  set(OIS_PLATFORM_HEADERS
    includes/win32/Win32ForceFeedback.h
    includes/win32/Win32InputManager.h
    includes/win32/Win32JoyStick.h
    includes/win32/Win32KeyBoard.h
    includes/win32/Win32Mouse.h
    includes/win32/Win32Prereqs.h
    src/win32/extras/WiiMote/hiddevice.h
    src/win32/extras/WiiMote/OISWiiMote.h
    src/win32/extras/WiiMote/OISWiiMoteFactoryCreator.h
    src/win32/extras/WiiMote/OISWiiMoteForceFeedback.h
    src/win32/extras/WiiMote/OISWiiMoteRingBuffer.h
    src/win32/extras/WiiMote/wiimote.h
  )
  set(OIS_PLATFORM_SOURCES
    src/win32/Win32ForceFeedback.cpp
    src/win32/Win32InputManager.cpp
    src/win32/Win32JoyStick.cpp
    src/win32/Win32KeyBoard.cpp
    src/win32/Win32Mouse.cpp
    src/win32/extras/WiiMote/hiddevice.cpp
    src/win32/extras/WiiMote/OISWiiMote.cpp
    src/win32/extras/WiiMote/OISWiiMoteFactoryCreator.cpp
    src/win32/extras/WiiMote/OISWiiMoteForceFeedback.cpp
    src/win32/extras/WiiMote/OISWiiMoteRingBuffer.cpp
    src/win32/extras/WiiMote/wiimote.cpp
  )
  set(CMAKE_USE_RELATIVE_PATHS true)
  set(CMAKE_MODULE_PATH "${OIS_SOURCE_DIR}/CMake")
  # need DirectX includes and libraries
  find_package(DirectX REQUIRED)
  include_directories(${DirectX_INCLUDE_DIRS})
  set(OIS_PLATFORM_LIBS ${DirectX_DINPUT8_LIBRARY} ${DirectX_DXGUID_LIBRARY})
  if (MINGW)
    # standard MinGW cannot cope with the new Win32JoyStick implementation
	# therefore we use the old one from OIS 1.2
	list(REMOVE_ITEM OIS_PLATFORM_SOURCES src/win32/Win32JoyStick.cpp)
	list(APPEND OIS_PLATFORM_SOURCES src/win32/Win32JoyStick.old.cpp)
  endif ()
elseif (APPLE)
  option(OIS_BUILD_PLATFORM_APPLE_IOS "Apple is iOS" FALSE)
  if(OIS_BUILD_PLATFORM_APPLE_IOS)
    set(OIS_PLATFORM iphone)
    set(OIS_PLATFORM_HEADERS
      includes/iphone/iPhoneAccelerometer.h
      includes/iphone/iPhoneHelpers.h
      includes/iphone/iPhoneInputManager.h
      includes/iphone/iPhoneMultiTouch.h
      includes/iphone/iPhonePrereqs.h
    )
    set(OIS_PLATFORM_SOURCES
      src/iphone/iPhoneAccelerometer.mm
      src/iphone/iPhoneInputManager.mm
      src/iphone/iPhoneMultiTouch.mm
    )
  else(OIS_BUILD_PLATFORM_APPLE_IOS)
    set(OIS_PLATFORM mac)
    set(OIS_PLATFORM_HEADERS
      includes/mac/CocoaInputManager.h
      includes/mac/CocoaJoyStick.h
      includes/mac/CocoaKeyboard.h
      includes/mac/CocoaMouse.h
      includes/mac/CocoaHelpers.h
      includes/mac/MacHelpers.h
      includes/mac/MacHIDManager.h
      includes/mac/MacInputManager.h
      includes/mac/MacJoyStick.h
      includes/mac/MacKeyboard.h
      includes/mac/MacMouse.h
      includes/mac/MacPrereqs.h
    )
    set(OIS_PLATFORM_SOURCES
      src/mac/CocoaInputManager.mm
      src/mac/CocoaJoyStick.mm
      src/mac/CocoaKeyboard.mm
      src/mac/CocoaMouse.mm
      src/mac/MacHelpers.cpp
      src/mac/MacHIDManager.cpp
      src/mac/MacInputManager.cpp
      src/mac/MacJoyStick.cpp
      src/mac/MacKeyboard.cpp
      src/mac/MacMouse.cpp
    )
  endif(OIS_BUILD_PLATFORM_APPLE_IOS)
else()
  set(OIS_PLATFORM linux)
  set(OIS_PLATFORM_HEADERS
    includes/linux/EventHelpers.h
    includes/linux/LinuxForceFeedback.h
    includes/linux/LinuxInputManager.h
    includes/linux/LinuxJoyStickEvents.h
    includes/linux/LinuxKeyboard.h
    includes/linux/LinuxMouse.h
    includes/linux/LinuxPrereqs.h
  )
  set(OIS_PLATFORM_SOURCES
    src/linux/EventHelpers.cpp
    src/linux/LinuxForceFeedback.cpp
    src/linux/LinuxInputManager.cpp
    src/linux/LinuxJoyStickEvents.cpp
    src/linux/LinuxKeyboard.cpp
    src/linux/LinuxMouse.cpp
  )
endif ()

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "_d")
endif(MSVC)

if(APPLE)
  set_source_files_properties(${OIS_HEADERS} ${OIS_SOURCES} PROPERTIES COMPILE_FLAGS "-x objective-c++")
endif ()

if(NOT APPLE AND NOT ANDROID)
  add_definitions(-DOIS_NONCLIENT_BUILD -DOIS_DYNAMIC_LIB)
  if(WIN32)
    add_library(OIS SHARED ${OIS_HEADERS} ${OIS_SOURCES} ${OIS_PLATFORM_HEADERS} ${OIS_PLATFORM_SOURCES} Win32/OIS.rc)
    target_include_directories(OIS PUBLIC Win32)
  else()
    add_library(OIS SHARED ${OIS_HEADERS} ${OIS_SOURCES} ${OIS_PLATFORM_HEADERS} ${OIS_PLATFORM_SOURCES})
  endif()
else ()
  add_definitions(-DOIS_NONCLIENT_BUILD)
  add_library(OIS STATIC ${OIS_HEADERS} ${OIS_SOURCES} ${OIS_PLATFORM_HEADERS} ${OIS_PLATFORM_SOURCES})
endif ()

if (MINGW)
  # remove lib prefix from DLL outputs
  set_target_properties(OIS PROPERTIES PREFIX "")
endif ()

target_link_libraries(OIS ${OIS_PLATFORM_LIBS})

if (APPLE)
  if(OGRE_BUILD_PLATFORM_APPLE_IOS)
    set_target_properties(OIS PROPERTIES
      LINK_FLAGS "-framework UIKit"
    )
  else()
    set_target_properties(OIS PROPERTIES
      LINK_FLAGS "-framework IOKit -framework Cocoa -framework Carbon"
    )
  endif()
endif ()

if (APPLE)
 set_target_properties(OIS PROPERTIES XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH "NO")

 if(OGRE_BUILD_PLATFORM_APPLE_IOS)
  set_target_properties(OIS PROPERTIES XCODE_ATTRIBUTE_GCC_THUMB_SUPPORT "NO")
 endif()
endif()

if(${OIS_BUILD_DEMOS})
  if(WIN32)
    add_executable(Demo_Console demos/OISConsole.cpp Win32/CommandLine/CommandLine.rc)
    target_include_directories(Demo_Console PUBLIC Win32/CommandLine)
    target_link_libraries(Demo_Console OIS)
    add_executable(Demo_FFTest demos/FFConsoleDemo.cpp Win32/DemoFFTest/FF.rc)
    target_include_directories(Demo_FFTest PUBLIC Win32/DemoFFTest)
    target_link_libraries(Demo_FFTest OIS)
  else()
    add_executable(Demo_Console demos/OISConsole.cpp)
    target_link_libraries(Demo_Console OIS)
    add_executable(Demo_FFTest demos/FFConsoleDemo.cpp)
    target_link_libraries(Demo_FFTest OIS)
  endif()
endif()

if(MSVC)
  INSTALL(TARGETS OIS
    RUNTIME DESTINATION bin/Debug
    CONFIGURATIONS Debug
    LIBRARY DESTINATION lib/Debug
    CONFIGURATIONS Debug
    ARCHIVE DESTINATION lib/Debug
    CONFIGURATIONS Debug
  )
  INSTALL(FILES ${PROJECT_BINARY_DIR}/Debug/OIS_d.pdb DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/Debug CONFIGURATIONS Debug)
  INSTALL(TARGETS OIS
    RUNTIME DESTINATION bin/Release
    CONFIGURATIONS Release
    LIBRARY DESTINATION lib/Release
    CONFIGURATIONS Release
    ARCHIVE DESTINATION lib/Release
    CONFIGURATIONS Release
  )
  INSTALL(TARGETS OIS
    RUNTIME DESTINATION bin/MinSizeRel
    CONFIGURATIONS MinSizeRel
    LIBRARY DESTINATION lib/MinSizeRel
    CONFIGURATIONS MinSizeRel
    ARCHIVE DESTINATION lib/MinSizeRel
    CONFIGURATIONS MinSizeRel
  )
  INSTALL(TARGETS OIS
    RUNTIME DESTINATION bin/RelWithDebInfo
    CONFIGURATIONS RelWithDebInfo
    LIBRARY DESTINATION lib/RelWithDebInfo
    CONFIGURATIONS RelWithDebInfo
    ARCHIVE DESTINATION lib/RelWithDebInfo
    CONFIGURATIONS RelWithDebInfo
  )
  INSTALL(FILES ${PROJECT_BINARY_DIR}/RelWithDebInfo/OIS.pdb DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/RelWithDebInfo CONFIGURATIONS RelWithDebInfo)
else(MSVC)
  INSTALL(TARGETS OIS
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endif(MSVC)
install(FILES ${OIS_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
install(FILES ${OIS_PLATFORM_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${OIS_PLATFORM})
