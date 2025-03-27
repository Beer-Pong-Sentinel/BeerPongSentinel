cmake_minimum_required(VERSION 3.16)

# Project name and settings
project(BeerPongSentinel VERSION 0.1 LANGUAGES CXX)
add_definitions(-D_WIN32 -DWIN64)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########################################################################
# Set these paths to match your system:
# TODO: Set your Qt installation directory here
set(CMAKE_PREFIX_PATH "C:/Qt/6.8.2/msvc2022_64")

# TODO: Set your Spinnaker SDK path here
set(SPINNAKER_SDK_PATH "C:/Program Files/Teledyne/Spinnaker")

# TODO: Set your OpenCV build path here
set(OPEN_CV_PATH "C:/opencv/build")

# TODO: Set your Teknic SDK path here (base directory)
set(TEKNIC_SDK_PATH "C:/Program Files (x86)/Teknic/ClearView/sdk")

set(EIGEN3_INCLUDE_DIR "C:/libs/eigen/eigen-3.4.0")
set(EIGEN3_UNSUPPORTED_DIR "C:/libs/eigen/eigen-3.4.0/unsupported")
########################################################################

set(QT_VERSION_MAJOR 6)
set(Qt6_DIR "${CMAKE_PREFIX_PATH}/lib/cmake/Qt6")

find_package(QT NAMES Qt6 REQUIRED COMPONENTS Widgets LinguistTools OpenGL OpenGLWidgets SerialPort Concurrent Qml)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools OpenGL OpenGLWidgets SerialPort Concurrent Qml)

set(TS_FILES BeerPongSentinel_en_CA.ts)

# List all your source files here
set(PROJECT_SOURCES
    main.cpp
    MainWindow.cpp
    MainWindow.h
    MainWindow.ui
    ${TS_FILES}
    config.json
    CameraCaptureThread.cpp
    CameraSetup.cpp
    CameraStreamWidget.cpp
    ImageProcessing.cpp
    AltitudeControl.cpp
    CameraCaptureThread.h
    CameraSetup.h
    CameraStreamWidget.h
    ImageProcessing.h
    AltitudeControl.h
    json.hpp
    CameraCalibration.cpp
    CameraCalibration.h
    AzimuthControl.cpp
    AzimuthControl.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(BeerPongSentinel
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        config.json
        CameraCaptureThread.cpp CameraSetup.cpp CameraStreamWidget.cpp ImageProcessing.cpp
        CameraCaptureThread.h CameraSetup.h CameraStreamWidget.h ImageProcessing.h
        json.hpp
        CameraCalibration.cpp
        CameraCalibration.h
        ProcessTimer.h ProcessTimer.cpp

    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET BeerPongSentinel APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(BeerPongSentinel SHARED ${PROJECT_SOURCES})
    else()
        add_executable(BeerPongSentinel ${PROJECT_SOURCES})
    endif()
    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

# Add include directories for external SDKs
include_directories("${SPINNAKER_SDK_PATH}/include")
include_directories("${OPEN_CV_PATH}/include")
include_directories("${TEKNIC_SDK_PATH}/inc")
include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("${EIGEN3_UNSUPPORTED_DIR}")

# Link directories for Spinnaker and OpenCV (if needed)
link_directories("${SPINNAKER_SDK_PATH}/lib64/vs2015")
link_directories("${OPEN_CV_PATH}/x64/vc16/lib")

# Link libraries:
# - Use generator expressions to select the proper Teknic library depending on configuration.
target_link_libraries(BeerPongSentinel PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    "${SPINNAKER_SDK_PATH}/lib64/vs2015/Spinnaker_v140.lib"
    "${OPEN_CV_PATH}/x64/vc16/lib/opencv_world4100d.lib"
    "${TEKNIC_SDK_PATH}/sFoundation Source/sFoundation/win/Debug/x64/sFoundation20.lib"
    "${TEKNIC_SDK_PATH}/lib/win/Release/x64/sFoundation20.lib"
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::SerialPort
    Qt6::Concurrent
    Qt6::Qml
)

# Custom post-build command to copy sFoundation DLLs (replicates the VS custom build step)
if(MSVC)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(PLATFORM_NAME "x64")
    else()
        set(PLATFORM_NAME "Win32")
    endif()
endif()

add_custom_command(TARGET BeerPongSentinel POST_BUILD
    COMMAND "C:/Program Files (x86)/Teknic/ClearView/sdk/examples/sFoundationCopy.bat" ${PLATFORM_NAME} $<CONFIG> "$<TARGET_FILE_DIR:BeerPongSentinel>"
    COMMENT "Copying sFoundation Files"
)

# Set target properties for executable bundling
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.BeerPongSentinel)
endif()
set_target_properties(BeerPongSentinel PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS BeerPongSentinel
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(BeerPongSentinel)
endif()
