cmake_minimum_required(VERSION 3.16)

project(BeerPongSentinel VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools OpenGL OpenGLWidgets SerialPort Concurrent Qml)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools OpenGL OpenGLWidgets SerialPort Concurrent Qml)

set(TS_FILES BeerPongSentinel_en_CA.ts)

set(PROJECT_SOURCES
        main.cpp
        MainWindow.cpp
        MainWindow.h
        MainWindow.ui
        ${TS_FILES}
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
        add_library(BeerPongSentinel SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(BeerPongSentinel
            ${PROJECT_SOURCES}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

# Define Spinnaker SDK path
set(SPINNAKER_SDK_PATH "C:/Program Files/Teledyne/Spinnaker")

# Define OpenCV path
set(OPEN_CV_PATH "C:/opencv/build")

# Add Spinnaker SDK include directory
include_directories("${SPINNAKER_SDK_PATH}/include")

# Add opencv include path
include_directories("${OPEN_CV_PATH}/include")

# Link to the Spinnaker SDK library directory
link_directories("${SPINNAKER_SDK_PATH}/lib64/vs2015")

# Link to the opencv library directory
link_directories("${OPEN_CV_PATH}/x64/vc16/lib")

# Link libraries, including Spinnaker
target_link_libraries(BeerPongSentinel PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    "${SPINNAKER_SDK_PATH}/lib64/vs2015/Spinnaker_v140.lib"
    "${OPEN_CV_PATH}/x64/vc16/lib/opencv_world4100d.lib"
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::SerialPort
    Qt6::Concurrent
    Qt6::Qml)


#target_link_libraries(BeerPongSentinel PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.BeerPongSentinel)
endif()
set_target_properties(BeerPongSentinel PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
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
