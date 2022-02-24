cmake_minimum_required(VERSION 3.5)

project(geometric_match_ros LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(roscpp)
find_package(cv_bridge)
find_package(sensor_msgs)
find_package(OpenCV)

include_directories(
 ${roscpp_INCLUDE_DIRS}
 ${cv_bridge_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${sensor_msgs_INCLUDE_DIRS}
 ${geometry_msgs_INCLUDE_DIRS}
 include/rapidjson/include
)

if(ANDROID)
  add_library(geometric_match_ros SHARED
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
  )
else()
  add_executable(geometric_match_ros
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui

    imagecropper/imagecropper.h # add
    imagecropper/imagecropper.cpp # add
    imagecropper/imagecropper_e.h # add
    imagecropper/imagecropper_p.h # add

    algorithm/geomatch.h # add
    algorithm/geomatch.cpp # add

    worker.h # add
    worker.cpp # add
  )
endif()

target_link_libraries(geometric_match_ros PRIVATE Qt5::Widgets ${roscpp_LIBRARIES} ${cv_bridge_LIBRARIES} ${OpenCV_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES})
