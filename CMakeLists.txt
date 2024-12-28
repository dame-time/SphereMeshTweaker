cmake_minimum_required(VERSION 3.16)

project(SphereMeshTweaker VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# set(CMAKE_OSX_SYSROOT "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX15.2.sdk")
# set(ZLIB_LIBRARY "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX15.2.sdk/usr/lib/libz.tbd")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools)
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)
find_package(Qt6 REQUIRED COMPONENTS Core)

set(TS_FILES SphereMeshTweaker_it_IT.ts)
set(GLM_DIR "${CMAKE_SOURCE_DIR}/external/glm")
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/eigen")
set(LIBIGL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/libigl/include")
set(ASSIMP_DIR "${CMAKE_SOURCE_DIR}/external/assimp")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${LIBIGL_INCLUDE_DIR})
include_directories(${ASSIMP_DIR}/include)

add_subdirectory(${GLM_DIR})
add_subdirectory(${ASSIMP_DIR})
add_subdirectory(external/SphereMeshBlendShape)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        ${TS_FILES}
)

qt_add_resources(PROJECT_SOURCES resources.qrc)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(SphereMeshTweaker
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        openglwidget.h openglwidget.cpp
        resources.qrc
        quadric.h quadric.cpp
        quadric3.h quadric3.cpp
        quadric2.h quadric2.cpp
        mesh.h mesh.cpp
        shader.h shader.cpp
        camera.h camera.cpp
        sphere_mesh_renderer.h sphere_mesh_renderer.cpp
        ray.h
    )

# Define target properties for Android with Qt 6 as:
#    set_property(TARGET SphereMeshTweaker APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(SphereMeshTweaker SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(SphereMeshTweaker
            ${PROJECT_SOURCES}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_include_directories(${PROJECT_NAME} PRIVATE ${GLM_DIR})
target_link_libraries(SphereMeshTweaker PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt6::OpenGLWidgets ${RESOURCES} glm assimp SphereMeshBlendShape)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.SphereMeshTweaker)
endif()
set_target_properties(SphereMeshTweaker PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS SphereMeshTweaker
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(SphereMeshTweaker)
endif()
