cmake_minimum_required(VERSION 3.19)
project(ROIGenerator LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(GDAL REQUIRED)
qt_standard_project_setup()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

qt_add_executable(ROIGenerator WIN32 MACOSX_BUNDLE
    main.cpp
    ROIArea.h ROIArea.cpp
    GDALHandler.h GDALHandler.cpp
    io/GeoJSON.h io/GeoJSON.cpp
    mainwindow.h mainwindow.cpp mainwindow.ui
    pathplanner.h pathplanner.cpp
    geometry/ConvexHull.h geometry/ConvexHull.cpp
    geometry/PolygonGeometry.h geometry/PolygonGeometry.cpp
    geometry/PolygonClipping.h geometry/PolygonClipping.cpp
    domain/Drone.h domain/Drone.cpp
    domain/GeoValidation.h domain/GeoValidation.cpp
)

target_compile_options(ROIGenerator PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(ROIGenerator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ROIGenerator PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets GDAL::GDAL)

include(GNUInstallDirs)

install(TARGETS ROIGenerator
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET ROIGenerator
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
