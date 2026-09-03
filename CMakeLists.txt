cmake_minimum_required(VERSION 3.19)
project(ROIGenerator LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets Test)
find_package(GDAL REQUIRED)
qt_standard_project_setup()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set_property(SOURCE src/ui/ImageCanvas.h PROPERTY SKIP_AUTOUIC TRUE)

qt_add_executable(ROIGenerator WIN32 MACOSX_BUNDLE
    src/main.cpp
    src/ui/ImageCanvas.h src/ui/ImageCanvas.cpp
    src/ui/LabelPlacer.h src/ui/LabelPlacer.cpp
    src/ui/ColorPalette.h src/ui/ColorPalette.cpp
    src/io/GDALHandler.h src/io/GDALHandler.cpp
    src/io/GeoJSON.h src/io/GeoJSON.cpp
    src/ui/MainWindow.h src/ui/MainWindow.cpp src/ui/mainwindow.ui
    src/planning/PathPlanner.h src/planning/PathPlanner.cpp
    src/planning/IDecompositionStrategy.h
    src/planning/IWaypointGenerator.h
    src/planning/StripDecomposition.h src/planning/StripDecomposition.cpp
    src/planning/BoustrophedonSweep.h src/planning/BoustrophedonSweep.cpp
    src/geometry/ConvexHull.h src/geometry/ConvexHull.cpp
    src/geometry/PolygonGeometry.h src/geometry/PolygonGeometry.cpp
    src/geometry/PolygonClipping.h src/geometry/PolygonClipping.cpp
    src/domain/Drone.h src/domain/Drone.cpp
    src/domain/GeoValidation.h src/domain/GeoValidation.cpp
)

target_compile_options(ROIGenerator PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(ROIGenerator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

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

enable_testing()
add_subdirectory(tests)
