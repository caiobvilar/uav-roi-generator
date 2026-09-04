find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Test)

set(CMAKE_AUTOMOC ON)

# -- geometry (hull, area, clipping, minimum-area-rectangle) --
add_executable(test_geometry test_geometry.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/ConvexHull.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/PolygonGeometry.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/PolygonClipping.cpp
)
target_include_directories(test_geometry PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_geometry PRIVATE Qt6::Core Qt6::Gui Qt6::Test)
add_test(NAME geometry COMMAND test_geometry)

# -- planning (decomposition + waypoint generation) --
add_executable(test_planning test_planning.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/ConvexHull.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/PolygonGeometry.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/PolygonClipping.cpp
    ${CMAKE_SOURCE_DIR}/src/planning/StripDecomposition.cpp
    ${CMAKE_SOURCE_DIR}/src/planning/BoustrophedonSweep.cpp
    ${CMAKE_SOURCE_DIR}/src/domain/Drone.cpp
    ${CMAKE_SOURCE_DIR}/src/domain/GeoValidation.cpp
)
target_include_directories(test_planning PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_planning PRIVATE Qt6::Core Qt6::Gui Qt6::Test)
add_test(NAME planning COMMAND test_planning)

# -- label placement (in-bounds + non-overlap) --
add_executable(test_labelplacer test_labelplacer.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/LabelPlacer.cpp
)
target_include_directories(test_labelplacer PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_labelplacer PRIVATE Qt6::Core Qt6::Gui Qt6::Test)
add_test(NAME labelplacer COMMAND test_labelplacer)

# -- geojson (CRS reprojection regression) --
add_executable(test_geojson test_geojson.cpp
    ${CMAKE_SOURCE_DIR}/src/io/GeoJSON.cpp
)
target_include_directories(test_geojson PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_geojson PRIVATE Qt6::Core Qt6::Gui Qt6::Test GDAL::GDAL)
add_test(NAME geojson COMMAND test_geojson)
