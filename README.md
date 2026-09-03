# UAV ROI Generator

A Qt6 + GDAL desktop application (C++/CMake) for generating drone coverage
waypoints over a user-drawn region of interest (ROI) on a georeferenced image.

## What it does

- Opens a georeferenced raster (GeoTIFF, PNG+worldfile, etc.) via GDAL.
- Lets the user draw an arbitrary polygon over the image.
- Computes the convex hull (Graham scan) and the minimum-area rotated rectangle
  (rotating calipers) for the ROI.
- Decomposes the ROI into sub-areas proportionally to each drone's relative
  coverage capability.
- Generates boustrophedon (lawn-mower) flight lines / waypoints per sub-area,
  accounting for camera footprint, forward/side overlap, ground sample distance
  (GSD), and motion-blur velocity limits.
- Exports the result as GeoJSON (reprojected to WGS84).

Drone parameters (camera, battery, velocity) are read from a JSON file — see
[`artifacts/drones.json`](artifacts/drones.json) for the expected schema.

## Dependencies

- Qt6 >= 6.5 (Core, Gui, Widgets)
- GDAL

On Fedora:

```bash
sudo dnf install -y qt6-qtbase-devel gdal-devel cmake gcc-c++
```

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

Run the binary at `build/ROIGenerator`.

## Usage

1. Open a georeferenced image (`Open image`).
2. Draw a polygon over the region of interest.
3. Load a drone list (`Open Drone Info`, see `artifacts/drones.json`).
4. Decompose the ROI and generate waypoints.
5. Export the result as GeoJSON.

## Notes

- Coordinates are exported to WGS84 (EPSG:4326); the ROI/map handling works in a
  projected CRS and reprojects on export. Some internal validation logic was
  developed against UTM zone 24S (EPSG:31984) imagery — adjust if your source
  data uses a different CRS.
- This tool originated as part of a PhD research project on multi-UAV formation;
  the path-planning portion implements coverage decomposition from a published
  method and was validated against Google Maps.

## License

[MIT](LICENSE)
