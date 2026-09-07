# Deqing Serve Agent Guidelines

This file defines durable instructions for AI agents working in this repository. Keep it concise and actionable. Treat the current implementation as authoritative when README files or historical comments disagree with the code.

## 1. Project Overview

- Language: C++20 (required).
- Web framework: Drogon.
- Build system: CMake 3.10 or newer.
- Data services: PostgreSQL and Redis.
- Major dependencies: GDAL, PROJ, GEOS, OpenSceneGraph, TIFF, TBB, SQLite3, JsonCpp, and Zlib.
- The service listens on `0.0.0.0:9997`; Docker Compose maps host port `9990` to it by default.
- A working Redis configuration is required for full startup. PostgreSQL backs airspace, risk-area, fence, statistics, and OSGB grid-storage features.

## 2. Repository Map

- `main.cc`: initializes the base tile, Drogon, A* weights, CORS, TIFF elevation data, and Redis health checks, then starts the service.
- `controller/`: Drogon HTTP controllers.
  - `api_multiSource_basicGrid.*`: grid encoding, decoding, bounds, and parent/child levels.
  - `api_multiSource_geometricGrid.*`: point, line, polygon, polygon-with-holes, and buffer grid generation.
  - `api_multiSource_Data3dGrid.*`: volumetric region grid generation.
  - `api_multiSource_triangleGrid.*`: triangle, polyhedron, OSGB processing, and PostgreSQL persistence.
  - `api_multiSource_tifGrid.*`: background GeoTIFF/DEM conversion and Redis storage.
  - `api_multiSource_airSpace.*`: flyable-area statistics.
  - `api_airRoute_Astar.*`: 3D A* routing and path thinning.
  - `api_airRoute_*ConflictCheck.*`: point and polyline route-conflict checks.
- `GridEvaluatorLib/`: rule parsing, batched Redis reads, dynamic costs, and conflict evaluation.
- `dqglib/`: core static library for 3D grid encoding, geometry filling, adjacency, and OSGB/TIFF conversion.
- `models/TIFF.*`: thread-safe GDAL elevation-reader singleton.
- `models/GridData.*`: grid data model. The current build explicitly excludes `models/GridData.cc`; verify its intended role before changing it.
- `text/prefixToCode.*`: prefix and code helper logic.
- `readme/`: API and deployment documentation. It may lag behind the code.
- `build/` and `cmake-build-*`: generated directories, not source-editing targets.

## 3. Build, Run, and Validate

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/deqing_serve
```

Do not delete or clear an existing build directory unless the user explicitly authorizes it. If it belongs to another toolchain, create a new, clearly named build directory.

Configuration lookup behavior:

- `region.json`: use `./region.json` from the repository root; fall back to `../region.json` from a build directory.
- `config.json`: try `../config.json`, `./config.json`, then `/app/config.json`.
- `weight.json`: try `./weight.json`, `../weight.json`, then `/app/weight.json`.
- The TIFF path comes from `custom_config.tiff_file_path` and defaults to `./data/dem.tif`.

Validation order:

1. Documentation-only changes: check Markdown structure, paths, and commands; no compilation is required.
2. CMake or C++ changes: run at least an incremental build.
3. If CMake registers tests, run `ctest --test-dir build --output-on-failure`.
4. Do not assume a `test_main` target exists. If there are no automated tests, say so and validate the smallest relevant API scenario.
5. For Docker changes, run `docker compose config` first when available. Do not start, stop, or rebuild deployment containers without authorization.

## 4. Code Style

- Use 4-space indentation, K&R braces, and UTF-8.
- Use `camelCase` for variables and functions. Follow the local module's established class and filename style.
- Controller naming is historically inconsistent; do not perform unrelated bulk renames.
- Do not introduce `using namespace std;`; qualify standard-library names with `std::`. Do not touch unrelated legacy code only to enforce this rule.
- Follow existing project include conventions for local headers; use angle brackets for third-party headers.
- Use `///` for function documentation and `//` for inline comments.
- Prefer Simplified Chinese for implementation comments and user-facing project documentation.
- Comments should explain constraints, units, coordinate systems, or non-obvious reasoning instead of restating code.
- When adding `.cc` or `.cpp` files, update the relevant top-level or `dqglib/CMakeLists.txt` source list.

## 5. HTTP Controller Rules

- When changing a route, update and verify its `METHOD_ADD`, `ADD_METHOD_TO`, or `PATH_ADD` declaration and any corresponding API documentation.
- Validate the JSON body, required fields, field types, finite numeric values, and ranges before conversion or use.
- Coordinate arrays use `[longitude, latitude, height]`; longitude and latitude use WGS84/EPSG:4326.
- Keep absolute elevation distinct from relative operating height. In A*, target absolute altitude is ground elevation plus `workHeight`.
- OSGB and polyhedron endpoints currently constrain `level` to `0..21`. Other endpoints must use their existing `getGridSize` or module-specific validation rather than adopting a guessed global range.
- Use appropriate HTTP status codes and preserve the existing JSON contract, including at least `status` and `message` for errors.
- Catch expected exceptions and return clean errors; do not let request failures terminate the service.
- Invoke asynchronous callbacks exactly once and do not add long blocking work to Drogon event-loop threads.
- Background threads must own captured data for their full lifetime. Protect shared state with the existing `std::shared_ptr`, mutex, or concurrency patterns.

## 6. Grid and Route Invariants

- `projectBaseTile` must be initialized successfully before local-grid calculations.
- Use `double` for coordinates, heights, and distances. Use explicit integer types consistent with the surrounding API for levels and indices.
- Reuse `dqglib` for code truncation, hierarchy conversion, and ground projection; do not reimplement grid-code string logic.
- A* uses a 3D 26-neighbor search. Changes to neighbor expansion, safety radius, time advancement, or heuristics must be checked in both raw and smoothed route endpoints.
- `workHeight` and `planeRadius` are meters; `speed` is meters per second. Keep units explicit near calculations.
- Preserve current degraded behavior when true TIFF elevation is unavailable. Never treat an invalid elevation sentinel as zero elevation.
- Preserve chunking, batching, and concurrency limits for high-level or large-area grid operations.

## 7. Redis and Rule Engine

- Treat the key formats implemented in `GridEvaluator.cc` and `TIFtoCode.cc` as authoritative; do not copy potentially stale README examples without verification.
- When adding a rule, update key collection, batched retrieval, cache parsing, constraint evaluation, and final cost assignment together.
- Preserve batched asynchronous access: `MGET` for string values, `SMEMBERS` for sets, and `HMGET` for hash fields.
- Cache misses, invalid JSON, and Redis failures need explicit, conservative fallback behavior.
- Time rules combine UTC, Beijing time, and route arrival time. Confirm whether inputs are seconds or milliseconds and cover boundary and cross-day cases.
- Never log passwords, full connection strings, or large raw Redis payloads.

## 8. PostgreSQL and File Safety

- Bind SQL values as parameters; never concatenate user-controlled values into SQL.
- Dynamic table names may only come from validated internal values, such as a checked grid level.
- Changes involving `osgbgrid_<level>`, `update_log`, `air_space`, `fence`, or `risk_area` must account for schema creation, indexes, reads, writes, and transaction boundaries.
- Validate external paths for existence, type, and allowed roots. Reject `..`, `~`, and paths that escape the allowed root.
- `config.json` may contain PostgreSQL and Redis credentials. Never expose full configuration values in responses, logs, snapshots, or commits.
- Do not modify or regenerate large binary files such as `.tif`, `.osgb`, or `.lib` unless explicitly requested.

## 9. Git and Work Scope

- Run `git status --short` before editing and preserve existing user changes.
- Modify only files required by the request; do not format or refactor unrelated modules.
- Do not use `git reset --hard`, `git checkout --`, or `git clean`, and do not delete build directories to work around failures.
- Do not commit, push, or change remote branches unless explicitly requested.
- Read the relevant controller, implementation, and core-library interfaces before changing behavior. Search callers, configuration keys, table names, and Redis keys rather than inferring them from filenames.
- Prefer the smallest compatible change unless the user explicitly requests a breaking change.
- Before handoff, run `git diff --check` and `git diff --stat`, then report validation performed and any checks not run.

## 10. Additional Instructions

If `.github/copilot-instructions.md` later contains non-empty guidance, follow it as well. When instructions conflict, prefer the valid instruction closest to the edited directory and most specific to the task.
