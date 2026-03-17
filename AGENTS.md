# Agent Coding Guidelines for Deqing_serve

This file contains instructions for AI agents working on this codebase.

## 1. Project Overview

- **Language**: C++20 (Required)
- **Framework**: [Drogon](https://github.com/drogonframework/drogon) (High-performance C++ HTTP web framework)
- **Build System**: CMake (>= 3.10)
- **Database**: PostgreSQL (via Drogon ORM or libs), Redis (optional/cli)
- **Dependencies**: SQLite3, TBB, GEOS, JsonCpp

## 2. Build & Test Commands

### Build
```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

### Run
```bash
# From build directory
./deqing_serve
```
*Note: Ensure `config.json` and `region.json` are accessible (usually in project root or copies in build).*

### Test
```bash
# From build directory
cd test
make
./test_main  # Executable name may vary based on test/CMakeLists.txt
```

## 3. Code Style & Conventions

### Naming
- **Variables**: `camelCase` (e.g., `configFilePath`, `projectBaseTile`)
- **Functions**: `camelCase` (e.g., `getGridByPoint`, `initializeProjectBaseTileFromConfig`)
- **Classes**: `camelCase` or `PascalCase` (inconsistent, check surrounding code. `basicGrid` seen in controllers).
- **Namespaces**: `snake_case` or `camelCase` (e.g., `api::multiSource`).
- **Files**:
    - Controllers: `api_multiSource_featureName.cc` / `.h`
    - Core: `PascalCase.cc` (e.g., `GridEvaluator.cc`) or `lowercase.cc` (`main.cc`).
    - **Rule**: Follow the naming convention of the specific module you are editing.

### Formatting
- **Indentation**: 4 spaces.
- **Braces**: K&R style (opening brace on the same line).
- **Encoding**: UTF-8.

### Comments
- **Language**: Mandarin Chinese is preferred for implementation comments and documentation.
- **Style**: Use `///` for function documentation and `//` for inline comments.

## 4. Architecture Patterns

### Controllers (Drogon)
Controllers are located in `controller/`. They typically follow this pattern:

```cpp
// Header: controller/api_multiSource_example.h
#pragma once
#include <drogon/HttpController.h>
using namespace drogon;

namespace api {
namespace multiSource {
class Example : public drogon::HttpController<Example> {
public:
    METHOD_LIST_BEGIN
    // Map path to method
    ADD_METHOD_TO(Example::handler, "/api/example", Get, Post);
    METHOD_LIST_END

    void handler(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
};
}
}
```

**Implementation Details:**
1.  **Input Validation**: Strict validation of JSON fields.
    ```cpp
    auto jsonBody = req->getJsonObject();
    if (!jsonBody || !jsonBody->isMember("field")) {
        // Return 400 Bad Request with JSON error message
    }
    ```
2.  **Response Format**:
    ```json
    {
        "status": "success" | "error",
        "message": "Description...",
        "data": { ... }
    }
    ```
3.  **Error Handling**:
    - Catch exceptions and return clean JSON error responses.
    - Do not crash the server.

### Grid/Geometry Logic
- Logic resides in `GridEvaluatorLib/` or `dqglib/`.
- Use explicit types (e.g., `double` for coordinates, `uint8_t` for levels).

## 5. File Operations & Paths
- Configuration files (`config.json`, `region.json`) are loaded from:
    1. `../config.json` (build dir relative)
    2. `./config.json` (current dir)
    3. `/app/config.json` (Docker)
- Always check file existence before opening.

## 6. General Rules for Agents
- **NO `using namespace std;`**: Explicitly use `std::`.
- **Headers**: Use relative paths for local includes (`#include "../controller/..."`) and angle brackets for libs (`<drogon/drogon.h>`).
- **Safety**: Verify JSON input types (`asDouble()`, `asInt()`) before usage.
- **Refactoring**: When modifying `main.cc` or core logic, ensure `CMakeLists.txt` is updated if new files are added.

## 7. Copilot/Cursor Instructions
(From `.github/copilot-instructions.md`)
- [Include content from that file if it wasn't empty, but it appeared to be empty or minimal. Respect it if populated later.]
