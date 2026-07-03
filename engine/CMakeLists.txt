cmake_minimum_required(VERSION 3.18)
project(moto_engine VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

# ── Compiler flags ────────────────────────────────────────────────────────────
add_compile_options(-O3 -march=native -Wall -Wextra -Wno-unused-parameter)

# ── ONNX Runtime (optional) ───────────────────────────────────────────────────
# Set ORT_ROOT to the extracted ONNX Runtime directory, e.g.:
#   cmake -DORT_ROOT=/tmp/ort_headers ..
set(ORT_ROOT "" CACHE PATH "Path to ONNX Runtime install dir (optional)")

set(HAVE_ORT OFF)
if(ORT_ROOT)
    find_path(ORT_INCLUDE onnxruntime_cxx_api.h HINTS "${ORT_ROOT}/include" NO_DEFAULT_PATH)
    find_library(ORT_LIB onnxruntime HINTS "${ORT_ROOT}/lib" NO_DEFAULT_PATH)
    if(ORT_INCLUDE AND ORT_LIB)
        set(HAVE_ORT ON)
        message(STATUS "ONNX Runtime found: ${ORT_LIB}")
    else()
        message(WARNING "ORT_ROOT set but headers/lib not found — building without ONNX Runtime")
    endif()
endif()

# ── Source files ──────────────────────────────────────────────────────────────
set(ENGINE_SRCS
    src/Kinematics.cpp
    src/EOM.cpp
    src/PacejkaTire.cpp
    src/OnnxTire.cpp
    src/RoadParams.cpp
    src/FullMFTire.cpp       # Phase 2.1 — full motorcycle Pacejka (large camber, gyro Mz, turn-slip)
    src/Phase2Tire.cpp       # Phase 2.1+2.2+2.3 — FullMF + LuGre bristle + Hertzian contact patch
    src/PINNCoeffFitter.cpp  # Phase 3.1 — real-time PINN coefficient fitting
)

# ── Library target ────────────────────────────────────────────────────────────
add_library(moto_engine_lib STATIC ${ENGINE_SRCS})
target_include_directories(moto_engine_lib PUBLIC include)

if(HAVE_ORT)
    target_include_directories(moto_engine_lib PUBLIC ${ORT_INCLUDE})
    target_link_libraries(moto_engine_lib PUBLIC ${ORT_LIB})
    target_compile_definitions(moto_engine_lib PUBLIC HAVE_ONNXRUNTIME)
endif()

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(moto_engine main.cpp)
target_link_libraries(moto_engine PRIVATE moto_engine_lib)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS moto_engine DESTINATION bin)
install(TARGETS moto_engine_lib DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# ── Summary ───────────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "=== moto_engine build configuration ===")
message(STATUS "  Build type  : ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  ONNX Runtime: ${HAVE_ORT}")
if(HAVE_ORT)
    message(STATUS "    ORT headers: ${ORT_INCLUDE}")
    message(STATUS "    ORT library: ${ORT_LIB}")
endif()
message(STATUS "")
