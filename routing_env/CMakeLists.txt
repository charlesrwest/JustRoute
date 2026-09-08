cmake_minimum_required(VERSION 3.20)
project(routing_env LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(ROUTING_ENV_BUILD_TESTS "Build the doctest unit tests" ON)
option(ROUTING_ENV_BUILD_PYTHON "Build the pybind11 Python module" ON)

find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(routing_env_core
  src/grid.cpp
  src/astar.cpp
  src/cost_model.cpp
  src/net.cpp
  src/board.cpp
  src/synthetic.cpp
  src/pcb_rdl.cpp
  src/kicad_pcb.cpp
  src/env.cpp
  src/wave_gpu.cpp
)
# Optional in-process GPU wavefront: requires vulkan headers + glslc at BUILD
# time only (Vulkan itself is dlopen'd at runtime — no link dependency; absent
# loader/device => CPU fallback). Without the toolchain, wave_gpu.cpp builds
# as a stub reporting unavailable.
find_program(GLSLC glslc)
include(CheckIncludeFileCXX)
check_include_file_cxx("vulkan/vulkan.h" HAVE_VULKAN_H)
if(GLSLC AND HAVE_VULKAN_H)
  set(WAVE_SPV ${CMAKE_CURRENT_BINARY_DIR}/wave_step_map.spv)
  set(WAVE_SPV_H ${CMAKE_CURRENT_BINARY_DIR}/wave_spv.h)
  add_custom_command(OUTPUT ${WAVE_SPV}
    COMMAND ${GLSLC} ${CMAKE_CURRENT_SOURCE_DIR}/tools/wave_step_map.comp -o ${WAVE_SPV}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/wave_step_map.comp)
  add_custom_command(OUTPUT ${WAVE_SPV_H}
    COMMAND ${CMAKE_COMMAND} -DIN=${WAVE_SPV} -DOUT=${WAVE_SPV_H}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spv.cmake
    DEPENDS ${WAVE_SPV} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spv.cmake)
  set(FIELD_SPV ${CMAKE_CURRENT_BINARY_DIR}/field_sweep.spv)
  set(FIELD_SPV_H ${CMAKE_CURRENT_BINARY_DIR}/field_spv.h)
  add_custom_command(OUTPUT ${FIELD_SPV}
    COMMAND ${GLSLC} ${CMAKE_CURRENT_SOURCE_DIR}/tools/field_sweep.comp -o ${FIELD_SPV}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/field_sweep.comp)
  add_custom_command(OUTPUT ${FIELD_SPV_H}
    COMMAND ${CMAKE_COMMAND} -DIN=${FIELD_SPV} -DOUT=${FIELD_SPV_H}
            -DSYM=g_field_spv
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spv.cmake
    DEPENDS ${FIELD_SPV} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spv.cmake)
  set(BAKE_SPV ${CMAKE_CURRENT_BINARY_DIR}/bake_cost.spv)
  set(BAKE_SPV_H ${CMAKE_CURRENT_BINARY_DIR}/bake_spv.h)
  add_custom_command(OUTPUT ${BAKE_SPV}
    COMMAND ${GLSLC} ${CMAKE_CURRENT_SOURCE_DIR}/tools/bake_cost.comp -o ${BAKE_SPV}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/bake_cost.comp)
  add_custom_command(OUTPUT ${BAKE_SPV_H}
    COMMAND ${CMAKE_COMMAND} -DIN=${BAKE_SPV} -DOUT=${BAKE_SPV_H}
            -DSYM=g_bake_spv
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spv.cmake
    DEPENDS ${BAKE_SPV} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spv.cmake)
  add_custom_target(wave_spv_header DEPENDS ${WAVE_SPV_H} ${FIELD_SPV_H} ${BAKE_SPV_H})
  add_dependencies(routing_env_core wave_spv_header)
  set_source_files_properties(src/wave_gpu.cpp PROPERTIES
    COMPILE_DEFINITIONS ROUTING_HAVE_WAVE_GPU=1)
  target_include_directories(routing_env_core PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(routing_env_core PUBLIC ${CMAKE_DL_LIBS})
endif()
target_include_directories(routing_env_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(routing_env_core SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
# Optional row-parallel wavefront (single-board latency): threads split rows
# inside one wave step. Off by default at runtime; see set_wave_threads.
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  target_link_libraries(routing_env_core PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(routing_env_core PUBLIC ROUTING_HAVE_OPENMP=1)
endif()
target_compile_options(routing_env_core PRIVATE -Wall -Wextra -O2)
# The static core lib is linked into the shared pybind module, so it must be PIC.
set_target_properties(routing_env_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Tests (doctest, header-only)
# ---------------------------------------------------------------------------
if(ROUTING_ENV_BUILD_TESTS)
  enable_testing()
  set(DOCTEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/doctest)

  # Helper: one test binary per test file; each defines its own main
  function(routing_env_add_test name)
    add_executable(${name} tests/${name}.cpp)
    target_link_libraries(${name} PRIVATE routing_env_core)
    target_include_directories(${name} PRIVATE ${DOCTEST_DIR})
    target_compile_definitions(${name} PRIVATE
      ROUTING_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
    add_test(NAME ${name} COMMAND ${name})
  endfunction()

  routing_env_add_test(test_grid)
  routing_env_add_test(test_astar)
  routing_env_add_test(test_net_routing)
  routing_env_add_test(test_cost_model)
  routing_env_add_test(test_board)
  routing_env_add_test(test_pcb_rdl)
  routing_env_add_test(test_kicad_pcb)
  routing_env_add_test(test_env)
  routing_env_add_test(test_perf)
endif()

# ---------------------------------------------------------------------------
# Python bindings (pybind11)
# ---------------------------------------------------------------------------
if(ROUTING_ENV_BUILD_PYTHON AND Python3_FOUND)
  find_package(pybind11 CONFIG QUIET)
  if(pybind11_FOUND)
    pybind11_add_module(routing_env _core.cpp)
    # wheel packaging (scikit-build-core): place the module at the wheel root
    install(TARGETS routing_env LIBRARY DESTINATION . COMPONENT python)
    target_link_libraries(routing_env PRIVATE routing_env_core)
    target_include_directories(routing_env PRIVATE include)
  else()
    message(WARNING "pybind11 not found; skipping Python module build")
  endif()
endif()
