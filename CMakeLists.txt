cmake_minimum_required(VERSION 3.12)
project(NoCDAS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(CNOC_QUANT_GOLDEN "Enable compile-time INT8 cNoC quantized golden trace mode" OFF)
option(RTL_DEBUG_MATMUL "Enable guarded RTL MatMul/Linear debug displays in the Verilated router" OFF)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(RTL_ROUTER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/rtl/router")
set(RTL_ROUTER_FILELIST "${RTL_ROUTER_ROOT}/filelist.f")

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp" "src/NoC/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "src/*.hpp" "src/NoC/*.hpp")
list(FILTER SOURCES EXCLUDE REGEX ".*/src/rtl/verilated_router/.*")
list(FILTER HEADERS EXCLUDE REGEX ".*/src/rtl/verilated_router/.*")

execute_process(
    COMMAND verilator --getenv VERILATOR_ROOT
    OUTPUT_VARIABLE VERILATOR_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(VERILATOR_ROOT AND EXISTS "${VERILATOR_ROOT}/include/verilated.cpp")
    set(VERILATED_ROUTER_DIR "${CMAKE_CURRENT_BINARY_DIR}/rtl/verilated_router")
    set(VERILATED_ROUTER_STAMP "${VERILATED_ROUTER_DIR}/VNoCRouter.stamp")
    set(VERILATED_ROUTER_MODEL_LIB "${VERILATED_ROUTER_DIR}/libVNoCRouter.a")
    set(VERILATED_ROUTER_RUNTIME_LIB "${VERILATED_ROUTER_DIR}/libverilated.a")
    set(VERILATED_ROUTER_ALL_LIB "${VERILATED_ROUTER_DIR}/VNoCRouter__ALL.a")
    set(VERILATED_ROUTER_DEFINES "-DROUTER_ENABLE_COSIM")
    if(RTL_DEBUG_MATMUL)
        list(APPEND VERILATED_ROUTER_DEFINES "-DRTL_DEBUG_MATMUL")
    endif()

    file(STRINGS "${RTL_ROUTER_FILELIST}" RTL_ROUTER_FILELIST_ENTRIES)
    set(RTL_ROUTER_DEPENDS "${RTL_ROUTER_FILELIST}")
    foreach(RTL_FILE_ENTRY ${RTL_ROUTER_FILELIST_ENTRIES})
        string(STRIP "${RTL_FILE_ENTRY}" RTL_FILE_ENTRY)
        if(RTL_FILE_ENTRY AND NOT RTL_FILE_ENTRY MATCHES "^#")
            list(APPEND RTL_ROUTER_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${RTL_FILE_ENTRY}")
        endif()
    endforeach()

    add_custom_command(
        OUTPUT "${VERILATED_ROUTER_STAMP}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${VERILATED_ROUTER_DIR}"
        COMMAND verilator --cc
                ${VERILATED_ROUTER_DEFINES}
                --Mdir "${VERILATED_ROUTER_DIR}"
                --prefix VNoCRouter
                -Wno-WIDTHEXPAND
                -f "${RTL_ROUTER_FILELIST}"
                --top-module Router
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMAND "${CMAKE_COMMAND}" -E touch "${VERILATED_ROUTER_STAMP}"
        DEPENDS ${RTL_ROUTER_DEPENDS}
        COMMENT "Generating Verilated RTL router model"
        VERBATIM
    )

    add_custom_command(
        OUTPUT
            "${VERILATED_ROUTER_MODEL_LIB}"
            "${VERILATED_ROUTER_RUNTIME_LIB}"
            "${VERILATED_ROUTER_ALL_LIB}"
        COMMAND "${CMAKE_COMMAND}" -E env OBJCACHE= make -C "${VERILATED_ROUTER_DIR}" -f VNoCRouter.mk libVNoCRouter
        DEPENDS "${VERILATED_ROUTER_STAMP}"
        COMMENT "Building Verilated RTL router archive"
        VERBATIM
    )

    add_custom_target(verilated_router_model
        DEPENDS
            "${VERILATED_ROUTER_MODEL_LIB}"
            "${VERILATED_ROUTER_RUNTIME_LIB}"
            "${VERILATED_ROUTER_ALL_LIB}"
    )
endif()

add_executable(NoCDASim ${SOURCES} ${HEADERS})

target_include_directories(NoCDASim PRIVATE src src/NoC src/input)

if(CNOC_QUANT_GOLDEN)
    target_compile_definitions(NoCDASim PRIVATE CNOC_QUANT_GOLDEN=1)
endif()

if(VERILATOR_ROOT AND EXISTS "${VERILATOR_ROOT}/include")
    add_dependencies(NoCDASim verilated_router_model)
    target_compile_definitions(NoCDASim PRIVATE ROUTER_ENABLE_COSIM=1)
    target_include_directories(NoCDASim PRIVATE
        "${VERILATOR_ROOT}/include"
        "${VERILATOR_ROOT}/include/vltstd"
        "${CMAKE_CURRENT_BINARY_DIR}/rtl/verilated_router"
    )
    target_link_libraries(NoCDASim PRIVATE
        "${VERILATED_ROUTER_MODEL_LIB}"
        "${VERILATED_ROUTER_RUNTIME_LIB}"
        pthread
        dl
    )
endif()
