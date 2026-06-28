cmake_minimum_required(VERSION 3.22)
project(TheoremProver VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT MSVC)
    add_compile_options(
        -Wall 
        -Wextra 
        -Werror 
        -pedantic 
        -O3 
        -march=native
    )
endif()


add_library(solver_core
    src/ast.cpp
    src/parser.cpp
    src/sat_core.cpp
)

target_include_directories(solver_core PUBLIC include)
add_executable(theorem_prover src/main.cpp)

target_link_libraries(theorem_prover PRIVATE solver_core)