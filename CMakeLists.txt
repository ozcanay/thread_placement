cmake_minimum_required(VERSION 3.10)
project(placer CXX)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(spdlog REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    message(STATUS "openmp found!")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(${PROJECT_NAME} 
main.cpp
thread_helper.cpp
intel_helper.cpp
logger_helper.cpp
cpu_helper.cpp
string_helper.cpp
cha_helper.cpp
imc_helper.cpp
math_helper.cpp
os_helper.cpp
pci_helper.cpp
)

#target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -pedantic -Werror)

target_link_libraries(${PROJECT_NAME} PRIVATE
    pthread
    spdlog::spdlog
)

install(FILES
    thft_core_topo.txt
    thft_cha_topo.txt
    BaseSequence_SKX_18-slice.txt
    automate.sh
    DESTINATION
    ${CMAKE_SOURCE_DIR}/build)