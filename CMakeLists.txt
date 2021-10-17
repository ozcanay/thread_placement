cmake_minimum_required(VERSION 3.10)
project(placer CXX)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(spdlog REQUIRED)

add_executable(${PROJECT_NAME} 
main.cpp
thread_helper.cpp
intel_helper.cpp
logger_helper.cpp
cpu_helper.cpp
string_helper.cpp
)

#target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -pedantic -Werror)

target_link_libraries(${PROJECT_NAME} PRIVATE
    pthread
    spdlog::spdlog
)

install(FILES
    thft_core_topo.txt
    thft_cha_topo.txt
    automate.sh
    DESTINATION
    ${CMAKE_SOURCE_DIR}/build)