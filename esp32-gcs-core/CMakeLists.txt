cmake_minimum_required(VERSION 3.20)
project(DroneGroundStation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
        SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG        release-2.30.0
)
FetchContent_MakeAvailable(SDL2)

# Adăugăm noile fișiere .cpp
add_executable(DroneGroundStation
        src/main.cpp
        src/MavlinkManager.cpp
        src/GuiManager.cpp
)

target_include_directories(DroneGroundStation PRIVATE src src/mavlink)
target_link_libraries(DroneGroundStation PRIVATE SDL2::SDL2)

if(WIN32)
    add_custom_command(TARGET DroneGroundStation POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:SDL2::SDL2>"
            "$<TARGET_FILE_DIR:DroneGroundStation>")
endif()