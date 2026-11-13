cmake_minimum_required(VERSION 3.20)
project(wheelleg_c_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# The MuJoCo SDK is supplied by the active Python installation.  Keeping the
# path configurable means a standalone MuJoCo SDK can be used later as well.
set(MUJOCO_ROOT "D:/python/Lib/site-packages/mujoco" CACHE PATH
    "MuJoCo SDK root (contains include/ and mujoco.dll)")

add_executable(wheelleg_c_sim
    wheelleg_sim.c
    ground_detection.c
    ../embedded_mpc/explicit_mpc.c)

target_include_directories(wheelleg_c_sim PRIVATE
    "${MUJOCO_ROOT}/include/mujoco"
    "${CMAKE_CURRENT_LIST_DIR}/../embedded_mpc")
target_link_libraries(wheelleg_c_sim PRIVATE "${MUJOCO_ROOT}/mujoco.dll" m)
if (WIN32)
    target_link_libraries(wheelleg_c_sim PRIVATE opengl32 gdi32 user32)
endif()

add_custom_command(TARGET wheelleg_c_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MUJOCO_ROOT}/mujoco.dll" "$<TARGET_FILE_DIR:wheelleg_c_sim>/mujoco.dll")
