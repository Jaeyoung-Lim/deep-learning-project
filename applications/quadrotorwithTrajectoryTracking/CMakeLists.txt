add_executable(quadrotor_Tracking
        ${RAI_TASK_SRC}
        quadrotor_Tracking.cpp)

target_include_directories(quadrotor_Tracking PUBLIC)
target_link_libraries(quadrotor_Tracking ${RAI_LINK})