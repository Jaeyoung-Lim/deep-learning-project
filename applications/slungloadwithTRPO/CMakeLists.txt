add_executable(quadrotor_TRPO
        ${RAI_TASK_SRC}
        slungload_TRPO.cpp)

target_include_directories(slungload_TRPO PUBLIC)
target_link_libraries(slungload_TRPO ${RAI_LINK})