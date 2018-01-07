add_executable(slungload_PPO
        ${RAI_TASK_SRC}
        slungload_PPO.cpp)

target_include_directories(slungload_PPO PUBLIC)
target_link_libraries(slungload_PPO ${RAI_LINK})