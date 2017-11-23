add_executable(TRPO
        ${RAI_TASK_SRC}
        main.cpp)
target_include_directories(TRPO PUBLIC)
target_link_libraries(TRPO ${RAI_LINK})
