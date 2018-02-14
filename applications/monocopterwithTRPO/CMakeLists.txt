add_executable(monocopter_TRPO
        ${RAI_TASK_SRC}
        monocopter_TRPO.cpp)

target_include_directories(monocopter_TRPO PUBLIC)
target_link_libraries(monocopter_TRPO ${RAI_LINK})