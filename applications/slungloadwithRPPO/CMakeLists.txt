add_executable(slungload_RPPO
        ${RAI_TASK_SRC}
        slungload_RPPO.cpp)

target_include_directories(slungload_RPPO PUBLIC)
target_link_libraries(slungload_RPPO ${RAI_LINK})