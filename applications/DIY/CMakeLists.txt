add_executable(DIY_example
        ${RAI_TASK_SRC}
        run.cpp)
target_include_directories(DIY_example PUBLIC)
target_link_libraries(DIY_example ${RAI_LINK})
