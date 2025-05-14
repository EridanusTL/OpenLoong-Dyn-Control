add_executable(pinocchio_test pinocchio_test.cc)

target_link_libraries(pinocchio_test core ${sysSimLibs})
