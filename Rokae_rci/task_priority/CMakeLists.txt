INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/inc/Eigen
    ${PROJECT_SOURCE_DIR}/inc/include
)
  
LINK_DIRECTORIES(
    /usr/lib/x86_64-linux-gun
    ${PROJECT_SOURCE_DIR}/lib
)

set(TASK_PRIORITY
  test_task_priority_admittance
  test_task_priority_impedance
  follow_task_priority_admittance
)

foreach(example ${TASK_PRIORITY})
  add_executable(${example} ${example}.cpp task_priority.cpp runner.cpp tcp_recv.cpp)
  TARGET_LINK_LIBRARIES(${example}  
    RCI_Client.a
    xMateModel.a
    orocos-kdl.a
    glog.a
    pthread
) 
endforeach()
