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
  task_priority_test
)

foreach(example ${TASK_PRIORITY})
  add_executable(${example} ${example}.cpp task_priority.cpp runner.cpp)
  TARGET_LINK_LIBRARIES(${example}  
    RCI_Client.a
    xMateModel.a
    orocos-kdl.a
    glog.a
    pthread
) 
endforeach()
