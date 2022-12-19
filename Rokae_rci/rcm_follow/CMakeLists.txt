INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/inc/Eigen
    ${PROJECT_SOURCE_DIR}/inc/include
)
  
LINK_DIRECTORIES(
    /usr/lib/x86_64-linux-gun
    ${PROJECT_SOURCE_DIR}/lib
)

set(ADMITTANCES
  admittance_test
)

foreach(example ${ADMITTANCES})
  add_executable(${example} ${example}.cpp admittance.cpp runner.cpp)
  TARGET_LINK_LIBRARIES(${example}  
    RCI_Client.a
    xMateModel.a
    orocos-kdl.a
    glog.a
    pthread
) 
endforeach()

set(ADMITTANCES_IMPROVED
  admittance_improved_test
)

foreach(example ${ADMITTANCES_IMPROVED})
  add_executable(${example} ${example}.cpp admittance_improved.cpp runner_improved.cpp)
  TARGET_LINK_LIBRARIES(${example}  
    RCI_Client.a
    xMateModel.a
    orocos-kdl.a
    glog.a
    pthread
) 
endforeach()

