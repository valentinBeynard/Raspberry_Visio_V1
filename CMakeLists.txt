cmake_minimum_required(VERSION 3.5)
project(2019_Raspberry) # Project Name Here

find_package(wpilib REQUIRED)

set(SRCS_DIR
    src/
    )

set(SRCS
    ${SRCS_DIR}main.cpp
    ${SRCS_DIR}hud.cpp
    ${SRCS_DIR}TopView.cpp
    ${SRCS_DIR}VisionTarget.cpp
    ${SRCS_DIR}VisionTargetPair.cpp
    ${SRCS_DIR}Vision.cpp
    ${SRCS_DIR}Tools.cpp
    ${SRCS_DIR}Spline.cpp
    )
    
set(HEADERS
    ${SRCS_DIR}Spline.h          
    ${SRCS_DIR}hud.h
    ${SRCS_DIR}Statistics.cpp
    ${SRCS_DIR}TopView.h
    ${SRCS_DIR}VisionTarget.h
    ${SRCS_DIR}Statistics.h
    ${SRCS_DIR}Values.h
    ${SRCS_DIR}PrecompilationDefine.h
    ${SRCS_DIR}VisionTargetPair.h
    ${SRCS_DIR}Tools.h
    ${SRCS_DIR}Vision.h
    )

add_executable(2019_Raspberry.exe ${SRCS} ${HEADERS}) # exectuable name as first parameter
target_link_libraries(2019_Raspberry.exe ntcore cscore wpiutil)
