cmake_minimum_required(VERSION 3.5)
project(2019_Raspberry) # Project Name Here

find_package(wpilib REQUIRED)

add_executable(2019_Raspberry.exe main.cpp) # exectuable name as first parameter
target_link_libraries(2019_Raspberry.exe ntcore cscore wpiutil)
