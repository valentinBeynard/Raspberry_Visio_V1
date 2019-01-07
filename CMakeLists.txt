cmake_minimum_required(VERSION 3.5)
project(full_processing2) # Project Name Here

find_package(wpilib REQUIRED)

add_executable(full_processing2.exe main.cpp) # exectuable name as first parameter
target_link_libraries(full_processing2.exe ntcore cscore wpiutil)