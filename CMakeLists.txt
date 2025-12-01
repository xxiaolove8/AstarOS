cmake_minimum_required(VERSION 4.0)
project(AstarOS C)

set(CMAKE_C_STANDARD 11)

add_executable(AstarOS main.c
        PathAPI.c
        AStar.c
        SerialWin.c
        SerialWin.h
        #ASTAR_DEMO.c
)
