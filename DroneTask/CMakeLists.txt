cmake_minimum_required(VERSION 3.10)
project(DroneTask)

set(CMAKE_CXX_STANDARD 17)

# 1. Підключаємо бібліотеку
list(APPEND CMAKE_PREFIX_PATH "C:/MAVSDK")
find_package(MAVSDK REQUIRED)

add_executable(DroneTask DroneTask.cpp)
target_link_libraries(DroneTask MAVSDK::mavsdk)

# ==========================================
# РОЗУМНА АВТОМАТИЗАЦІЯ (ПОШУК ПО ВСІХ ПАПКАХ)
# ==========================================

# GLOB_RECURSE шукає .dll файли у всіх вкладених папках (Release, lib, Debug тощо)
file(GLOB_RECURSE MAVSDK_DLLS "C:/MAVSDK/*.dll")

# Якщо DLL знайдено - копіюємо їх автоматично
if(MAVSDK_DLLS)
    add_custom_command(TARGET DroneTask POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${MAVSDK_DLLS}
        "$<TARGET_FILE_DIR:DroneTask>"
        COMMENT "Автоматичне копіювання DLL..."
    )
endif()

# Автоматично копіюємо Python-скрипт
add_custom_command(TARGET DroneTask POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/plot_graph.py"
    "$<TARGET_FILE_DIR:DroneTask>"
    COMMENT "Автоматичне копіювання plot_graph.py..."
)