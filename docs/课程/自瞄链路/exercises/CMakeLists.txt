cmake_minimum_required(VERSION 3.16)
project(mc02_vision_learning_exercises LANGUAGES CXX)

include(CTest)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

function(add_course_exercise name source)
  add_executable(${name} "${source}")
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_course_exercise(exercise_sp_frame_crc "01-sp-frame-crc/solution/main.cpp")
add_course_exercise(exercise_usb_cdc "02-usb-cdc/solution/main.cpp")
add_course_exercise(exercise_vision_timeout "03-vision-timeout-revoke/solution/main.cpp")
add_course_exercise(exercise_wsl_bridge "04-windows-wsl-bridge/solution/main.cpp")
