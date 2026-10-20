# 自有C源码的格式告警：固件和主机共用；先编译正反例，再为实际源码添加选项。
# 检查在project之后调用，应用选项在目标及其源码声明完成后调用；报告只写构建目录。
include_guard(GLOBAL)
get_filename_component(MC02_FORMAT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(MC02_FORMAT_FLAGS
  -Wformat=2
  -Wformat-overflow=2
  -Wformat-truncation=2
  -Werror=format
  -Werror=format-security
  -Werror=format-nonliteral
  -Werror=format-overflow
  -Werror=format-truncation)

# 配置期间拒绝失效的格式检查；MinGW长度诊断不足由脚本单独报告，不能写成已通过。
function(mc02_check_format_diagnostics)
  if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "格式检查目前只验证GCC；新编译器须先补正反例")
  endif()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  execute_process(COMMAND "${Python3_EXECUTABLE}"
    "${MC02_FORMAT_ROOT}/tools/workflow/check_format_diagnostics.py"
    --compiler "${CMAKE_C_COMPILER}" --output "${CMAKE_BINARY_DIR}/format-diagnostics"
    --optimization=-O2
    RESULT_VARIABLE result OUTPUT_VARIABLE report ERROR_VARIABLE errors)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "格式检查正反例失败：\n${report}\n${errors}")
  endif()
  string(STRIP "${report}" report)
  message(STATUS "${report}")
endfunction()

# 单个目标可能同时包含厂商文件；只给仓库自有目录中的.c添加选定告警，避免改动第三方策略。
function(mc02_apply_format_diagnostics target)
  get_target_property(sources ${target} SOURCES)
  get_property(seen GLOBAL PROPERTY MC02_FORMAT_SEEN_SOURCES)
  set(has_own_source FALSE)
  foreach(source IN LISTS sources)
    get_filename_component(absolute "${source}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    file(RELATIVE_PATH relative "${MC02_FORMAT_ROOT}" "${absolute}")
    if(relative MATCHES "^(bsp|modules|application|Core/Src|USB_DEVICE|tests)/.*\\.c$")
      set(has_own_source TRUE)
      if(NOT absolute IN_LIST seen)
        set_property(SOURCE "${absolute}" APPEND PROPERTY COMPILE_OPTIONS ${MC02_FORMAT_FLAGS})
        list(APPEND seen "${absolute}")
      endif()
    endif()
  endforeach()
  if(has_own_source)
    target_include_directories(${target} PRIVATE "${MC02_FORMAT_ROOT}/bsp/log")
  endif()
  set_property(GLOBAL PROPERTY MC02_FORMAT_SEEN_SOURCES "${seen}")
endfunction()
