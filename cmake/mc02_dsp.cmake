# 固定 DSP 最小源码清单，供固件和主机复用；不下载文件、不搜索旧预编译库。
set(MC02_DSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../Middlewares/Third_Party/CMSIS-DSP-1.7.0")
set(MC02_DSP_SOURCES
  ${MC02_DSP_ROOT}/Source/MatrixFunctions/arm_mat_init_f32.c
  ${MC02_DSP_ROOT}/Source/MatrixFunctions/arm_mat_add_f32.c
  ${MC02_DSP_ROOT}/Source/MatrixFunctions/arm_mat_sub_f32.c
  ${MC02_DSP_ROOT}/Source/MatrixFunctions/arm_mat_mult_f32.c
  ${MC02_DSP_ROOT}/Source/MatrixFunctions/arm_mat_trans_f32.c
  ${MC02_DSP_ROOT}/Source/MatrixFunctions/arm_mat_inverse_f32.c
  ${MC02_DSP_ROOT}/Source/FastMathFunctions/arm_sin_f32.c
  ${MC02_DSP_ROOT}/Source/FastMathFunctions/arm_cos_f32.c
  ${MC02_DSP_ROOT}/Source/CommonTables/arm_common_tables.c)

# host 为真时使用该版本自带的 GCC 可移植分支，仅相关测试继承头文件所需宏。
function(mc02_add_dsp target host)
  add_library(${target} STATIC ${MC02_DSP_SOURCES})
  target_include_directories(${target} PUBLIC ${MC02_DSP_ROOT}/Include)
  target_compile_definitions(${target} PRIVATE ARM_MATH_MATRIX_CHECK ARM_DSP_CONFIG_TABLES
    ARM_FAST_ALLOW_TABLES ARM_TABLE_SIN_F32)
  target_compile_options(${target} PRIVATE -fno-fast-math -ffunction-sections -fdata-sections)
  if(host)
    target_compile_definitions(${target} PUBLIC __GNUC_PYTHON__)
  else()
    target_include_directories(${target} PRIVATE ${MC02_DSP_ROOT}/../../../Drivers/CMSIS/Include)
  endif()
endfunction()
