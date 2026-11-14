# 编译缓存是可选项：找到 ccache 才启用，找不到时保持原始编译命令。
# 这样 CI 缓存命中时减少重复编译，本机没装 ccache 也不会改变构建结果。
option(MC02_USE_CCACHE "找到 ccache 时缓存 C 编译结果" ON)

function(mc02_enable_ccache)
    if(NOT MC02_USE_CCACHE)
        return()
    endif()
    find_program(MC02_CCACHE_PROGRAM ccache)
    if(MC02_CCACHE_PROGRAM)
        set(CMAKE_C_COMPILER_LAUNCHER "${MC02_CCACHE_PROGRAM}" PARENT_SCOPE)
        message(STATUS "MC-02: 使用 ccache ${MC02_CCACHE_PROGRAM}")
    endif()
endfunction()
