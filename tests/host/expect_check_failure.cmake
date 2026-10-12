if (NOT DEFINED TEST_PROGRAM)
    message(FATAL_ERROR "TEST_PROGRAM is required")
endif ()

execute_process(
    COMMAND "${TEST_PROGRAM}" --intentional-failure
    RESULT_VARIABLE TEST_RESULT
    OUTPUT_VARIABLE TEST_STDOUT
    ERROR_VARIABLE TEST_STDERR
)

if (NOT TEST_RESULT EQUAL 1)
    message(FATAL_ERROR "TEST_CHECK must exit with status 1; got '${TEST_RESULT}'\n${TEST_STDOUT}\n${TEST_STDERR}")
endif ()

if (NOT TEST_STDERR MATCHES "CHECK failed: 0 && \\\"intentional failure control\\\"")
    message(FATAL_ERROR "TEST_CHECK failure diagnostic missing\n${TEST_STDERR}")
endif ()
