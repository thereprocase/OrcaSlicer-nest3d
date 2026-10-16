# Runs the real orca-slicer binary, so it needs the built app and resources/, not just build/tests.
# The CI unit-test job only receives build/tests, so the test carries the RequiresApp label that
# scripts/run_unit_tests.sh excludes. Run it with `ctest -C Release -L RequiresApp`. It also exits 77
# (skipped) when the binary is missing.

find_program(ORCA_CLI_TEST_PYTHON NAMES python3)
if (NOT ORCA_CLI_TEST_PYTHON)
    message(STATUS "python3 not found, not registering the CLI tests")
    return()
endif ()

add_test(NAME cli_strict_mode
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_strict.sh $<TARGET_FILE:OrcaSlicer> ${ORCA_CLI_TEST_PYTHON})
add_test(NAME cli_empty_project_config
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_empty_project_config.sh $<TARGET_FILE:OrcaSlicer> ${ORCA_CLI_TEST_PYTHON} ${CMAKE_SOURCE_DIR})
set_tests_properties(cli_empty_project_config PROPERTIES
    LABELS "CLI;RequiresApp"
    SKIP_RETURN_CODE 77
    TIMEOUT 300)

set_tests_properties(cli_strict_mode PROPERTIES
    LABELS "CLI;RequiresApp"
    SKIP_RETURN_CODE 77
    TIMEOUT 900)

add_test(NAME cli_project_missing_keys
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_project_missing_keys.sh $<TARGET_FILE:OrcaSlicer> ${ORCA_CLI_TEST_PYTHON}
        ${CMAKE_SOURCE_DIR}/resources/profiles/BBL)
set_tests_properties(cli_project_missing_keys PROPERTIES
    LABELS "CLI;RequiresApp"
    SKIP_RETURN_CODE 77
    TIMEOUT 900)

add_test(NAME cli_malformed_input
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_malformed_input.sh $<TARGET_FILE:OrcaSlicer> ${ORCA_CLI_TEST_PYTHON})
set_tests_properties(cli_malformed_input PROPERTIES
    LABELS "CLI;RequiresApp"
    SKIP_RETURN_CODE 77
    TIMEOUT 900)

add_test(NAME cli_retraction_guard
    COMMAND ${ORCA_CLI_TEST_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/test_retraction_guard.py
        $<TARGET_FILE:OrcaSlicer> ${CMAKE_SOURCE_DIR}/resources/profiles/BBL)
set_tests_properties(cli_retraction_guard PROPERTIES
    LABELS "CLI;RequiresApp;RetractionLimiter"
    SKIP_RETURN_CODE 77
    TIMEOUT 900)

add_test(NAME cli_agent_profiles
    COMMAND ${ORCA_CLI_TEST_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/test_agent_profiles.py)
set_tests_properties(cli_agent_profiles PROPERTIES
    LABELS "CLI;RequiresApp;Profiles"
    ENVIRONMENT "ORCA_PROFILE_COMMAND=[\"$<TARGET_FILE:OrcaSlicer>\"]"
    TIMEOUT 120)

# Standalone transport coverage: no GUI, wxWidgets, cloud account or app resources.
find_package(Threads REQUIRED)
add_executable(local_agent_api_transport
    local_agent_api_transport.cpp
    ${CMAKE_SOURCE_DIR}/src/slic3r/GUI/LocalAgentApi.cpp)
target_include_directories(local_agent_api_transport PRIVATE
    ${CMAKE_SOURCE_DIR}/src/slic3r ${CMAKE_SOURCE_DIR}/deps_src)
target_compile_definitions(local_agent_api_transport PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(local_agent_api_transport PRIVATE boost_headeronly Threads::Threads)
if (WIN32)
    target_link_libraries(local_agent_api_transport PRIVATE ws2_32 mswsock)
endif ()
add_test(NAME cli_local_agent_api_transport
    COMMAND ${ORCA_CLI_TEST_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/test_local_agent_api_transport.py
        $<TARGET_FILE:local_agent_api_transport>)
set_tests_properties(cli_local_agent_api_transport PROPERTIES LABELS "CLI;LocalAgentApi" TIMEOUT 45)

add_test(NAME cli_agent_gui
    COMMAND ${ORCA_CLI_TEST_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/test_agent_gui.py)
set_tests_properties(cli_agent_gui PROPERTIES LABELS "CLI;LocalAgentApi" TIMEOUT 45)
