cmake_minimum_required(VERSION 3.12)
project(Ws15eP2_Test C)

# This CMakeLists.txt is for testing purposes only.
add_compile_definitions(TESTING)

# Collect all source files
set(SOURCES
Ws15eP2.c)
add_executable(Ws15eP2_Test ${SOURCES} Ws15eP2_Test.c)

# Setup testing
enable_testing()
include(CTest)

add_test(
    NAME Ws15eP2_UnitTest
    COMMAND Ws15eP2_Test
)
