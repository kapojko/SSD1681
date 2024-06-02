cmake_minimum_required(VERSION 3.12)
project(SSD1681_Test C)

# This CMakeLists.txt is for testing purposes only.
add_compile_definitions(TESTING)

# Specify the C standard
set(CMAKE_C_STANDARD 11)

# Specify the warning flags
if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall)
endif()

# Collect all source files
set(SOURCES
    SSD1681.c)
add_executable(SSD1681_Test ${SOURCES} SSD1681_Test.c)

# Setup testing
enable_testing()
include(CTest)

add_test(
    NAME SSD1681_UnitTest
    COMMAND SSD1681_Test
)
