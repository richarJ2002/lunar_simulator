cmake_minimum_required(VERSION 3.16)

# Project-wide support modules shared by localisation, control and every
# system; each depends on nothing else under src/.
add_subdirectory(console)
add_subdirectory(diagnostics)
