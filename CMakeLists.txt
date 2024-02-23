set(LIBRARY_NAME INA233)

add_library(${LIBRARY_NAME} INA233.cpp)

target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)