add_library(bsec
    src/bsec_integration.c
)

target_include_directories(bsec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_directories(bsec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_link_libraries(bsec
    bme680
    algobsec
    pico_stdlib
    hardware_i2c
)