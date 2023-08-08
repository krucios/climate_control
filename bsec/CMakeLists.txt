set(BSEC_CONFIG bme680_iaq_33v_300s_4d)

add_library(bsec
    src/bsec_integration.c
    config/${BSEC_CONFIG}/bsec_iaq.c
)

target_include_directories(bsec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/config/${BSEC_CONFIG})
target_link_directories(bsec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_link_libraries(bsec
    bme680
    algobsec
    pico_stdlib
    hardware_i2c
    hardware_sync
    hardware_flash
)