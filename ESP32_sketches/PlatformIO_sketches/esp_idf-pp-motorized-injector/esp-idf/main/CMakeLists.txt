idf_component_register(
    SRCS
        "app_main.cpp"
        "time_utils.cpp"
        "broadcast_data_store.cpp"
        "can_bus.cpp"
        "can_rx.cpp"
        "motor_wrapper.cpp"
        "safety_manager.cpp"
        "debounced_input.cpp"
        "hx711.cpp"
        "temperature.cpp"
        "leds.cpp"
        "ws2812.c"
        "ws2812_encoder.c"
        "homing.cpp"
        "refill.cpp"
        "compression.cpp"
        "ready_to_inject.cpp"
        "purge_zero.cpp"
        "antidrip.cpp"
        "injection.cpp"
        "error_manager.cpp"
        "debug_console.cpp"
        "display_comms.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_rmt esp_driver_gpio esp_driver_twai esp_driver_uart esp_timer esp_adc driver
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
