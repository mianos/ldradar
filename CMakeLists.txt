idf_component_register(
    SRCS
        "src/RadarSensor.cpp"
        "src/DebounceRadar.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_timer
        log
        jsonwrapper
)
# target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_23)

