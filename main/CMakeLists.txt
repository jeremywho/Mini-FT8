idf_component_register(
    SRCS "main.cpp"
         "autoseq.cpp"
         "audio_source.cpp"
         "gps.cpp"
         "radio_control.cpp"
         "radio_control_qmx.cpp"
         "radio_control_kh1_cat.cpp"
         "radio_trusdx.cpp"
         "audio_trusdx_serial.cpp"
         "ft8_audio_pipeline.cpp"
         "stream_uac.cpp"
         "stream_mic.cpp"
         "resample.cpp"
    INCLUDE_DIRS "."
    REQUIRES ui M5Unified M5GFX M5Cardputer board_cardputer_adv ft8_lib usb_host_uac usb_host_cdc_acm ch340_usb_serial fatfs sdmmc bt nvs_flash efuse
    PRIV_REQUIRES spiffs
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)
