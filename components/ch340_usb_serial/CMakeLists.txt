idf_component_register(
    SRCS "ch340_usb_serial.cpp"
    INCLUDE_DIRS "include"
    REQUIRES usb usb_host_cdc_acm usb_host_ch34x_vcp
)
