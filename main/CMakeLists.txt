
idf_component_register(SRCS "router_example.c" "camera_driver.c"
                    INCLUDE_DIRS ""
                    REQUIRES mcommon mconfig mwifi mlink json driver sd_card
)
