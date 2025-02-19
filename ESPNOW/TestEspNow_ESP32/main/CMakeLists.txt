set(ENABLE_CODE OFF)  # Change à ON pour activer
if(ENABLE_CODE)

       idf_component_register(SRCS "calculateur.c" "main.c"
                            "hc_sr04.c"
                            "i2c_lcd.c"
                            "i2c_lcd2.c"
                            "espnow_src.c"
                     INCLUDE_DIRS "." "include" "test")
endif

set(ENABLE_CODE ON)  # Change à ON pour activer
if(ENABLE_CODE)
       idf_component_register(SRCS "test/anglemort.c" 
                                   "test/main.c"
                            INCLUDE_DIRS "." "include" "test")
endif