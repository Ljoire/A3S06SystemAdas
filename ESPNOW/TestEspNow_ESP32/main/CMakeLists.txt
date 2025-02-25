# Programme principal ou test ?
set(USE_TEST_CODE ON)  # Mets à OFF pour compiler le programme principal

# Déclaration des fichiers sources et des dossiers d'inclusion
if(USE_TEST_CODE)
    set(SRCS "test/anglemort.c" 
             "test/main.c"
             "test/calculateur.c"
             "test/i2c_lcd.c"
             "test/BH1750.c")
    set(INCLUDE_DIRS "." "include" "test")
else()
    set(SRCS "calculateur.c" 
             "main.c"
             "hc_sr04.c"
             "i2c_lcd.c"
             "i2c_lcd2.c"
             "espnow_src.c")
    set(INCLUDE_DIRS "." "include" "test")
endif()

# Enregistrement unique du composant
idf_component_register(SRCS ${SRCS}
                       INCLUDE_DIRS ${INCLUDE_DIRS})
