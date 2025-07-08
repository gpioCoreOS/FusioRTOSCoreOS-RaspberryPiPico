add_executable(blackzcoreos
        blackzcoreos.c
        lcd_1602_i2c.c 
)

target_link_libraries(blackzcoreos pico_stdlib hardware_i2c freertos)
pico_add_extra_outputs(blackzcoreos)
