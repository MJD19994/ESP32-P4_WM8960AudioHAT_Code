idf_component_register(
    SRCS "wm8960.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus             # I2C driver dependency
             cmake_utilities     # CMake utilities dependency
)