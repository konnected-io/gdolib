# ESP-IDF 5.3 split the legacy "driver" component into per-peripheral
# esp_driver_* components, and IDF 6 no longer pulls them in through "driver".
# Require the split components where they exist so gdo.h's <driver/uart.h> and
# <driver/gpio.h> resolve on IDF 6.x, and keep "driver" for older releases.
if(IDF_VERSION_MAJOR GREATER 5 OR (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR GREATER_EQUAL 3))
    set(gdo_driver_requires esp_driver_uart esp_driver_gpio)
else()
    set(gdo_driver_requires driver)
endif()

idf_component_register(SRCS
                            "gdo.c"
                            "gdo_utils.c"
                            "secplus.c"
                       INCLUDE_DIRS
                            "include"
                       REQUIRES
                            ${gdo_driver_requires}
                            esp_timer
)
