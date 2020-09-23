set(SOURCES st7789_hal.c
			)

idf_component_register(SRCS "${SOURCES}"
					INCLUDE_DIRS .
					REQUIRES spi_master_ext)
