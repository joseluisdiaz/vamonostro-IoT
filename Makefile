ifndef TARGET
TARGET=z1
endif

# Linker optimizations
SMALL = 1

# This flag includes the IPv6 libraries
CONTIKI_WITH_IPV6 = 1

# Enable to pull-in Z1SP specific test/source files
CONTIKI_PROJECT = vamonosotro.c

CONTIKI_SOURCEFILES += sht11.c reed-sensor.c sht25.c tlc59116.c light-ziglet.c \
                       relay-phidget.c sht11.c sht11-sensor.c bmpx8x.c 

MODULES += dev/sht11

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

all: $(CONTIKI_PROJECT)
CONTIKI = ../../..
CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include
