# Makefile for building gpio_lx-I2C Sample code for libmpsse on Linux.

DEPENDENCIES := -lpthread
UNAME := $(shell uname)
# Assume target is Mac OS if build host is Mac OS; any other host targets Linux
ifeq ($(UNAME), Darwin)
	DEPENDENCIES := -lobjc -framework IOKit -framework CoreFoundation
else
	DYNAMIC_LIBS += -pthread -lrt
endif

CFLAGS = -Wall -Wextra 

DYNAMIC_LINK_OPTIONS := -Wl
APP = gpio_lx
STATIC_APP = $(APP)-static
DYNAMIC_APP = $(APP)-dynamic

all: $(STATIC_APP) $(DYNAMIC_APP)

$(STATIC_APP): gpio_lx.c
	$(CC) gpio_lx.c -o $(STATIC_APP) -lmpsse $(CFLAGS) $(DEPENDENCIES)

$(DYNAMIC_APP): gpio_lx.c
	$(CC) gpio_lx.c -o $(DYNAMIC_APP) $(CFLAGS) -lmpsse 

clean:
	-rm -f *.o ; rm $(STATIC_APP); rm $(DYNAMIC_APP)
