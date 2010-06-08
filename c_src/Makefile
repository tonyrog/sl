#
# Make the sl (serial line driver)
#
OSNAME := $(shell uname -s)
MACHINE := $(shell uname -m)

ERLDIR	:= $(shell erl -noshell -eval "io:format([126,115,126,110],[code:root_dir()])" -s erlang halt)
ERL_C_INCLUDE_DIR := $(ERLDIR)/usr/include

ifeq ($(OSNAME), Linux)
LD_SHARED=$(CC) -shared
DLL_LIBS=
endif

ifeq ($(OSNAME), Darwin)
LD_SHARED := $(CC) -m32 -bundle -flat_namespace -undefined suppress
DLL_LIBS=
CFLAGS += -m32
endif

OBJDIR = ../obj/$(TYPE)

DRV_OBJS = $(OBJDIR)/sl_drv.o

DRV_SO = ../priv/$(TYPE)/sl_drv.so

WARN = -Wall -Wextra -Wswitch-default -Wswitch-enum 

ifeq ($(TYPE), debug)
override CFLAGS += -Ddebug -DDEBUG $(WARN) -g -I$(ERL_C_INCLUDE_DIR) -fPIC
else
override CFLAGS += $(WARN) -g -I$(ERL_C_INCLUDE_DIR) -fPIC
endif

all:
	$(MAKE) TYPE=debug driver
	$(MAKE) TYPE=release driver

debug:
	$(MAKE) TYPE=debug driver

release:
	$(MAKE) TYPE=release driver

driver: $(DRV_SO)

clean:
	-rm -f $(DRV_SO)

$(DRV_SO): $(DRV_OBJS)
	$(LD_SHARED) -o $@ $(DRV_OBJS) $(DLL_LIBS) $(LD_FLAGS) 

$(OBJDIR)/sl_drv.o:  cbufv2.h

$(OBJDIR)/%.o:	%.c
	$(CC) -c -o $@ $(CFLAGS) $<
