#
# Make the sl (serial line driver)
#
EAPI_DIR = ../../eapi
include $(EAPI_DIR)/make/eapi_drv.mk

SL_DRV = ../priv/$(TYPE)/sl_drv.so
SL_DRV_OBJS = $(OBJDIR)/$(EAPI_PFX)sl_drv.o

all:
	$(MAKE) driver TYPE=debug
	$(MAKE) driver TYPE=release

debug:
	$(MAKE) driver TYPE=debug

release:
	$(MAKE) driver TYPE=release

driver: $(SL_DRV)

clean:
        $(MAKE) clean_internal TYPE=release
        $(MAKE) clean_internal TYPE=debug

clean_debug:
        $(MAKE) clean_internal TYPE=debug

clean_internal:
	rm -f $(SL_DRV)
	rm -f $(SL_DRV_OBJS)

$(SL_DRV): $(SL_DRV_OBJS)
	$(LD_SHARED) -o $@ $(SL_DRV_OBJS) $(DLL_LIBS) $(LD_FLAGS)

$(OBJDIR)/$(EAPI_PFX)sl_drv.o:  cbufv2.h

$(OBJDIR)/$(EAPI_PFX)%.o:	%.c
	$(CC) -c -o $@ $(CFLAGS) -I$(ERL_C_INCLUDE_DIR) $<
