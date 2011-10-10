#
# Make the sl (serial line driver)
#
EAPI_DIR = ../../eapi
include $(EAPI_DIR)/make/eapi_drv.mk

CFLAGS += -I $(EAPI_DIR)/include 
LDFLAGS += 

SL_DRV = $(PRIVDIR)/sl_drv.so
SL_DRV_OBJS = \
	$(OBJDIR)/$(EAPI_PFX)sl_drv.o \
	$(OBJDIR)/$(EAPI_PFX)sl_drv_impl.o

EAPI_LIB = $(EAPI_DIR)/lib/$(TYPE)/$(EAPI_PFX)eapi_drv.a

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
	$(LD_SHARED) -o $@ $(SL_DRV_OBJS) $(EAPI_LIB) $(LD_FLAGS)

$(OBJDIR)/$(EAPI_PFX)sl_drv_impl.o: $(EAPI_DIR)/include/eapi_drv.h
$(OBJDIR)/$(EAPI_PFX)sl_drv.o:  $(EAPI_DIR)/include/eapi_drv.h

$(OBJDIR)/$(EAPI_PFX)%.o:	%.c
	$(CC) -c -o $@ $(CFLAGS) -I$(ERL_C_INCLUDE_DIR) $<
