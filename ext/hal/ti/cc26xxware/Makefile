ifdef CONFIG_HAS_CC26XXWARE
ZEPHYRINCLUDE +=-I$(srctree)/ext/hal/ti/cc26xxware
ZEPHYRINCLUDE +=-I$(srctree)/ext/hal/ti/cc26xxware/inc
ZEPHYRINCLUDE +=-I$(srctree)/ext/hal/ti/cc26xxware/driverlib
endif # CONFIG_HAS_CC26XXWARE

ifdef CONFIG_CC26XXWARE_ROM_DRIVERLIB
KBUILD_CFLAGS += -DTARGET_IS_CC2650
endif  # CONFIG_CC26XXWARE_ROM_DRIVERLIB
