CONTIKI_PROJECT = server
all: $(CONTIKI_PROJECT)
CONTIKI = ../../../../..

PLATFORMS_ONLY = cc26x0-cc13x0
BOARDS_ONLY = launchpad/cc2650 launchpad/cc2640r2

MAKE_MAC = MAKE_MAC_BLE_L2CAP
MAKE_NET = MAKE_NET_IPV6
MAKE_ROUTING = MAKE_ROUTING_RPL_LITE

MODULES += os/services/shell

include $(CONTIKI)/Makefile.identify-target
MODULES_REL += $(TARGET)

include $(CONTIKI)/Makefile.include
