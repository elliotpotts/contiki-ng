#include "contiki.h"

PROCESS(barebones_process, "Barebones");
AUTOSTART_PROCESSES(&barebones_process);

PROCESS_THREAD(barebones_process, ev, data) {
  PROCESS_BEGIN();

  PROCESS_END();
}
