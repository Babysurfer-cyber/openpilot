#include <stdbool.h>
#include <string.h>

#include "fake_stm.h"
#include "can.h"

// Counterparts of the production UART and CAN-driver definitions, used by this
// branch's buffered Hyundai forwarding code. They are absent from fake_stm.
void putui(uint32_t i) { puth(i); }
bool safety_tx_buffered_for_fwd = false;

//int safety_tx_hook(CANPacket_t *to_send) { return 1; }

#include "faults.h"
#include "safety.h"
#include "drivers/can_common.h"

// libsafety stuff
#include "safety_helpers.h"
