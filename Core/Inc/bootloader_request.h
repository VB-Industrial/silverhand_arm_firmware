#ifndef INC_BOOTLOADER_REQUEST_H_
#define INC_BOOTLOADER_REQUEST_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bootloader_request_schedule(void);
bool bootloader_request_pending(void);
void bootloader_request_process(void);

#ifdef __cplusplus
}
#endif

#endif
