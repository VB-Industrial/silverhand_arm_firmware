/*
 * communications.h
 *
 *  Created on: Feb 28, 2024
 *      Author: VR
 */

#ifndef INC_COMMUNICATIONS_H_
#define INC_COMMUNICATIONS_H_

#include <stdbool.h>
#include "fdcan.h"

void cyphal_loop();
void heartbeat();
void send_JS(void); //float*, float*, float*
void setup_cyphal(FDCAN_HandleTypeDef*);
void cyphal_can_starter(FDCAN_HandleTypeDef*);

#endif /* INC_COMMUNICATIONS_H_ */
