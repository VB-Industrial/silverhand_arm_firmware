/*
 * communications.h
 *
 *  Created on: Feb 28, 2024
 *      Author: VR
 */

#ifndef INC_COMMUNICATIONS_H_
#define INC_COMMUNICATIONS_H_

#include "tmc5160.h"

void cyphal_loop();
void heartbeat();
void send_JS(void); //float*, float*, float*
void setup_cyphal(FDCAN_HandleTypeDef*);
void cyphal_can_starter(FDCAN_HandleTypeDef*);
void fusion_startup_sync(void);

#endif /* INC_COMMUNICATIONS_H_ */
