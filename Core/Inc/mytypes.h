/*
 * myTypes.h
 *
 *  Created on: Jan 7, 2024
 *      Author: denis
 */

#ifndef INC_MYTYPES_H_
#define INC_MYTYPES_H_
typedef enum {
	RMS,
	ampl,
	ARV
} lcdMode_t;
typedef struct{
  unsigned previous_state :1;
  unsigned current_state :1;
} buttonPinState_t;

#endif /* INC_MYTYPES_H_ */
