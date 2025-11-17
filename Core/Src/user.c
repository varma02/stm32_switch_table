/*
 * user.c
 *
 *  Created on: Nov 3, 2025
 *  Author: varma01, Szenergy
 */

#include "user.h"


// *** Variables ***

struct {
	VCU_STATE_A A;
	VCU_STATE_B B;
} vcu_state;

struct {
	STW_STATE_A A;
	STW_STATE_B B;
	STW_STATE_C C;
	STW_STATE_D D;
} stw_state;

volatile struct USER_FLAGS user_flags;

struct VEHICLE_STATE vehicle_state;

struct {
	enum DRIVE_STATE prev;
	enum DRIVE_STATE current;
} drive_state;

ADC_HandleTypeDef *pot_adc;
FDCAN_HandleTypeDef *fdcan;
TIM_HandleTypeDef *wiper_pwm;

struct {
	uint16_t current;
	uint16_t prev;
	uint16_t ema;
} pot_value;

struct {
	uint16_t current;
	uint16_t prev;
} rate_limiter;

struct WIPER_STATE wiper_state;

// *** Processing Functions ***

/**
 * Performs a single raw ADC read of the potentiometer
 * @retval the current value
 */
uint16_t ADC_Read() {
	uint16_t value;
	HAL_ADC_Start(pot_adc);
	if (HAL_ADC_PollForConversion(pot_adc, 5) == HAL_OK) {
		value = HAL_ADC_GetValue(pot_adc);
	}
	HAL_ADC_Stop(pot_adc);
	return value;
}

/**
 * Reads, filters and debounces the current value of the potentiometer
 */
void Pot_Read_Filtered() {
	pot_value.prev = pot_value.current;
	pot_value.current = ADC_Read();

	if (pot_value.current > POT_ZERO) {
        pot_value.ema = (POT_EMA * pot_value.current) + ((1 - POT_EMA) * pot_value.ema);
        pot_value.current = pot_value.ema;
	}

	if ((pot_value.current - pot_value.prev) <= POT_STEP)
		pot_value.current = pot_value.prev;

	if (pot_value.current <= POT_ZERO + POT_STEP)
		pot_value.current = POT_VALUES[0];
	else if (pot_value.current <= POT_ZERO + POT_STEP * 19)
		pot_value.current = POT_VALUES[(pot_value.current - POT_ZERO) / POT_STEP];
	else
		pot_value.current = POT_VALUES[19];
}

/**
 * Throttle limiter for smooth acceleration
 * @param value - the throttle value to rate limit
 * @retval the rate limited value
 */
uint16_t Rate_Limit(uint16_t value) {
	rate_limiter.prev = rate_limiter.current;
	rate_limiter.current = value;
    if(2000 + rate_limiter.current - rate_limiter.prev > 2000 + RATE_LIMIT_UP)
        rate_limiter.current = rate_limiter.prev + RATE_LIMIT_UP;
    else if(2000 + rate_limiter.current - rate_limiter.prev < 2000 - RATE_LIMIT_DOWN)
        rate_limiter.current = rate_limiter.prev - RATE_LIMIT_DOWN;
    return rate_limiter.current;
}

/**
 * Updates the drive state of the vehicle
 */
void Drive_State_Update() {
	drive_state.prev = drive_state.current;
	if (vcu_state.A.MC_OW == RESET) {
		if (pot_value.current > 0) {
			if (stw_state.A.DRIVE == SET) {
				drive_state.current = D_DRIVE_PEDAL;
			} else if (stw_state.A.REVERSE == SET) {
				drive_state.current = D_REVERSE_PEDAL;
			}
		} else if (stw_state.A.ACC == SET && pot_value.current == 0) {
			if (stw_state.A.DRIVE == SET) {
				drive_state.current = D_AUTO_ACC;
			} else if (stw_state.A.REVERSE == SET) {
				drive_state.current = D_AUTO_DEC;
			}
		} else {
			drive_state.current = D_NEUTRAL;
		}
	} else {
		drive_state.current = D_NEUTRAL;
	}
}

/**
 * Calculates the regulated throttle setting to send to the motor
 */
uint16_t Calculate_MC_Ref() {
	uint16_t reference = pot_value.current;

	if (drive_state.current != D_NEUTRAL) {
		if (drive_state.current == D_AUTO_ACC || drive_state.current == D_AUTO_DEC) {
			uint16_t speed = vehicle_state.rpm * SPEED_MULT_FACTOR;
			switch (stw_state.D.bits) {
			case 0:
			case 1:
			case 8:
				reference = 1023;
				break;
			case 2:
				if (vehicle_state.rpm >= 116)
					reference = 1023 * LUT_Z22 [speed];
				else
					reference = 1023;
				break;
			case 4:
				if (vehicle_state.rpm >= 221)
					reference = 1023 * LUT_Z24 [speed];
				else
					reference = 1023;
				break;
			case 16:
				if (speed >= 5)
					reference = 409;
				else
					reference = 1023;
				break;
			case 32:
				reference = 818;
				break;
			case 64:
				reference = 920;
				break;
			default:
				reference = 0;
				break;
			}
		}
	} else if (drive_state.prev != D_NEUTRAL && drive_state.current == D_NEUTRAL) {
		rate_limiter.current = 0;
		rate_limiter.prev = 0;
		reference = 0;
	} else {
		reference = 0;
	}

	return Rate_Limit(reference);
}

/**
 * Reads the state of the switches and adjusts the outputs accordingly
 */
void Port_Update() {
	vcu_state.A.AUTO           = HAL_GPIO_ReadPin(SW_AUTO_GPIO_Port, SW_AUTO_Pin);
	vcu_state.A.HAZARD         = HAL_GPIO_ReadPin(SW_HAZARD_GPIO_Port, SW_HAZARD_Pin);
	vcu_state.A.LIGHTS_ENABLE  = HAL_GPIO_ReadPin(SW_LIGHTS_GPIO_Port, SW_LIGHTS_Pin);
	vcu_state.A.MC_OW          = HAL_GPIO_ReadPin(SW_MC_OW_GPIO_Port, SW_MC_OW_Pin);
	vcu_state.A.WIPER          = HAL_GPIO_ReadPin(SW_WIPER_GPIO_Port, SW_WIPER_Pin);
	vcu_state.A.HEADLIGHT      = HAL_GPIO_ReadPin(SW_HEADLIGHT_GPIO_Port, SW_HEADLIGHT_Pin);
	vcu_state.A.BRAKE          = HAL_GPIO_ReadPin(IN_BRAKE_GPIO_Port, IN_BRAKE_Pin);
	vcu_state.B.RELAY_NO       = HAL_GPIO_ReadPin(IN_SHELL_RELAY_GPIO_Port, IN_SHELL_RELAY_Pin);

	if (vcu_state.A.BRAKE == SET) {
		HAL_GPIO_WritePin(OUT_BRAKE_GPIO_Port, OUT_BRAKE_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(OUT_BRAKE_GPIO_Port, OUT_BRAKE_Pin, GPIO_PIN_SET);
	}

	if (vcu_state.B.RELAY_NO == SET) {
		vcu_state.A.MC_OW = RESET;
	}
}

/**
 * Callback function that processes the received CAN message
 */
void CAN_Receive() {
	FDCAN_RxHeaderTypeDef header;
	uint8_t data[8];

	if (HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
		Error_Handler();
	}

	switch (header.Identifier) {
		case 0x185:
			user_flags.can_synced = SET;
			break;
		case 0x190:
			stw_state.A.bits = data[0];
			stw_state.B.bits = data[1];
			stw_state.C.bits = data[2];
			stw_state.D.bits = data[3];
			if (vcu_state.B.RELAY_NO == SET) {
				stw_state.A.ACC = RESET;
				stw_state.A.DRIVE = RESET;
				stw_state.A.REVERSE = RESET;
			}
			break;
		case 0x123:
			vehicle_state.rpm = ((uint16_t)data[0] << 8) | data[1];
			break;
		case 0x10:
//			if (data[0] & 1) {
//				HAL_GPIO_WritePin(OUT_SHELL_RELAY_GPIO_Port, OUT_SHELL_RELAY_Pin, GPIO_PIN_SET);
//			} else {
//				HAL_GPIO_WritePin(OUT_SHELL_RELAY_GPIO_Port, OUT_SHELL_RELAY_Pin, GPIO_PIN_RESET);
//			}
			break;
		default: break;
	}
}

/**
 * Transmits the switch states and throttle position to CAN
 */
void CAN_Send_Vcu() {
	FDCAN_TxHeaderTypeDef header;
	uint8_t data[6];

	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.Identifier = 0x129;
	header.IdType = FDCAN_STANDARD_ID;
	header.TxFrameType = FDCAN_DATA_FRAME;
	header.DataLength = FDCAN_DLC_BYTES_6;

	data[0] = vcu_state.A.bits;
	data[1] = vcu_state.B.bits;

	int32_t throttle_buffer = ((double)pot_value.current / 1023) * 100000;
	data[2] = throttle_buffer >> 24;
	data[3] = throttle_buffer >> 16;
	data[4] = throttle_buffer >> 8;
	data[5] = throttle_buffer;

	if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &header, data) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * Sends a motor control CAN command
 * @param reference - the throttle setting to send
 */
void CAN_Send_Mc(uint16_t reference) {
	FDCAN_TxHeaderTypeDef header;
	uint8_t data[4];

	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.Identifier = 0xA51;
	header.IdType = FDCAN_EXTENDED_ID;
	header.TxFrameType = FDCAN_DATA_FRAME;
	header.DataLength = FDCAN_DLC_BYTES_4;

	int32_t throttle_buffer = ((double)reference / 1023) * 100000;
	if (stw_state.A.REVERSE)
		throttle_buffer = throttle_buffer * -1;
	data[0] = throttle_buffer >> 24;
	data[1] = throttle_buffer >> 16;
	data[2] = throttle_buffer >> 8;
	data[3] = throttle_buffer;

	if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &header, data) != HAL_OK) {
		Error_Handler();
	}
}

void Wiper_Task() {
	switch (wiper_state.step) {
	case 0: // Standby state, waiting for wiper switch signal
		if (vcu_state.A.WIPER == SET)
			wiper_state.step = 1;
		break;
	case 1: // Setup phase, start PWM and converter
		wiper_pwm->Instance->CCR1 = 0;
		HAL_TIM_PWM_Start(wiper_pwm, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(OUT_WIPER_CONVERTER_GPIO_Port, OUT_WIPER_CONVERTER_Pin, GPIO_PIN_SET);
		wiper_state.running = SET;
		wiper_state.step = 2;
		break;
	case 2: // Wipe Right
		wiper_pwm->Instance->CCR1 = WIPER_RIGHT;
		if (vcu_state.A.WIPER == RESET) {
			wiper_state.step = 4;
		} else {
			wiper_state.step = 3;
		}
		break;
	case 3: // Wipe Left
		wiper_pwm->Instance->CCR1 = WIPER_LEFT;
		if (vcu_state.A.WIPER == RESET) {
			wiper_state.step = 4;
		} else {
			wiper_state.step = 2;
		}
		break;
	case 4: // Go to the center
		wiper_pwm->Instance->CCR1 = WIPER_CENTER;
		wiper_state.step = 5;
	case 5: // Turn off and go to standby
		HAL_TIM_PWM_Stop(wiper_pwm, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(OUT_WIPER_CONVERTER_GPIO_Port, OUT_WIPER_CONVERTER_Pin, GPIO_PIN_RESET);
		wiper_state.running = RESET;
		wiper_state.step = 0;
		break;
	default:
		break;
	}
}


// *** External functions ***

/**
 * Initializes the user defined variables
 */
void User_Init(ADC_HandleTypeDef *adc_ptr, FDCAN_HandleTypeDef *fdcan_ptr, TIM_HandleTypeDef *wiper_pwm_ptr) {
	pot_adc = adc_ptr;
	fdcan = fdcan_ptr;
	wiper_pwm = wiper_pwm_ptr;

	vcu_state.A.bits = 0b00000000;
	vcu_state.B.bits = 0b00000000;
	stw_state.A.bits = 0b00000000;
	stw_state.B.bits = 0b00000000;
	stw_state.C.bits = 0b00000000;
	stw_state.D.bits = 0b00000000;
	user_flags.can_receive = RESET;
	user_flags.can_synced = RESET;
	user_flags.interval_CAN = RESET;
	user_flags.interval_wiper = RESET;
	vehicle_state.rpm = 0;
	drive_state.current = D_NEUTRAL;
	drive_state.prev = D_NEUTRAL;
	pot_value.current = POT_ZERO;
	pot_value.prev = POT_ZERO;
	pot_value.ema = 0;
	rate_limiter.current = 0;
	rate_limiter.prev = 0;
	wiper_state.running = RESET;
	wiper_state.step = 0;
}

/**
 * Main loop for user tasks, runs every on clock cycle
 */
void User_Loop() {
	Port_Update();

	if (user_flags.interval_wiper == SET) {
		Wiper_Task();
		user_flags.interval_wiper = RESET;
	}

	if (user_flags.can_receive == SET) {
		CAN_Receive();
		user_flags.can_receive = RESET;
	}

	if (user_flags.interval_CAN == SET) {
		Pot_Read_Filtered();
		CAN_Send_Vcu();
		if (vcu_state.A.MC_OW == RESET && vcu_state.A.AUTO == RESET) {
			Drive_State_Update();
			CAN_Send_Mc(Calculate_MC_Ref());
		}
		user_flags.interval_CAN = RESET;
	}
}
