# EVC Autonomous - Electrical

Autonomous Project electrical stack code


## State Machine

![State machine diagram](./Autonomous%20Electrical%20State%20Machine.drawio.png)

## CAN messages

- ID = `0x100` - **Control commands** (RX)
	- Byte 0-1: throttle ERPM (uint16_t, little endian), 0-MAX_ERPM (0 = full stop, MAX_ERPM = max speed)
	- Byte 2-3: steering representation (uint16_t, little endian), 0-1000, where 500 = straight, 0 = full left, 1000 = full right
	- Byte 4-7: reserved / future use
- ID = `0x101` - **Status update** (TX)
	- Byte 0: state machine mode + rc mode
		- Bits0-3 = state machine mode (see logic.h::logic_mode_t)
		- Bit4 = RC mode (0 = rc mode, 1 = autonomous mode)
		- Bits5-7 = reserved
	- Byte 1-2: throttle PWM (uint16_t, little endian), the actual PWM value being sent to the ESC for throttle (1000-2000)
	- Byte 3-4: steering PWM (uint16_t, little endian), the actual PWM value being sent to the servo for steering (1000-2000)
	- Byte 5-7: reserved / future use
- ID = `0x102` - **E_Comms heartbeat** (RX)
	- Byte 0: E_Comms heartbeat counter (uint8_t)
	- Byte 1-7: reserved / future use
- ID = `CAN_VESC_MSG_NUM_TO_EXT_ID(3 = CAN_VESC_SET_RPM_MSG_NUM)` (ext id) - **VESC set (E)RPM** (TX)
 	- Byte 0-3: desired VESC ERPM (BE)
 	- Byte 4-7: reserved
- ID = `CAN_VESC_MSG_NUM_TO_EXT_ID(9 = CAN_VESC_STATUS_1_MSG_NUM)` (ext id) - **VESC status 1** (RX)
	- Byte 0-3: VESC ERPM (BE)
	- Byte 4-5: VESC current (in 0.1A, so 100 = 10A) (BE)
	- Byte 6-7: VESC duty cycle (in 0.001, so 1000 = 100%) (BE)

![CAN message flow diagram](./EVC-A26%20CAN%20flow%20path.drawio.png)

## RC configuration

- Throttle
	- Channel: 2 (index 1)
	- At idle (center position): 1500 microseconds pulse width
	- At full throttle forward: 2000 microseconds pulse width
	- Pushing the throttle stick back still sends 1500
- Steering
	- Channel: 4 (index 3)
	- At center position: 1500 microseconds pulse width
	- At full right: 2000 microseconds pulse width
	- At full left: 1000 microseconds pulse width
- Mode switch
	- Channel: 5 (index 4)
	- In RC mode: 1000 microseconds pulse width (not activated posistion)
	- In autonomous mode: 2000 microseconds pulse width (activated position)
- E-Stop switch
	- Channel: 6 (index 5)
	- Not pressed: 1000 microseconds pulse width (not activated position)
	- Pressed: 2000 microseconds pulse width (activated position)

## Pin Definitions 

- **CAN Bus (CAN2):**
    - RX: `PB5`
    - TX: `PB6`
    - **CAN_S (Silent):** `PB7` 
		- *LOW to enable transmitter*
		- *HIGH to disable transmitter*

- **VESC COMMS (UART5):**
	- RX: `PD2` 
    - TX: `PC12`
	
- **RC Reciever (USART3)**
	- RX: `PB11` 
    - TX: `PB10`

- **USB (Bootloader):**
    - DM: `PA11`
    - DP: `PA12`
	
- **STEERING OUT (PWM):**
    - TIM1 (CH1/CH1N): `PE9`

- **THROTTLE OUT (PWM):**
    - TIM11 (CH1): `PB9`

- **GPIOs:**
    - CAN_S: `PB7` 
    - Main Coil EN: `PE5`
    - Precharge EN: `PE6`
	- LED OUT: `PE8`

- **System / Debug:**
    - SWDIO: `PA13`
    - SWCLK: `PA14`
    - OSC_IN/OUT: `PH0` / `PH1` (8MHz HSE)