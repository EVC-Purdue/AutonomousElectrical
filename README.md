# EVC Autonomous - Electrical

Autonomous Project electrical stack code

## Autonomous Control/Distro Board

### CAN messages

- ID = `0x100` - **Control commands** (RX)
	- Byte 0-1: throttle (uint16_t, little endian), 0-1000, where 1000 = full throttle
	- Byte 2-3: steering (uint16_t, little endian), 0-1000, where 500 = straight, 0 = full left, 1000 = full right
	- Byte 4-7: reserved / future use
- ID = `0x101` - **Status update** (TX)
	- Byte 0: state machine mode + rc mode
		- Bits0-3 = state machine mode (see logic.h::logic_mode_t)
		- Bit4 = RC mode (0 = rc mode, 1 = autonomous mode)
		- Bits5-7 = reserved
	- Byte 1-2: throttle PWM (uint16_t, little endian), the actual PWM value being sent to the ESC for throttle (1000-2000)
	- Byte 3-4: steering PWM (uint16_t, little endian), the actual PWM value being sent to the servo for steering (1000-2000)
	- Byte 5-7: reserved / future use

### RC configuration

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


### Pin Definitions 

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

### State Machine

<img width="779" height="2021" alt="Autonomous Distro-Control Board
state machine"
src="https://github.com/user-attachments/assets/a88d2ef2-eab9-4c80-acc3-6d577d6a04e4"
/>

## Autonomous Nucleo

### Pin Definitions 

- Rubik Pi 3 <-> nucleo-f446re: `SPI2`
	- SCK: `PB10`
	- NSS: `PB12`
	- MOSI: `PC1`
	- MISO: `PC2`
- Motor controller control: `TIM2`
	- Output: `PA15`
	- 1000-2000 microseconds pulse width (stop - full forward)
- Steering servo control: `TIM3`
	- Output: `PA6`
	- 1000-2000 microseconds pulse width (left - right)
- RC contactor/e-stop PWM input: `TIM5`
	- Input: `PA0`
	- \>=1800 microseconds pulse width to close contactor
- GPIOs:
	- LD2: `PA5`
		- Output
		- On nucleo green LED
	- B1: `PC13`
		- Input: external interrupt mode on falling edge
		- On nucleuo button
	- CONTACTOR: `PB8`
		- Output: pull down
		- Set high to close contactor