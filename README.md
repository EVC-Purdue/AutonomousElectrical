# EVC Autonomous - Electrical

Autonomous Project electrical stack code

## Autonomous Control/Distro Board

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

### CAN messages

- ID = `0x100` - **Control commands** (RX)
	- Byte 0-1: throttle (uint16_t, little endian)
	- Byte 2-3: steering (uint16_t, little endian)
	- Byte 4-7: reserved / future use
- ID = `0x101` - **Status update** (TX)
	- Byte 0: precharge + contactor + rc mode (bit flags)
		- Bit0 = precharge
		- Bit1 = contactor
		- Bit2 = RC mode
		- Bits3-7 = 0
	- Byte 1-2: throttle PWM (uint16_t, little endian)
	- Byte 3-4: steering PWM (uint16_t, little endian)
	- Byte 5-7: reserved / future use

## Autonomous Nucelo

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