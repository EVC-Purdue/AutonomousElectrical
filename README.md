# EVC Autonomous - Electrical

Autonomous Project electrical stack code

## Pins

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