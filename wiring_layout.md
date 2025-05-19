# DC Motor Control Project - Wiring Layout for PlatformIO

## Components
1. Arduino Uno
2. L298N Motor Driver (x1)
3. Two DC motors (2-wire)
4. 1602 LCD display with I2C interface
5. Three LEDs (red, yellow, green)
6. Resistors for LEDs (220Ω)
7. Jumper wires
8. Power supply (7-12V for motors)

## Wiring Layout

### Arduino to L298N Motor Driver
- **Motor 1:**
  - Arduino pin 8 → L298N IN1
  - Arduino pin 9 → L298N IN2
  - Arduino pin 10 → L298N ENA (Enable A)

- **Motor 2:**
  - Arduino pin 6 → L298N IN3
  - Arduino pin 7 → L298N IN4
  - Arduino pin 11 → L298N ENB (Enable B)

### Arduino to LCD Display (I2C)
- Arduino SDA (A4) → LCD SDA
- Arduino SCL (A5) → LCD SCL
- Arduino 5V → LCD VCC
- Arduino GND → LCD GND

### Arduino to LEDs
- Arduino pin 13 → 220Ω resistor → Red LED → GND
- Arduino pin 12 → 220Ω resistor → Yellow LED → GND
- Arduino pin 2 → 220Ω resistor → Green LED → GND

### Power Supply
- Connect 7-12V power supply to L298N's power input
- Connect L298N's GND to Arduino's GND
- If needed, connect L298N's +5V to Arduino's 5V (only if not powering Arduino separately)

## L298N Motor Driver Configuration
- Make sure all jumpers on the L298N board are set correctly
- For each motor, connect the two wires to the output terminals:
  - Motor 1: Connect two wires to OUT1 and OUT2
  - Motor 2: Connect two wires to OUT3 and OUT4

## Notes
1. The I2C address of the LCD display might need adjustment in the code (default is 0x27)
2. Make sure the polarity of DC motor connections is correct - swapping the wires will reverse the direction
3. The L298N driver needs adequate cooling if running motors for extended periods
4. Make sure to double-check all connections before powering on
5. The DC motors might require a separate power supply depending on their current requirements
6. The code uses PWM (analogWrite) to control the speed of the motors through the ENA and ENB pins
