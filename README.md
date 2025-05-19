# DC Motor Control with PlatformIO

This project controls two DC motors with an L298N driver, shows status information on a 1602 LCD display with I2C interface, and uses three LEDs for visual status indication.

## Project Structure

The project is organized for PlatformIO IDE:

```
SmartCity/
├── src/                    # Source code files
│   └── main.cpp            # Main program code
├── include/                # Header files (not used in this project)
├── lib/                    # Project-specific libraries
│   └── MotorControl/       # Custom motor control library
│       ├── MotorControl.h
│       └── MotorControl.cpp
├── platformio.ini          # PlatformIO configuration
├── wiring_layout.md        # Wiring instructions
└── SmartCity.code-workspace # VSCode workspace file
```

## Required Libraries

This project requires the LiquidCrystal_I2C library, which is automatically installed by PlatformIO when building the project (configured in platformio.ini).

## Building and Uploading

1. Open the project in PlatformIO IDE
2. Connect your Arduino Uno board to your computer
3. Click the "Build" button (or use Cmd+Alt+B on Mac) to build the project
4. Click the "Upload" button (or use Cmd+Alt+U on Mac) to upload to the Arduino

## Hardware Configuration

Refer to the `wiring_layout.md` file for detailed wiring instructions for connecting the L298N motor driver, DC motors, LCD display, and LEDs.

## Functionality

The program performs the following actions:
1. Initializes LCD and shows startup message
2. Runs both DC motors in the same direction for 20 seconds
3. Displays countdown timer on LCD
4. Uses LEDs to indicate status (red = running, yellow = halfway, green = complete)
