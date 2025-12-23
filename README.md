# esp32Cutter

Project Overview: Automated Strap Cutter with Current Monitoring

This project is an automated strap-cutting system controlled by an ESP32 MCU. It monitors the current drawn by a DC motor used for tensioning a strap, and when the current exceeds a threshold (e.g., 60mA), it triggers a solenoid-actuated cutter to cut the strap. The system also includes visual indicators (LEDs) and a power-saving auto-shutdown feature.
Key Components

    ESP32 MCU – Main controller for logic, monitoring, and control.

    DC Motor – Used to apply tension to the strap.

    IRLZ44N MOSFET – Drives the DC motor and the cutter solenoid.

    INA219 Current Sensor – Measures the current drawn by the DC motor.

    Solenoid Cutter (H1) – Cuts the strap when triggered.

    LED Indicators:

        Green LED (P13) – Blinks when system is active/working.

        Red LED (P15) – Blinks when system is in standby.

    Command Button – Used for manual control or system reset.

    Power Supply – 3.3V for ESP32 and peripherals.

System Operation
1. Current Monitoring & Cutting Logic

    The INA219 sensor continuously measures the current through the DC motor.

    When the strap tension increases, the motor draws more current.

    If the current exceeds 60mA, the ESP32 triggers the solenoid cutter via the IRLZ44N MOSFET.

    This ensures the strap is cut only when adequate tension is achieved.

2. LED Status Indication

    Green LED (P13) blinking → System is active, motor is running.

    Red LED (P15) blinking → System is in standby, waiting for tension.

    Green LED blinks 10 times → Warning before auto-shutdown.

3. Auto-Shutdown Feature

    If no activity is detected for 60 seconds, the system enters power-saving mode.

    Before shutdown, the green LED blinks 10 times as a warning.

    The system can be reactivated via the command button.

Firmware Development

    Written in C/C++ using PlatformIO.

    Libraries used:

        Adafruit_INA219 for current sensing

        ESP32 GPIO control for motor, cutter, and LEDs

        Timer interrupts for auto-shutdown and LED blinking

Typical Use Case

    Strap is tensioned by the DC motor.

    Current is monitored in real-time.

    When tension is sufficient (current > 60mA), cutter is activated.

    System indicates status via LEDs.

    If idle for 60 seconds, it shuts down automatically.

Applications

    Packing machines

    Strapping tools

    Automated tension-controlled cutting systems

    Industrial automation