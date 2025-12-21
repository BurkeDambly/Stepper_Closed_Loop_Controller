# STM32 Closed-Loop Stepper Motor Controller

## Overview
This project implements a **closed-loop stepper motor controller** using an STM32 microcontroller and a magnetic rotary encoder.  
Unlike traditional open-loop stepper control, this system continuously measures the motor’s actual shaft position and compares it to the expected motion, allowing it to detect stalls, external disturbances, or missed steps and react immediately by correcting direction or behavior.

The goal is to achieve **robust, low-level motion feedback** without relying on complex field-oriented control or heavy software stacks.

---

## What It Does
- Drives a stepper motor using STEP/DIR signaling  
- Reads absolute shaft position from a magnetic encoder  
- Compares expected motion vs. measured motion in real time  
- Detects deviations (stall, resistance, or blockage)  
- Automatically responds by reversing or correcting motion  
- Operates without UART logging to ensure deterministic timing  

---

## How It Works
1. **Commanded Motion**  
   The firmware generates STEP pulses at a fixed rate and tracks the expected encoder count change per step.

2. **Encoder Feedback**  
   The encoder provides high-resolution absolute angle data, converted into incremental movement counts.

3. **Error Evaluation**  
   A sliding time window checks whether encoder movement matches the expected direction and magnitude.

4. **Decision Logic**  
   If measured motion deviates beyond a tight threshold, the controller flags an error and immediately changes direction or state.

This approach avoids PID tuning and instead relies on **directional consistency and motion validation**, making it fast and predictable.

---

## Components Used
- STM32 microcontroller (G4/F0 class)  
- Stepper motor driver (STEP/DIR interface, e.g. TMC series)  
- Magnetic rotary encoder (AS5048A or equivalent)  
- Stepper motor (NEMA-class)  
- External power supply (motor voltage appropriate)

---

## Use Cases
- Closed-loop stepper experimentation  
- Force or stall detection  
- Robotics joints and actuators  
- Educational motor control systems  

