# Robogol – Goltron v1.0 ⚽🤖

This repository contains the hardware design and embedded firmware development for **Robogol (Goltron v1.0)**, a 4-wheel drive mobile soccer robot powered by an **ESP32-WROOM-32** microcontroller[cite: 1].

The project focuses on integrating custom PCB design, power regulation, dual H-bridge motor control, and wireless Bluetooth gamepad connectivity using the **Bluepad32** library.

Robogol combines embedded electronics, mobile robotics, and PCB manufacturing for robotics competitions and educational engineering purposes[cite: 1].

---

## ⚙️ Main Features
- Custom single/double-sided PCB design (**Goltron v1.0**) developed in EasyEDA[cite: 1]
- Independent 4-motor traction control using dual **TB6612FNG** H-bridge drivers[cite: 1]
- High-efficiency step-down voltage regulation (**LM2596**) for logic isolation[cite: 1]
- Power supply decoupling and transient noise filtering for high-current motor peaks[cite: 1]
- Wireless low-latency control via standard Bluetooth gamepads (PlayStation, Xbox, Nintendo)
- Modular hardware architecture with dedicated power, control, and driver stages[cite: 1]

---

## 🧰 Technologies & Hardware Used
- **Microcontroller:** ESP32-WROOM-32 (30-pin DevKit)[cite: 1]
- **Firmware:** C++ / Arduino Framework & **Bluepad32** Library
- **Motor Drivers:** 2x TB6612FNG Dual H-Bridge Modules[cite: 1]
- **Voltage Regulation:** LM2596 DC-DC Buck Converter (5V regulated rail)[cite: 1]
- **Power Supply:** Rechargeable 18650 Lithium-Ion battery cells (series configuration)[cite: 1]
- **EDA & PCB Design:** EasyEDA & Adobe Illustrator (1:1 fabrication templates)[cite: 1, 3, 4]

---

## 📁 Project Structure
- `firmware/` → ESP32 C++ source code and Bluepad32 gamepad mapping
- `hardware/easyeda/` → Schematic and PCB source files (Goltron v1.0)[cite: 1]
- `hardware/gerbers/` → Gerber files ready for PCB manufacturing
- `hardware/docs/` → Printable 1:1 PCB layouts, schematic PDF, and BOM[cite: 1, 2]
- `media/` → Robot assembly photos, 3D PCB renders, and match test videos

---

## 💡 Applications
- Competitive robotic soccer tournaments (Robogol)[cite: 1]
- Embedded systems and microcontroller motor-control prototyping
- Custom PCB design and power electronics validation
- Educational mobile robotics and wireless telemetry testing

---

## 👤 Author
**Juan Esteban Díaz Delgado**  
Electronic Engineering Student – Universidad Surcolombiana  
Technologist in Software Analysis and Development – SENA  

**Role:**  
Schematic & PCB design (VOLTIA – Goltron v1.0), embedded firmware development, and hardware integration[cite: 1].

---

## 📄 License
This project is shared for academic, competitive, and educational purposes.  
Hardware designs and firmware are provided as open reference; commercial reproduction or redistribution without proper authorization is not permitted.

© 2026 – VOLTIA / Juan Esteban Díaz Delgado. All rights reserved[cite: 1].
