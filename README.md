# AntaresMicro 🚀

**AntaresMicro** is the most compact flight computer designed by SPL, based on the RP2040 microcontroller, intended for managing small amateur rockets.

---

## 📦 Main Features

- **Raspberry Pi RP2040** microcontroller  
- 1 **pyro** output for parachute deployment or stage ignition  
- 2 **PWM servo** channels  
- 1 **UART** port (e.g. for GPS or additional sensors)  
- 1 **SPI** bus (compatible with LoRa module)  
- **MicroSD** slot for data logging and settings  
- **MPU6050**: accelerometer + gyroscope  
- **BMP280**: barometric sensor for altitude detection  

---

## 🛠️ Hardware and Connections

| Component         | Connections                       | Notes                                |
|------------------|------------------------------------|--------------------------------------|
| RP2040           | —                                  | Main MCU                             |
| Power input      | Vin, GND                           | 7–35V (e.g. LiPo battery pack)       |
| Pyro channel     | GPIO 28                            | High current output                  |
| Servos           | GPIO 26, 27 (PWM)                  | Control for opening/mechanisms       |
| UART             | TX/RX                              | Connect GPS or other serial modules  |
| SPI              | SCK, MOSI, MISO, CS, RST, IRQ      | For LoRa module or similar           |
| MicroSD          | Dedicated SPI                      | Flight and log recording             |
| MPU6050          | I2C (SDA, SCL)                     | Acceleration/rotation readings       |
| BMP280           | I2C (SDA, SCL)                     | Pressure/altitude reading            |

---

## 💾 Firmware and Compilation

1. Install **PlatformIO** (as VSCode plugin or CLI)  
2. Clone the repository:  
   `git clone https://github.com/Nicrom098195/AntaresMicro.git`  
3. Enter the folder and upload the firmware:  
   `pio run --target upload`  
4. The firmware is located in `src/main.cpp` and handles:  
   - sensor initialization  
   - data reading  
   - SD logging  
   - actuator management (servos, pyro)  

---

## 🧩 Future Customizations

- Add magnetometer or GPS  
- Real-time communication via LoRa  
- Web interface for live data  
- Test mode and enhanced safety features  

---

## 📁 Project Structure

AntaresMicro/  
├── src/ → Main code  
├── lib/ → Additional libraries  
├── include/ → Header files  
├── test/ → Automated tests  
├── platformio.ini → PlatformIO configuration  
└── README.md → This file  

---

## 📄 License

Distributed under the MIT License. See the `LICENSE` file.

---

## 📬 Contact

For questions, feedback, or bug reports:  
**nicrom098195 [at] example.com**

---

## ⚠️ Disclaimer

This project is provided **solely for educational and experimental purposes**. The use of this code or hardware in real-world applications involves significant risk. The author **assumes no liability** for any damage to persons or property resulting from the use—whether proper or improper—of this software or hardware.  
It is the responsibility of the user to ensure compliance with **all local and international regulations** concerning safety, aviation, radio communications, and the transport of hazardous materials.

**Do not use this project for critical applications without proper validation and authorization.**
