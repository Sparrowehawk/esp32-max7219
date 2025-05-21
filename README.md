# ESP32 SPI demo for a MAX7219 8x8 dot display 

A basic demo on how to use a 8x8 dot matrix with an ESP32 via SPI. Example includes for one display and 3

## Parts Used
- ESP32
- 3 x 8x8 MAX7219 dot displays

## Project Directory
```
wireless-display-driver/
├── main/ 
│ ├── main-singular.c
│ ├── main-multi.c
│ └── CMakeLists.txt
│
├── README.md
├── CMakeLists.txt
└── LICENSE
```
  
## Wiring

 - Connect the relavent power pins, only the first display has to be connected to the ESP32 SPI pins, the rest are daisy chained. Note that with >2 displays, power may become an issue due to the displays high power draw
   - `GPIO 23` — SPI Transmitter (MOSI)
   - `GPIO 18` — SPI Clock (SCLK)
   - `GPIO 5` — Chip select (CS)
---

## License
MIT License

---

## Author
[Sparrowehawk](https://github.com/Sparrowehawk)
