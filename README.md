Background: The company behind Kohler Home Backup, Electriq Energy went out of business leaving countless Kohler Battery systems useless. For the DIY market, this means these high quality CATL modules are hitting the market for cheap. However, the original system was wired in series to achieve a 96v system. This does not work for home 24v or 48v systems, so a BMS system is needed to handle these.

Progress: So far, I've been listening to CAN traffic to try and decode what is happening. Posted here is the progress so far, including the Arduino code and diagram.

Kohler Parts Needed:
1. 4 24v Modules
2. BMS Module
3. 10kw wiring harness (includes all wiring for the above)

Other Parts Needed:
1. 30-Pin ESP32 (need 3.3v power)
2. Waveshare SN65HVD230 - CAN Module
3. Breadboard Wires to connect the ESP32 and CAN Module
4. Some other wires to wire into the harness

Diagram for ESP32 + CAN Module:

<img width="577" alt="image" src="https://github.com/user-attachments/assets/907820a6-b7c4-4990-8313-9f1ff39146a4" />
