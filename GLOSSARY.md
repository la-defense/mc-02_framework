# Firmware glossary

| Term | Meaning in this project |
|---|---|
| MC-02 | The STM32H723 lower computer and robot-control firmware. |
| Upper computer | sp_vision_25: camera acquisition, target detection/tracking, planning, and SP serial communication. |
| SP frame | The existing byte protocol between upper and lower computers. The lower computer receives 29-byte commands and transmits 43-byte state frames, both protected by CRC16/X25. |
| Vision timeout | The firmware safety rule that clears stale upper-computer commands after the configured period without a valid frame. |
| Bare-board HIL | Bench communication and parsing validation with the MC-02 board and debugger connected and actuators disconnected. |
| Neutral command | A 29-byte SP frame with mode zero and zeroed aim values; it revokes upper-computer control. |
