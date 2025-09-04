# Initial source
From : https://github.com/Abdurraziq/ZMPT101B-arduino

# Usage

```
external_components:
  - source: github://rafal83/zmpt101b@main
    components: [zmpt101b]

sensor:
  - platform: zmpt101b
    name: "Voltage"
    id: voltage
    pin: A0
    frequency: 50
    sensitivity: 941.25
```
