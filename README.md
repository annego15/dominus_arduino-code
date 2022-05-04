# Pins Arduino Uno

Pin | Function | Setting
----|----------|---------
A0 | NC | -
A1 | STEP | -
A2 | SW_MISO | -
A3 | CS | -
A4 | SW_SCK
A5 | SW_MOSI
A0 | Switch Test Kette | INPUT_PULLUP
A1 | Switch Test Band | INPUT_PULLUP
A2 | Switch Test Kette reverse| INPUT_PULLUP
A3 | Switch Test Band reverse | INPUT_PULLUP
A4 | Switch Test Falltür| INPUT_PULLUP
A5 | Switch Test Ausschieber| INPUT_PULLUP
D0 | RX | -
D1 | TX | -
D2 | DIAG1 TMC | INPUT_PULLUP
~D3 | Motor Kette Speed | OUTPUT
D4 | | -
~D5 | Servo Falltür | OUTPUT
~D6 | Servo Ausschieber 1 | OUTPUT
D7 | | -
D8 | Motor Band brake | OUTPUT
~D9 | (Motor Kette brake) / SPI MISO | OUTPUT
~D10 | Servo Ausschieber 2 | OUTPUT
~D11 | Motor Band Speed | OUTPUT
D12 | (Motor Kette dir) / SPI CLK | OUTPUT
D13 | Motor Band dir| OUTPUT
D18 | SDA Accel | -
D19 | SCL Accel | -





#define SWITCH_KETTE_PIN A0
#define SWITCH_BAND_PIN A1
#define SWITCH_KETTE_REVERSE_PIN A2
#define SWITCH_BAND_REVERSE_PIN A3
#define SWITCH_FALLTUER_PIN A4
#define SWITCH_AUSSCHIEBER_PIN A5