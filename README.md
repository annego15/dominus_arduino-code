# Pins Arduino Uno

Pin | Function | Setting
----|----------|---------
A0 | SW_MISO | -
A1 | CS | -
A2 | SW_SCK
A3 | SW_MOSI
A4 | SDA Accel | -
A5 | SCL Accel | -
D0 | RX | -
D1 | TX | -
D2 | DIAG1 TMC | INPUT_PULLUP
~D3 | Motor Kette Speed | OUTPUT
D4 | STEP | OUTPUT
~D5 | Servo Falltür | OUTPUT
~D6 | Servo Ausschieber 1 | OUTPUT
D7 | TEST_BUTTON | -
D8 | Motor Band brake | OUTPUT
~D9 | (Motor Kette brake) / SPI MISO | OUTPUT
~D10 | Servo Ausschieber 2 | OUTPUT
~D11 | Motor Band Speed | OUTPUT
D12 | (Motor Kette dir) / SPI CLK | OUTPUT
D13 | Motor Band dir | OUTPUT






#define SWITCH_KETTE_PIN A0
#define SWITCH_BAND_PIN A1
#define SWITCH_KETTE_REVERSE_PIN A2
#define SWITCH_BAND_REVERSE_PIN A3
#define SWITCH_FALLTUER_PIN A4
#define SWITCH_AUSSCHIEBER_PIN A5