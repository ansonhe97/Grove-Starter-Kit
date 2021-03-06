# Grove Starter Kit For Arduino

Grove Starter Kit for Arduino is one of the best Arduino Starter Kit for beginners. It includes one Arduino compatible Board and 10 additional Arduino sensors and all in one-piece of PCB design. **All the modules have been connected to the Seeeduino through the PCB stamp holes so no Grove cables are needed to connect**. Of course, you can also take the modules out and use Grove cables to connect the modules. You can build any Arduino project you like with this Grove Starter Kit.

## Hardware Overview

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218160437.jpg)

**Note:** Dimensions - 17.69 * 11.64 * 1.88cm

1. **[Grove - LED](http://wiki.seeedstudio.com/Grove-Red_LED/):** Simple LED module
2. **[Grove - Buzzer](http://wiki.seeedstudio.com/Grove-Buzzer/):** Piezo Buzzer
3. **[Grove - OLED Display 0.96"](http://wiki.seeedstudio.com/Grove-OLED-Display-0.96-SSD1315/):** 128×64 dot resolution High brightness,self-emission and high contrast ratio Big screen on a compact design Low power consumption.
4. **[Grove - Button](http://wiki.seeedstudio.com/Grove-Button/):** Push button for human input interfaces
5. **[Grove - Rotary Potentiometer](http://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/):** Rotary knob for human input interfaces
6. **[Grove - Light](http://wiki.seeedstudio.com/Grove-Light_Sensor/):** Detects surrounding light intensity
7. **[Grove - Sound](http://wiki.seeedstudio.com/Grove-Sound_Sensor/):** Detects surrounding sound intensity
8. **[Grove - Temperature & Humidity Sensor](http://wiki.seeedstudio.com/Grove-TemperatureAndHumidity_Sensor/):** Detects surrounding temperature and humidity values
9. **[Grove - Air Pressure Sensor](http://wiki.seeedstudio.com/Grove-Barometer_Sensor-BMP280/):** Detects surrounding atmospheric pressure
10. **[Grove - 3-Axis Accelerator](http://wiki.seeedstudio.com/Grove-3-Axis_Digital_Accelerometer-1.5g/):** Detects object acceleration
11. **[Seeeduino Lotus](http://wiki.seeedstudio.com/Seeeduino_Lotus/):** Arduino Compatible Board with Grove Ports 

!!!Note
        By default, Grove modules are connected to Seeeduino via PCB stamp holes. This means you don't need to use Grove cables to connect if not broken out. The default pins are as follow:

|Modules|Interface|Pins/Address|
|---|---|---|
|LED|Digital|D4|
|Buzzer|Digital|D5|
|OLED Display 0.96"|I2C|I2C, 0x78(default)|
|Button|Digital|D6|
|Rotary Potentiometer|Analog|A0|
|Light|Analog|A6|
|Sound|Analog|A2|
|Temperature & Humidity Sensor|Digital|D3|
|Air Pressure Sensor|I2C|I2C, 0x77(default) / 0x76(optional)|
|3-Axis Accelerator|I2C|I2C, 0x63(default)|

## Part List

|Modules|Quantity|
|---|---|
|**Sensors**||
|Temperature & Humidity Sensors|x1|
|3-Axis Accelerometers|x1|
|Air Pressure|x1|
|Light Sensor|x1|
|Sound Sensor|x1|
|**Input Modules**||
|Rotary Potentiometer|x1|
|Button|x1|
|**Output Modules**||
|LED|x1|
|Buzzer|x1|
|**Display Module**||
|OLED Display|x1|
|**Grove Cables**|x6|
|**Micro USB Cable**|x1|

## Learning Objectives

- Basics of Open Source Hardware Systems.
- Basic Arduino Programming.
- Communication principles and methods for sensors.
- Hands-on implementation of Open Source Hardware projects.

### Plug and Play Unboxing Demo

The Grove Starter Kit has a plug and play unboxing demo, where you first plug in the power to the board, you get the chance to experience all the sensors in one go! Use the button and rotary potentiometer to experience each sensor demo!

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200211115732.jpg)

- **Scroll** -> Rotating Rotary Potentiometer
- **Select** -> Short Press Button
- **Exit Current Demo** -> Long Press Button

Buzzer and LED module are used for key prompt.

## How to get started with Arduino

### Install the Arduino IDE

- **Arduino IDE** is an integrated development environment for Arduino, which is used for single-chip microcomputer software programming, downloading, testing and so on.
- Download and Install [Arduino IDE](https://www.arduino.cc/en/Main/Software) for your desired operating system here.

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200210114649.jpg)

### Install the USB driver

- Arduino connects to the PC via a USB cable. The USB driver depends on the type of USB chip you're using on your Arduino. *Note: USB chips are usually printed on the back of the development board.*

  - Download the [CP2102 USB Driver](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers). **Note:** Download according to your OS.
  - After the driver installation is completed, connect Arduino to the USB port of PC with USB cable. 
    - **For Windows users:** You can see it in `My Computer` -> `Properties` -> `Hardware` -> `Device Management`. A `COM` will appear.
    - **For Mac OS users:** You can navigate to `` on the top left corner, and choose `About this Mac` -> `System Report...` -> `USB`. A CP2102 USB Driver should appear.
  - If the driver is not installed, or if the driver is installed incorrectly (not matching the chip model), it will appear as an "unknown device" in the device manager. At this point, the driver should be reinstalled.

### Start the Arduino IDE

1. Open the **Arduino IDE** on your PC.
2. Click on `Tools` -> `Board` to select the correct Development Board Model. Select **Arduino/Genuino Uno** as Board.

![](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/board.png)

3. Click `Tools` -> `Port` to select the correct Port (the Serial Port shown in Device Manager in the previous step). In this case, `COM6` is selected. **For Mac OS users**, it should be `/dev/cu.SLAB_USBtoUART`.

![](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/port.png)

1. Create a new Arduino file and name it `Hello.ino`, then copy the following code into it:

```Cpp
void setup() {
  Serial.begin(9600); // initializes the serial port with a baud rate of 9600
}
void loop() {
  Serial.println("hello, world"); // prints a string to a serial port
  delay(1000); //delay of 1 second
}
```

5. In the upper left corner of the Arduino IDE, there are two buttons, **Verify and Upload**. First, press the Verify button(✓) to compile. After the compilation is successful, press the upload button(→).

![](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/func.png)

6. Navigate to `Tools` -> `Serial Monitor`, or click the **Serial Monitor** in the upper right corner(Magnifier Symbol), you can see the program running results:

![](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/print.png)

!!!Note
        All modules are pre-wired on a single circuit board, so no cables and soldering are needed. However, if you break  out the modules and want to connect them with Grove  cables, please kindly check Breakout Guide.

## Module Demo

### 1. LED

We have completed the output "Hello world" program. Now let's learn how to light the LED module. We know the three basic components of a control system: Input, Control, and Output. But lighting up LED uses only the output, not the input. Seeeduino is the control unit, LED module is the output unit and the output signal is digital signal.

#### Background Information

- **What is Digital Signal**

**Digital signal:** Digital signal refers to the value of the amplitude is discrete, the amplitude is limited to a finite number of values. In our controller, the digital signal has two states: LOW(0V) for 0; HIGH(5V) for 1. So sending a HIGH signal to LED can light it up.

![Alt text](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/digital.png)

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Grove Cable(If Broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174317.png)

- **Hardware connection:**
  - **Module connection**
    - Default connection by PCB stamp hole.
  - Connect the Seeeduino to the computer through the USB cable.
  
- **Software code:**
  - Open Arduino IDE.
  - Copy the following code, click Verify to check for syntax errors. Verify that there are no errors, and you can upload the code.

```Cpp
//LED Blink
//The LED will turn on for one second and then turn off for one second
int ledPin = 4;
void setup() {
    pinMode(ledPin, OUTPUT);
}
void loop() {
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
}
```

- **Software Analysis:**

```cpp
setup(){
}
```

The `setup()` function is called when a sketch starts. Use it to initialize variables, pin modes, start using libraries, etc. The `setup()` function will only run once, after each powerup or reset of the Arduino board.

```cpp
loop(){
}
```

After creating a `setup()` function, which initializes and sets the initial values, the `loop()` function does precisely what its name suggests, and loops consecutively, allowing your program to change and respond. Use it to actively control the Arduino board.

```cpp
int ledPin = 4;
```

**Description:**

Converts a value to the int data type.

**Syntax:**

int(**x**) or (int)**x** (C-style type conversion)

**Parameters:**

**x**: a value. Allowed data types: any type.

Assigned an `int` type 4 to variable named ledPin.

```cpp
pinMode(ledPin, OUTPUT);
```

**Description:**

Configures the specified pin to behave either as an input or an output. See the Digital Pins page for details on the functionality of the pins.

As of Arduino 1.0.1, it is possible to enable the internal pullup resistors with the mode `INPUT_PULLUP`. Additionally, the `INPUT` mode explicitly disables the internal pullups.

**Syntax:**

pinMode(**pin, mode**)

**Parameters:**

**pin**: the Arduino pin number to set the mode of.

**mode**: `INPUT`, `OUTPUT`, or `INPUT_PULLUP`.

Setting ledPin to the output mode.

```cpp
digitalWrite(ledPin, HIGH);
```

**Description:**

Write a `HIGH` or a `LOW` value to a digital pin.

If the pin has been configured as an OUTPUT with pinMode(), its voltage will be set to the corresponding value: 5V (or 3.3V on 3.3V boards) for `HIGH`, 0V (ground) for `LOW`.

If the pin is configured as an INPUT, digitalWrite() will enable (HIGH) or disable (LOW) the internal pullup on the input pin. It is recommended to set the pinMode() to `INPUT_PULLUP` to enable the internal pull-up resistor. See the Digital Pins tutorial for more information.

If you do not set the pinMode() to OUTPUT, and connect an LED to a pin, when calling digitalWrite(HIGH), the LED may appear dim. Without explicitly setting pinMode(), digitalWrite() will have enabled the internal pull-up resistor, which acts like a large current-limiting resistor.

**Syntax:**

digitalWrite(**pin, value**)

**Parameters:**

**pin**: the Arduino pin number.

**value**: `HIGH` or `LOW`.

When we set the ledPin as output, HIGH means sending high level to the pin, LED turns on.

```cpp
digitalWrite(ledPin, LOW);
```

When we set the led as output, low stands for sending low level to the pin, LED turns off.

```cpp
delay(1000);
```

**Description:**

Pauses the program for the amount of time (in milliseconds) specified as parameter. (There are 1000 milliseconds in a second.)

**Syntax:**

delay(**ms**)

**Parameters:** 

**ms**: the number of milliseconds to pause. Allowed data types: unsigned long.

Delay the program by 1000ms(1s).

- **Demo Effect and Serial Print Result:**

The LED module will be 1 second on and 1 second off.

- **Breakout Guide**

If modules are broken out from the board. Use a Grove cable to connect the **Grove LED** to Seeeduino Lotus's digital interface **D4**.  

### 2. Button

The first thing we need to know is that the input of the button is a digital signal, and there are only two states, 0 or 1, so we can control the output based on those two states.

- **Practice:** Use button to turn ON and OFF the LED module

- **Components Involved:**
    1. Seeeduino Lotus
    2. Grove LED
    3. Grove Button
    4. Grove Cables(If broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174437.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Hardware analysis**:
  - Input: Button
  - Control: Seeeduino
  - Output: LED module

Both the sensor and the LED use digital signals, so they should be connected to digital interfaces.

- **Software code**:
  - Open Arduino IDE.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

```Cpp
//Button to turn ON/OFF LED
//Constants won't change. They're used here to set pin numbers:
const int buttonPin = 6;     // the number of the pushbutton pin
const int ledPin =  4;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
}
```

- **Code analysis:**

```cpp
pinMode(ledPin, OUTPUT);
```

Define LED as the output unit.

```cpp
pinMode(buttonPin, INPUT);
```
  
Define button as the input unit.

```cpp
buttonState = digitalRead(buttonPin);
```

**Description:**

Reads the value from a specified digital pin, either `HIGH` or `LOW`.

**Syntax:**

digitalRead(**pin**)

**Parameters:**

**pin**: the Arduino `pin` number you want to read

This function is used to read the states of digital pins, either HIGH or LOW. When the button is pressed, the state is HIGH, otherwise is LOW.

```cpp
  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}
```

**Description:**

The if…​else allows greater control over the flow of code than the basic if statement, by allowing multiple tests to be grouped. An else clause (if at all exists) will be executed if the condition in the if statement results in false. The else can proceed another if test, so that multiple, mutually exclusive tests can be run at the same time.

Each test will proceed to the next one until a true test is encountered. When a true test is found, its associated block of code is run, and the program then skips to the line following the entire if/else construction. If no test proves to be true, the default else block is executed, if one is present, and sets the default behavior.

Note that an else if block may be used with or without a terminating else block and vice versa. An unlimited number of such else if branches are allowed.

**Syntax:**

```cpp
if (condition1) {
  // do Thing A
}
else if (condition2) {
  // do Thing B
}
else {
  // do Thing C
}
```

The usage of the statement is: if the logical expression in parentheses is true, execute the statement in curly braces after **if**, if not, execute the statement in curly braces after the **else**. If the state of the button is high, the LED pin outputs a high level and turn the LED on, else turn LED off.

- **Demo Effect and Serial Print Result:**

Pressing the button will turn the LED module on.

- **Breakout Guide**

Use a Grove cable to connect the Grove LED to Seeeduino Lotus's digital interface **D4**. Connect the Grove Button to digital interface **D6**.

### 3. Rotary Potentiometer

In the last section, we studied that button only has two states, ON/OFF state corresponding 0V and 5V, but in practice, we often counter the need for many states, not just 0V and 5V. Then you need to use Analog Signal! Rotary Potentiometer is a classic example that uses analog signal.

#### Background Information

- **What is Analog Signal**

**Analog signals:** Signals vary continuously in time and value, and the amplitude, frequency, or phase of the signal changes continuously at any time, such as the current broadcast sound signal, or image signal, etc.The analog signal has sine wave and triangle wave and so on.The analog pins of your microcontroller can have between 0V and 5V is mapped to a range between 0 and 1023 where 1023 is mapped as 5V and 512 is mapped as 2.5v and etc.

![Alt text](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/analog.png)

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Grove Rotary Switch
  4. Grove Cables(If broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174521.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Hardware analysis:**

  - Input: Rotary Potentiometer
  - Control: Seeeduino Lotus
  - Output: LED module

The input is an analog signal, so it is connected to the analog signal interface, the LED module is connected to the digital signal interface.

- **Software code:**
  - Open Arduino IDE.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

```Cpp
//Rotary controls LED
int rotaryPin = A0;    // select the input pin for the rotary
int ledPin = 4;      // select the pin for the LED
int rotaryValue = 0;  // variable to store the value coming from the rotary

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // read the value from the sensor:
  rotaryValue = analogRead(rotaryPin);
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  delay(rotaryValue);
  // turn the ledPin off:
  digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds:
  delay(rotaryValue);
}
```

- **Code analysis:**

```cpp
rotaryValue = analogRead(rotaryPin);
```

**Description:**

Reads the value from the specified analog pin. Arduino boards contain a multichannel, 10-bit analog to digital converter. This means that it will map input voltages between 0 and the operating voltage(5V or 3.3V) into integer values between 0 and 1023. On an Arduino UNO, for example, this yields a resolution between readings of: 5 volts / 1024 units or, 0.0049 volts (4.9 mV) per unit.

**Syntax:**

analogRead(**pin**)

**Parameters:**

**pin**: the name of the analog input pin to read from (A0 to A5 on most boards).

**Returns:** The analog reading on the pin. Although it is limited to the resolution of the analog to digital converter (0-1023 for 10 bits or 0-4095 for 12 bits). Data type: int.

This function is used to read the value of Analog pins(the rotary sensor position), the range of values is: 0 ~ 1023.

```cpp
delay(rotaryValue);
```

Delay function, The millisecond duration of the delay is the value in parentheses. Because the value is the value of the analog signal of the knob pin being read, so the delay time can be controlled by the knob.

- **Demo Effect and Serial Print Result:**

Turning the Potentiometer will change the frequency of LED flickering.

- **Breakout Guide**

Use a Grove cable to connect LED to Seeeduino Lotus's digital interface **D4**, and a Grove cable to connect the Grove Rotary Switch to analog signal interface **A0**.

### 4. Buzzer

Just like the LED module, Buzzer is also an output module, instead of lighting up it produces a beep sound. This can be used for many situations for indication purposes.  We studied the use of potentiometer in the last section, so how do we use the potentiometer to control the volume of the buzzer? This requires the used of PWM control!

#### Background Information

- **What is PWM**

**Pulse Width Modulation, or PWM**, is a technique for getting analog results with digital means. Digital control is used to create a square wave, a signal switched between on and off. This on-off pattern can simulate voltages in between full on (5 Volts) and off (0 Volts) by changing the portion of the time the signal spends on versus the time that the signal spends off. The duration of "on time" is called the pulse width. To get varying analog values, you change, or modulate, that pulse width. If you repeat this on-off pattern fast enough with an LED for example, the result is as if the signal is a steady voltage between 0 and 5v controlling the brightness of the LED. *Reference: [Arduino](https://www.arduino.cc/en/tutorial/PWM)*

As the diagram indicates below, use `analogWrite()` to generate PWM waves, the higher the percentage of Duty Cycle, the louder the buzzer.

<div align=center><img src="https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200211170133.jpg"/></div>

There are six digital pins on your Seeeduino that are marked with the symbol “~”, which means they can send out a PWM signal : 3,5,6,9,10,11. They are celled PWM pins.

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove Buzzer
  3. Grove Cable(If Broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174543.png)

- **Hardware connection:**
  - **Module connection**
    - Default connection by PCB stamp hole.
  - Connect the Seeeduino to the computer through the USB cable.
  
- **Software code:**
  - Open Arduino IDE.
  - Copy the following code, click Verify to check for syntax errors. Verify that there are no errors, and you can upload the code.

```Cpp
int BuzzerPin = 5;
int Potentiometer = A0;

void setup() {
  pinMode(BuzzerPin, OUTPUT);
}

void loop() {
  int potentioValue, Value;
  potentioValue = analogRead(Potentiometer);
  Value = map(potentioValue, 0, 1023, 0, 255); //Mapping potentiometer value to PWM signal value
  analogWrite(BuzzerPin, Value);
}
```

- **Software Analysis:**

```cpp
analogWrite(BuzzerPin, Value);
```

**Description:**

Writes an analog value (PWM wave) to a pin. Can be used to light a LED at varying brightnesses or drive a motor at various speeds. After a call to analogWrite(), the pin will generate a steady rectangular wave of the specified duty cycle until the next call to analogWrite() (or a call to digitalRead() or digitalWrite()) on the same pin.

**Syntax:**

analogWrite(**pin, value**)

**Parameters:**

**pin**: the Arduino `pin` to write to. Allowed data types: int. 

**value**: the duty cycle: between `0` (always off) and `255` (always on). Allowed data types: int.

Writes an analog value (PWM wave) to the Buzzer.

- **Demo Effect and Serial Print Result:**

The buzzer beeps 3 times fast at startup, waits a second then beeps continuously
 at a slower pace.

- **Breakout Guide**

Use a Grove cable to connect the Grove Buzzer to Seeeduino Lotus's digital interface **D5**.

### 5. Light Sensor

The light sensor contains a photosensitive resistor to measure the intensity of light. The resistance of the photosensitive resistor decreases with the increase of light intensity. The output signal is the analog value, the brighter the light source, the larger the analog value. Based on this property, you can use it to make a light switch.

In the following sections, we will use Serial Monitor to observe results from our sensors so here comes the brief introduction!

#### Background Information

- **What is Serial Monitor**

Serial Monitor is a useful tool to observe results on Arduino, it can be very useful in terms of printing results from the sensors or debugging in general. You can also send data back to the controller via serial monitor to do certain tasks! Note: Make sure the Serial data transfer match with the code.

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200211150116.jpg)

You can open the Serial Plotter by clicking **Tools** -> **Serial Monitor**.

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200217144001.jpg)

- **Exercise:** As the environment slowly brightens, the LED lights will lighten. As the light slowly dimmed, the LED dimmed. The LED will go from dark to light or from light to dark. To achieve this, we will use pulse width modulation(PWM).
  
- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Grove Light Sensor
  4. Grove Cable(If broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174604.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Hardware analysis:**

  - Input: Light Sensor
  - Control: Seeeduino Lotus
  - Output: LED module

- **Software code:**
  - Open Arduino IDE.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

```Cpp
// Light Switch
int sensorpin = A6; // Analog input pin that the sensor is attached to
int ledPin = 4;    // LED port
int sensorValue = 0;        // value read from the port
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  pinMode(ledPin,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(sensorpin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  Serial.println(sensorValue);
  // change the analog out value:
  analogWrite(ledPin, outputValue);
  delay(30);
}
```

You can also see the light intensity readings from the **Serial Monitor**, navigate to **Tools** -> **Serial Monitor**.

- **Code Analysis:**

```cpp
Serial.begin(9600);
```

**Description:**

Sets the data rate in bits per second (baud) for serial data transmission. For communicating with Serial Monitor, make sure to use one of the baud rates listed in the menu at the bottom right corner of its screen. You can, however, specify other rates - for example, to communicate over pins 0 and 1 with a component that requires a particular baud rate.

An optional second argument configures the data, parity, and stop bits. The default is 8 data bits, no parity, one stop bit.

The software running on the computer communicates with the development board, and the baud rate is 9600.

**Syntax:**

Serial.begin(**speed**)

**Parameters:**

**speed**: Speed of Serial communication. i.e `9600`, `115200` and etc.

Set the Serial baud rate to 9600.

```cpp
Serial.println(sensorValue);
```

**Description:**

Prints data to the serial port as human-readable ASCII text followed by a carriage return character (ASCII 13, or '\r') and a newline character (ASCII 10, or '\n'). This command takes the same forms as Serial.print().

**Syntax:**

Serial.println(**val**) or Serial.println(**val**, **format**)

**Parameters:** 

**val**: the value to print. Allowed data types: any data type.

**format**: specifies the number base (for integral data types) or number of decimal places (for floating point types).

Serial port print the Light sensor’s value.  So you open the **serial monitor** on the IED interface, and you see the value of the output sensor.

```cpp
outputValue = map(sensorValue, 0, 1023, 0, 255);
```

**Description:**

Re-maps a number from one range to another. That is, a value of **fromLow** would get mapped to **toLow**, a value of **fromHigh** to **toHigh**, values in-between to values in-between, etc.

Does not constrain values to within the range, because out-of-range values are sometimes intended and useful. The `constrain()` function may be used either before or after this function, if limits to the ranges are desired.

Note that the "lower bounds" of either range may be larger or smaller than the "upper bounds" so the `map()` function may be used to reverse a range of numbers, for example

**y = map(x, 1, 50, 50, 1);**

The function also handles negative numbers well, so that this example

**y = map(x, 1, 50, 50, -100);**

is also valid and works well.

The map() function uses integer math so will not generate fractions, when the math might indicate that it should do so. Fractional remainders are truncated, and are not rounded or averaged.

**Syntax:**

map(**value, fromLow, fromHigh, toLow, toHigh**)

**Parameters:**

**value**: the number to map.

**fromLow**: the lower bound of the value’s current range.

**fromHigh**: the upper bound of the value’s current range.

**toLow**: the lower bound of the value’s target range.

**toHigh**: the upper bound of the value’s target range.

Mapping light sensor analog signal(0 to 1023)to brightness value of LED(0 to 255).

Keep equal and potentiometer after mapping the value of the time. Map has five parameters, which in turn is: to map the original value, the original value of the minimum value, original value maximum, minimum value after the mapping, mapping the maximum. In this way, the data returned by the sensor can be mapped from its original value of 0-1023 to 0-255.

```cpp
analogWrite(ledPin,255-outputvalue);
```

The function is used to write an analog value between 0 - 255 a PWM pin. analogWrite() can only be used for PWM pins. The new mapping data in the previous statement can be output to ledPin to lighten / dim the LED.

- **Demo Effect and Serial Print Result:**

The LED module will change its intensity according to the light intensity of surrounding. The darker the surrounding, the lighter it gets.

- **Breakout Guide**

Use Grove Cable to connect the Grove LED to Seeeduino Lotus's digital signal interface **D4**,connect the Grove Light Sensor to Seeeduino Lotus's analog signal interface **A1**.

### 6. Sound Sensor

The sound sensor can detect the sound intensity of the environment, and its output is also simulated. I'm sure you've all been exposed to the sound control lights, but now we can do one ourselves, and with the basics, this experiment will be easy for you. Here used Serial Plotter to visualize results.

#### Background Information

- **What is Serial Plotter**

Serial Plotter is similar to Serial Monitor, allowing you to natively graph serial data from your Arduino to your computer in real time. This is very useful when data needs to be visualized.

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200212105028.png)

You can open the Serial Plotter by clicking **Tools** -> **Serial Plotter**.

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200217143635.jpg)

- **Practice:** The LED lights light up when the sound is made. When there is no sound and it is very quiet, the LED lights go off.

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Grove Sound Sensor
  4. Grove cable(If broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174633.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Copy the following code, click Verify to check for syntax errors. Verify that there are no errors, and you can upload the code.

```Cpp
//Sound Control Light
int soundPin = A2; // Analog sound sensor is to be attached to analog
int ledPin = 4; // Digital LED is to be attached to digital
void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}
void loop(){
  int soundState = analogRead(soundPin); // Read sound sensor’s value
  Serial.println(soundState);
  // if the sound sensor’s value is greater than 200, the light will be on for 5 seconds.
  //Otherwise, the light will be turned off
  if (soundState > 200) {
    digitalWrite(ledPin, HIGH);
    delay(100);
  }else{
    digitalWrite(ledPin, LOW);
  }
}
```

You can also see the light intensity readings from the **Serial Monitor**, navigate to **Tools** -> **Serial Plotter**.

- **Code analysis:**

```cpp
Serial.begin(9600);
```

The software running on the computer communicates with the development board, and the baud rate is 9600.

```cpp
Serial.print(" ");
```

This function is used to output data from the serial port, the output is what is contained in the double quotation marks.

```cpp
Serial.println( );
```

This statement is similar to the one above, except that **serial.println** has a newline return.

```cpp
Serial.println(soundState);
```

Serial port print the sound sensor’s value.  So you open the **serial monitor** on the IED interface, and you see the value of the output sensor.

- **Demo Effect and Serial Print Result:**

The LED module will light up if the surrounding is loud enough.

- **Breakout Guide**

Use Grove cables to connect the Grove LED to Seeeduino Lotus's digital signal interface **D4**, Connect the Grove Sound Sensor to Seeeduino Lotus's analog signal interface **A2**.

### 7. OLED Display

OLED Display can be used for many situations, where you could use it to visualize sensor readings!

#### Background Information

- **What are Arduino Libraries**

The Arduino environment can be extended through the use of libraries, just like most other programming platforms. Libraries provide extra functionalities for use in sketches, i.e. working with specific hardware or manipulating data. To use a library in a sketch, select it from **Sketch** ->**Include Library**.

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200210120746.jpg)

For more information, please also visit [How to install Arduino Libraries](http://wiki.seeedstudio.com/How_to_install_Arduino_Library/).

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove OLED
  3. Grove cable(If broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174720.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Install the **U8g2 library**: Navigate to **Sketch** -> **Include Library** -> **Manage Libraries...** and Search for the keyword "**U8g2**" in the **Library Manager**, then install.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

```cpp
#include <Arduino.h>
#include <U8x8lib.h>

U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

void setup(void) {
  u8x8.begin();
  u8x8.setFlipMode(1);
}

void loop(void) {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 0);
  u8x8.print("Hello World!");
}
```

- **Code analysis**

```cpp
#include <>
```

**Description:**

`#include` is used to include outside libraries in your sketch. This gives the programmer access to a large group of standard C libraries (groups of pre-made functions), and also libraries written especially for Arduino.

Note that `#include`, similar to `#define`, has no semicolon terminator, and the compiler will yield cryptic error messages if you add one.

**#include** is an instruction that introduces a header file. Here we use the DHT.h, <Arduino.h>, <U8g2lib.h>, <SPI.h>, <Wire.h> library, these library are included in Arduino IDE.

```cpp
#define
```

**Description:**

`#define` is a useful C++ component that allows the programmer to give a name to a constant value before the program is compiled. Defined constants in arduino don’t take up any program memory space on the chip. The compiler will replace references to these constants with the defined value at compile time.

Force a variable to be the value you want.

```cpp
U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
```

Once the object is declared, you can use functions from the library.

```cpp
u8x8.begin();
```

**Description:**

Simplified setup procedure of the display for the Arduino environment. See the setup guide for the selection of a suitable U8g2 constructor.

**Syntax:**

u8x8.begin()

Initialize the u8g2 library

```cpp
u8x8.setFlipMode(1);
```

**Description:**

Some displays support a 180 degree rotation of the internal frame buffer. This hardware feature can be controlled with this procedure. Important: Redraw the complete display after changing the flip mode. Best is to clear the display first, then change the flip mode and finally redraw the content. Results will be undefined for any existing content on the screen.

**Syntax:**

u8x8.setFlipMode(**mode**)

**Parameters:**

**mode**: `0` or `1`

Flips the display 180 degree.

```cpp
u8x8.setCursor();
```

**Description:**

Define the cursor for the print function. Any output of the print function will start at this position.

**Syntax:**

u8x8.setCursor(**x, y**)

**Parameters:**

**x, y**: Column/row position for the cursor of the print function.

Sets the draw cursor position.

```cpp
u8x8.setFont()
```

**Description:** 

Define a u8x8 font for the glyph and string drawing functions.

**Syntax:**

u8x8.setFont(***font_8x8**)

Set the font for display.

```cpp
u8x8.print();
```

Draw the content on the OLED.

- **Demo Effect and Serial Print Result:**

Prints Hello World onto the OLED Display.

- **Breakout Guide**

Use Grove cable to connect the OLED to Seeeduino Lotus's **I2C** interface (Note: I2C's default address is 0x78).

### 8. Temperature and Humidity Sensor

Have you ever wondered about the temperature and humidity of your surroundings? Want to know the exact number? Want to wear a skirt or coat today depending on the temperature?Let's make a temperature meter!

#### Background Information

- **What is Protocol Signal (I2C)**

**Protocol signal:** the protocol signal we use is I2C, so here is a brief introduction to I2C. I2C bus just need two wires in the transmission of information connection between the devices: the SDA (Serial Data Line) and SCL (Serial Clock line). These two lines are bidirectional I/O lines, the main component used to start the bus transfer data, and generate the clock to open transmission device, any devices that are addressing at this time is considered from the device. The relationship between master and slave, sender and receiver on the bus is not constant, but depends on the direction of data transmission. If the host wants to send data to the slave device, the host first addresses the slave device, then actively sends data to the slave device, and finally terminates the data transmission by the host.If the host is to receive data from the slave, the slave is first addressed by the master. The host then receives the data sent from the device, and the host terminates the receiving process. In this case.The host is responsible for generating the timing clock and terminating the data transfer.

- **Practice:** Let your OLED display display the current ambient temperature and humidity.
- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove OLED
  3. Grove Temperature and Temperature Sensor
  4. Grove cable(If broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174754.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Install the **Grove Temperature and Humidity Sensor(DHT11) library**: Navigate to **Sketch** -> **Include Library** -> **Manage Libraries...** and Search for the keyword "**Grove Temperature and Humidity Sensor(DHT11)**" in the **Library Manager**, then install.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

```Cpp
//Temperature and Humidity Sensor
//Temperature and Humidity Sensor
#include "DHT.h"
#include <Arduino.h>
#include <U8x8lib.h>

#define DHTPIN 3     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11 
DHT dht(DHTPIN, DHTTYPE);

U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

void setup(void) {
  Serial.begin(9600); 
  Serial.println("DHTxx test!");
  dht.begin();
  u8x8.begin();
  u8x8.setPowerSave(0);  
  u8x8.setFlipMode(1);
}

void loop(void) {

  float temp, humi;
  temp = dht.readTemperature();
  humi = dht.readHumidity();
  
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 33);
  u8x8.print("Temp:");
  u8x8.print(temp);
  u8x8.print("C");
  u8x8.setCursor(0,50);
  u8x8.print("Humidity:");
  u8x8.print(humi);
  u8x8.print("%");
  u8x8.refreshDisplay();
  delay(200);
}
```

- **Code analysis:**

```cpp
float temp, humi;
```

Defines variables to store readings.

```cpp
temp = dht.readTemperature();
humi = dht.readHumidity();
```

**Description:**

Functions to be used to read temperature and humidity values from the sensor.

**Syntax:**

**dht.readTemperature()** and **dht.readHumidity()**. Return type: float.

Call these functions to read the temperature and humidity and store them in defined variables.

- **Demo Effect and Serial Print Result:**

The surrounding temperature and humidity appears on the OLED screen.

- **Breakout Guide**

Use Grove cable to connect the OLED to Seeeduino Lotus's **I2C** interface (Note: I2C's default address is 0x78). Connect the Grove Temperature and Humidity Sensor to Seeeduino Lotus's digital signal interface **D3**.

### 9. Air Pressure Sensor

Grove Air Pressure Sensor(BMP280) is a breakout board for Bosch BMP280 high-precision and low-power digital barometer. This module can be used to measure temperature and atmospheric pressure accurately. As the atmospheric pressure changes with altitude, it can also measure approximate altitude of a place.

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove Air Pressure Sensor
  3. Grove cable(if broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174824.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Install the **Grove Barometer Sensor library**: Navigate to **Sketch** -> **Include Library** -> **Manage Libraries...** and Search for the keyword "**Grove BMP280**" in the **Library Manager**, then install.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.
  - In this program, acceleration information are sent from the sensor to Seeeduino via I2C bus and then Seeeduino printed them onto the serial monitor. Open the **serial monitor** to check the result.

```Cpp
//Air pressure detection
#include "Seeed_BMP280.h"
#include <Wire.h>

BMP280 bmp280;

void setup() {
    Serial.begin(9600);
    if (!bmp280.init()) {
        Serial.println("Device not connected or broken!");
    }
}

void loop() {

    float pressure;

    //get and print temperatures
    Serial.print("Temp: ");
    Serial.print(bmp280.getTemperature());
    Serial.println("C"); // The unit for  Celsius because original arduino don't support speical symbols

    //get and print atmospheric pressure data
    Serial.print("Pressure: ");
    Serial.print(pressure = bmp280.getPressure());
    Serial.println("Pa");

    //get and print altitude data
    Serial.print("Altitude: ");
    Serial.print(bmp280.calcAltitude(pressure));
    Serial.println("m");

    Serial.println("\n");//add a line between output of different times.

    delay(1000);
}
```

- **Code analysis:**

```cpp
#include <Wire.h>
```

**#include** is an instruction that introduces a header file. Here we use the <Wire.h> library, this library is included in Arduino IDE. 

```cpp
#include "Seeed_BMP280.h"
```

Represents the Seeed_BMP280.h header file that introduces the current path.

```cpp
if (!bmp280.init()) {
    Serial.println("Device not connected or broken!");
}
```

**Description:**

Initialize the air pressure sensor.

**Syntax:**

**bmp280.init()**

if the Air pressure sensor did not start properly, then prints out error to serial monitor.

```cpp
Serial.print(bmp280.getTemperature());
```

**Description:**

Functions to be used to read temperature value from the sensor.

**Syntax:**

**bmp280.getTemperature()**. Return type: float

Prints the the temperature data to serial monitor.

```cpp
Serial.print(pressure = bmp280.getPressure());
```

**Description:**

Functions to be used to read air pressure value from the sensor.

**Syntax:**

**bmp280.getPressure()**. Return type: float

Prints the current air pressure.

```cpp
Serial.print(bmp280.calcAltitude(pressure));
```

**Description:**

Takes the pressure value can convert to altitude.

**Syntax:**

bmp280.calcAltitude(**float**). Return type: float

**Parameter:**

**float**: Pressure value.

Prints the amplitude.

- **Demo Effect and Serial Print Result:**

The Air pressure readings are display on the Serial Monitor.

- **Breakout Guide**

Use Grove cable to connect Grove 3-axis Accelerometer to Seeeduino Lotus's **I2C** interface using a Grove cable (note: I2C default address is 0x77 or 0x76).

### 10. 3-Axis Accelerometer

This is the last sensor, the triaxial accelerometer, and with this module, you can easily add motion monitoring to your design. So we can do a lot of interesting little experiments on the basis of motion.

- **Practice:** when motion is detected, the buzzer gives an alarm indicating that the object is in motion.

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove 3-axis Accelerometer
  3. Grove cable(if broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174853.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Download the [3-Axis Digital Accelerometer( ±2g to 16g)](https://github.com/Seeed-Studio/Seeed_Arduino_LIS3DHTR)  from Github. Click on **Sketch** > **Include library** > **Add .ZIP library**, import the library into the IED.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.
  - In this program, acceleration information are sent from the sensor to Seeeduino via I2C bus and then Seeeduino printed them onto the serial monitor. Open the **serial monitor** to check the result.

```Cpp
//Gravity Acceleration
#include "LIS3DHTR.h"
#ifdef SOFTWAREWIRE
    #include <SoftwareWire.h>
    SoftwareWire myWire(3, 2);
    LIS3DHTR<SoftwareWire> LIS(I2C_MODE);//IIC
    #define WIRE myWire
#else
    #include <Wire.h>
    LIS3DHTR<TwoWire> LIS(I2C_MODE);//IIC
    #define WIRE Wire
#endif

void setup() {
    Serial.begin(9600);
    while (!Serial) {};
    LIS.begin(WIRE); //IIC init
    delay(100);
    LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
}
void loop() {
    if (!LIS) {
        Serial.println("LIS3DHTR didn't connect.");
        while (1);
        return;
    }
    //3 axis
    Serial.print("x:"); Serial.print(LIS.getAccelerationX()); Serial.print("  ");
    Serial.print("y:"); Serial.print(LIS.getAccelerationY()); Serial.print("  ");
    Serial.print("z:"); Serial.println(LIS.getAccelerationZ());

    delay(500);
}
```

- **Code analysis:**

```cpp
#include "LIS3DHTR.h"
#ifdef SOFTWAREWIRE
    #include <SoftwareWire.h>
    SoftwareWire myWire(3, 2);
    LIS3DHTR<SoftwareWire> LIS(I2C_MODE);//IIC
    #define WIRE myWire
#else
    #include <Wire.h>
    LIS3DHTR<TwoWire> LIS(I2C_MODE);//IIC
    #define WIRE Wire
#endif
```

Initializing the module using software I2C or hardware I2C.

```cpp
while (!Serial) {};
```

Code stops here if don't open the serial monitor, so open serial monitor.

```cpp
LIS.begin(WIRE);
LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
```

**Description:** Initialize the accelerator.

**Syntax:** `LIS.begin(Wire)`.

**Description:** Sets the output data rate of the accelerator.

**Syntax:** `LIS.setOutputDataRate(odr_type_t odr)`.

Initialize the accelerator and set the output rate to 50Hz.

```cpp
Serial.print("x:"); Serial.print(LIS.getAccelerationX()); Serial.print("  ");
Serial.print("y:"); Serial.print(LIS.getAccelerationY()); Serial.print("  ");
Serial.print("z:"); Serial.println(LIS.getAccelerationZ());
```

**Description:** 

Functions to be used to read X-axis value from the sensor.

**Syntax:**

**LIS.getAccelerationX()**. Return type: float.

**Description:** 

Functions to be used to read Y-axis value from the sensor.

**Syntax:**

**LIS.getAccelerationY()**. Return type: float.

**Description:** 

Functions to be used to read Z-axis value from the sensor.

**Syntax:**

**LIS.getAccelerationZ()**. Return type: float.

Prints the 3 axis data to serial monitor.

- **Demo Effect and Serial Print Result:**

The 3-axis accelerator readings are display on the Serial Monitor.

- **Breakout Guide**

Use Grove cable to connect Grove 3-axis Accelerometer to Seeeduino Lotus's **I2C** interface using a Grove cable (note: I2C default address is 0x4c).

## Projects

### 1. Music dynamic rhythm lamp

- **Project description:** In this experiment, we will make the buzzer play pleasant music and the led lights flash according to the music frequency and beat.

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Buzzer
  4. Grove Cables(if broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218174938.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

```cpp
//Music Dynamic Rhythm Lamp
#define NTD0 -1
#define NTD1 294
#define NTD2 330
#define NTD3 350
#define NTD4 393
#define NTD5 441
#define NTD6 495
#define NTD7 556

#define NTDL1 147
#define NTDL2 165
#define NTDL3 175
#define NTDL4 196
#define NTDL5 221
#define NTDL6 248
#define NTDL7 278

#define NTDH1 589
#define NTDH2 661
#define NTDH3 700
#define NTDH4 786
#define NTDH5 882
#define NTDH6 990
#define NTDH7 112

#define WHOLE 1
#define HALF 0.5
#define QUARTER 0.25
#define EIGHTH 0.25
#define SIXTEENTH 0.625

int tune[]=
{
NTD3,NTD3,NTD4,NTD5,
NTD5,NTD4,NTD3,NTD2,
NTD1,NTD1,NTD2,NTD3,
NTD3,NTD2,NTD2,
NTD3,NTD3,NTD4,NTD5,
NTD5,NTD4,NTD3,NTD2,
NTD1,NTD1,NTD2,NTD3,
NTD2,NTD1,NTD1,
NTD2,NTD2,NTD3,NTD1,
NTD2,NTD3,NTD4,NTD3,NTD1,
NTD2,NTD3,NTD4,NTD3,NTD2,
NTD1,NTD2,NTDL5,NTD0,
NTD3,NTD3,NTD4,NTD5,
NTD5,NTD4,NTD3,NTD4,NTD2,
NTD1,NTD1,NTD2,NTD3,
NTD2,NTD1,NTD1
};

float durt[]=
{
1,1,1,1,
1,1,1,1,
1,1,1,1,
1+0.5,0.5,1+1,
1,1,1,1,
1,1,1,1,
1,1,1,1,
1+0.5,0.5,1+1,
1,1,1,1,
1,0.5,0.5,1,1,
1,0.5,0.5,1,1,
1,1,1,1,
1,1,1,1,
1,1,1,0.5,0.5,
1,1,1,1,
1+0.5,0.5,1+1,
};

int length;
int tonepin=5;
int ledp=4;

void setup()
{
  pinMode(tonepin,OUTPUT);
  pinMode(ledp,OUTPUT);
  length=sizeof(tune)/sizeof(tune[0]);
}

void loop()
{
  for(int x=0;x<length;x++)
  {
    tone(tonepin,tune[x]);
    digitalWrite(ledp, HIGH); 
    delay(400*durt[x]);
    digitalWrite(ledp, LOW);
    delay(100*durt[x]);
    noTone(tonepin);

  }
  delay(4000);
}
```

- **Code analysis:**
  
```cpp
#define NTD
```

Here is the definition of the frequency of the D key, which is divided into bass, alto, and treble.

```cpp
#define WHOLE 1
#define HALF 0.5
#define QUARTER 0.25
#define EIGHTH 0.25
#define SIXTEENTH 0.625
```

Note: rhythm is divided into one beat, half beat, 1/4 beat, 1/8 beat, we specify a beat note time is 1;Half beat is 0.5;1/4 beat is 0.25;1/8 of 0.125.

```cpp
int tune[]=...
```

List the frequencies according to the spectrum.

```cpp
float durt[]=...
```

List the beats according to the spectrum.

```cpp
delay(100*durt[x]);
```

Control LED lights on and off respectively.

- **Demo Effect and Serial Print Result:**

The buzzer will beep a tune while the LED module will flicker with same the frequency.

- **Breakout Guide**

Connect Grove LED to Seeeduino Lotus's digital signal interface **D4**, connect Buzzer to Seeeduino Lotus's digital signal interface **D5**.

### 2. Make an intelligent sound-light induction desk lamp

- **Project description:** as the name implies, this project is to make a small lamp controlled by Sound and Light. We need to use LED module. Of course, Light Sensor and Sound Sensor are also indispensable. In this way, you can achieve the function of the smart desk lamp: when the sound, the lamp will light up;If the environment turns dark, the lamp will automatically turn brighter.

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Light Sensor
  4. Sound Sensor
  5. Grove cable(If broken out)

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200218175138.png)

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

```Cpp
//light Induction Desk Lamp
int soundPin = A2; // Analog sound sensor is to be attached to analog
int lightPin = A6; //Analog light sensor is to be attached to analog
int ledPin = 4; // Digital LED is to be attached to digital

void setup() {
  pinMode(ledPin, OUTPUT);
}
void loop(){
  int soundState = analogRead(soundPin); // Read sound sensor’s value
  int lightState = analogRead(lightPin); // Read light sensor’s value
  // if the sound sensor's value is greater than 50 or the sound sensor's is less than 10, the light will be on.
  //Otherwise, the light will be turned off
if (soundState > 50 || lightState < 10) {
  digitalWrite(ledPin, HIGH);
  //delay(5000); //You can delete the "//" to make the LED on for five seconds
}else{
  digitalWrite(ledPin, LOW);
}
}
```

- **Code analysis:**

```cpp
if (soundState > 50 || lightState < 10) {
  ...
}
```

In parentheses is a logical expression. Both **&&** and **||** are commonly used in logical expressions. The common usage is **if (expression 1 || expression 2)** and **if (expression 1 && expression 2)**.

**||** represents "**or**", satisfies one of them, the whole expression is true, and satisfies the condition of the if judgment.

**&&** means "**and**", the statement in if{} is executed only if all expressions in parentheses are true.

- **Demo Effect and Serial Print Result:**

If the surrounding sound is loud enough or light intensity is low, the LED module will light up more intensity.

- **Breakout Guide**

Connect the Grove LED to Seeeduino Lotus's digital signal interface **D4**, Connect the Light Sensor to Seeeduino Lotus's analog signal interface **A1**. Connect the Sound Sensor to Seeeduino Lotus's analog signal interface **A2** using a Grove cable.

## Resources

1. **Modules Libraries on Github:**
   - [OLED Display](https://github.com/olikraus/U8g2_Arduino)
   - [Temperature & Humidity Sensor](https://github.com/Seeed-Studio/Grove_Temperature_And_Humidity_Sensor)
   - [Air Pressure Sensor](https://github.com/Seeed-Studio/Grove_BMP280)
   - [3-Axis Accelerator](https://github.com/Seeed-Studio/Seeed_Arduino_LIS3DHTR)

2. [**Sensor Datasheet**](https://github.com/SeeedDocument/Grove-Starter-Kit-For-Arduino/raw/master/res/Grove-starter-kit-for-arduino-datasheet.zip)

3. [**Initial Arduino Firmware Demo**](https://github.com/SeeedDocument/Grove-Starter-Kit-For-Arduino/raw/master/res/GroveStarterKitFirmware.zip)

## More Learning

- [LSTM for live IoT data prediction](https://github.com/256ericpan/LSTM_IoT)

## Tech Support
Please submit any technical issue into our [forum](http://forum.seeedstudio.com/)<br /><p style="text-align:center"><a href="https://www.seeedstudio.com/act-4.html?utm_source=wiki&utm_medium=wikibanner&utm_campaign=newproducts" target="_blank"><img src="https://github.com/SeeedDocument/Wiki_Banner/raw/master/new_product.jpg" /></a></p>