# Grove Starter Kit For Arduino

Grove Starter Kit for Arduino is one of the best Arduino Starter Kit for beginners. It includes one Arduino compatible Board and 10 additional Arduino sensors and all in one-piece of PCB design. **All the modules have been connected to the Seeeduino through the PCB stamp holes so no Grove cables are needed to connect**. Of course, you can also take the modules out and use Grove cables to connect the modules. You can build any Arduino project you like with this Grove Starter Kit.

## Hardware Overview

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200206211934.png)

**Note:** Dimensions - 17.69 * 1.64 * 1.88cm

1. **[Grove - LED](http://wiki.seeedstudio.com/Grove-Red_LED/):** Simple LED module
2. **[Grove - Buzzer](http://wiki.seeedstudio.com/Grove-Buzzer/):** Piezo Buzzer
3. **[Grove - OLED Display 0.96"](http://wiki.seeedstudio.com/Grove-OLED_Display_0.96inch/):** 128×64 dot resolution High brightness,self-emission and high contrast ratio Big screen on a compact design Low power consumption.
4. **[Grove - Button](http://wiki.seeedstudio.com/Grove-Button/):** Push button for human input interfaces
5. **[Grove - Rotary Potentiometer](http://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/):** Rotary knob for human input interfaces
6. **[Grove - Light](http://wiki.seeedstudio.com/Grove-Light_Sensor/):** Detects surrounding light intensity
7. **[Grove - Sound](http://wiki.seeedstudio.com/Grove-Sound_Sensor/):** Detects surrounding sound intensity
8. **[Grove - Temperature & Humidity Sensor](http://wiki.seeedstudio.com/Grove-TemperatureAndHumidity_Sensor/):** Detects surrounding temperature and humidity values
9. **[Grove - Barometer Sensor](http://wiki.seeedstudio.com/Grove-Barometer_Sensor-BMP280/):** Detects surrounding atmospheric pressure
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
|Light|Analog|A1|
|Sound|Analog|A2|
|Temperature & Humidity Sensor|Digital|D3|
|Barometer Sensor|I2C|I2C, 0x76(default)/0x77(optional)|
|3-Axis Accelerator|I2C|I2C, 0x4c(default)|

## Part List

|Modules|Quantity|
|---|---|
|**Sensors**||
|Temperature & Humidity Sensors|x1|
|3-Axis Accelerometers|x1|
|Barometer|x1|
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
|**Grove Cables**|x10|
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

It is used primarily to write code that initializes. The setup() function is called only once after the MCU is started.

```cpp
loop(){
}
```

A cyclic function. After the setup() function is completed, the MCU will call the loop() function, and the loop() function will be run again and again.

```cpp
int ledPin = 4;
```

Here, we assigned the variable name "ledPin" to pin 4, that is to say, ledPin stands for pin4 on the Seeeduino.

```cpp
pinMode(ledPin, OUTPUT);
```

Set ledPin to output mode.

```cpp
digitalWrite(ledPin, HIGH);
```

When we set the ledPin as output, HIGH means sending high level to the pin, LED turns on.

```cpp
digitalWrite(ledPin, LOW);
```

When we set the led as output, low stands for sending low level to the pin, LED turns off.

```cpp
delay(1000);
```

The number in parentheses represents the number of milliseconds of delay.1000 milliseconds is 1 second, which means the delay is 1 second.

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

This function is used to read the states of digital pins, either HIGH or LOW. When the button is pressed, the state is HIGH, otherwise is LOW.

```cpp
  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}
```

The usage of the statement is: if the logical expression in parentheses is true, execute the statement in curly braces after **if**, if not, execute the statement in curly braces after the **else**. If the state of the button is high, the LED pin outputs a high level and turn the LED  on, else turn LED off.

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
int ledPin = 6;      // select the pin for the LED
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

This function is used to read the value of Analog  pins, the range of values is: 0 ~ 1023.

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

Writes an analog value (PWM wave) to a pin. Can be used to light a LED at varying brightnesses or drive a motor at various speeds. Values can be from 0(always off) to 255(always on).

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
int sensorpin = A1; // Analog input pin that the sensor is attached to
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

The software running on the computer communicates with the development board, and the baud rate is 9600.

```cpp
Serial.println(sensorValue);
```

Serial port print the Light sensor’s value.  So you open the **serial monitor** on the IED interface, and you see the value of the output sensor.

```cpp
outputValue = map(sensorValue, 0, 1023, 0, 255);
```

Mapping light sensor analog signal(0 to 1023)to brightness value of LED(0 to 255).
Keep equal and potentiometer after mapping the value of the time. **Map** has five parameters, which in turn is: to map the original value, the original value of the minimum value, original value maximum, minimum value after the mapping, mapping the maximum. In this way, the data returned by the sensor can be mapped from its original value of 0-1023 to 0-255.

```cpp
analogWrite(ledPin,outputvalue);
```

The function is used to write an analog value between 0 - 255 a PWM pin. analogWrite() can only be used for PWM pins. The new mapping data in the previous statement can be output to ledPin to lighten / dim the LED.

- **Demo Effect and Serial Print Result:**

The LED module will change its intensity according to the light intensity of surrounding. The brighter the surrounding, the lighter it gets.

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
// if the sound sensor’s value is greater than 20, the light will be on for 5 seconds.
//Otherwise, the light will be turned off
if (soundState > 20) {
  digitalWrite(ledPin, HIGH);
  delay(5000);
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
**#include** is an instruction that introduces a header file. Here we use the DHT.h, <Arduino.h>, <U8g2lib.h>, <SPI.h>, <Wire.h> library, these library are included in Arduino IDE. 


```cpp
#define
```

Force a variable to be the value you want.

```cpp
U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
```

Once the object is declared, you can use functions from the library.

```cpp
u8x8.begin();
```

Initialize the library

```cpp
u8x8.setFlipMode(1);
```

Flips the display 180 degree.

```cpp
u8x8.setCursor();
```

Sets the draw cursor position.

```cpp
u8x8.setFont()
```

Set font set.

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

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Download the  [Seeed DHT library](https://github.com/Seeed-StudioGrove_Temperature_And_Humidity_Sensor)  from Github. Clink on **Sketch** > **Include library** > **Add .ZIP library**, import the library into the IDE.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

```Cpp
//Temperature and Humidity Sensor
//Temperature and Humidity Sensor
#include "DHT.h"
#include <Arduino.h>
#include <U8x8lib.h>

#define DHTPIN 4     // what pin we're connected to
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

Call these functions to read the temperature and humidity and store them in defined variables.

- **Demo Effect and Serial Print Result:**

The surrounding temperature and humidity appears on the OLED screen.

- **Breakout Guide**

Use Grove cable to connect the OLED to Seeeduino Lotus's **I2C** interface (Note: I2C's default address is 0x78). Connect the Grove Temperature and Humidity Sensor to Seeeduino Lotus's digital signal interface **D3**.

### 9. Barometer

tmp

### 10. 3-Axis Accelerometer

This is the last sensor, the triaxial accelerometer, and with this module, you can easily add motion monitoring to your design. So we can do a lot of interesting little experiments on the basis of motion.

- **Practice:** when motion is detected, the buzzer gives an alarm indicating that the object is in motion.

- **Components Involved:**
  1. Seeeduino Lotus
  2. Grove 3-axis Accelerometer
  3. Grove cable

- **Hardware connection:**
  - **Module connection:**
    - Default connection by PCB stamp hole.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Download the [3-Axis Digital Accelerometer(±1.5g)](https://github.com/Seeed-Studio/Accelerometer_MMA7660)  from Github. Click on **Sketch** > **Include library** > **Add .ZIP library**, import the library into the IED.
  - Copy the following code, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.
  - In this program, acceleration information are sent from the sensor to Seeeduino via I2C bus and then Seeeduino printed them onto the serial monitor. Open the **serial monitor** to check the result.

```Cpp
//Gravity Acceleration
#include <Wire.h>
#include "MMA7660.h"
MMA7660 accelemeter;
void setup()
{
  accelemeter.init();  
  Serial.begin(9600);
}
void loop()
{
  int8_t x;
  int8_t y;
  int8_t z;
  float ax,ay,az;
  accelemeter.getXYZ(&x,&y,&z);

  Serial.print("x = ");
  Serial.println(x); 
  Serial.print("y = ");
  Serial.println(y);   
  Serial.print("z = ");
  Serial.println(z);

  accelemeter.getAcceleration(&ax,&ay,&az);
  Serial.println("accleration of X/Y/Z: ");
  Serial.print(ax);
  Serial.println(" g");
  Serial.print(ay);
  Serial.println(" g");
  Serial.print(az);
  Serial.println(" g");
  Serial.println("*************");
  delay(1000);
}
```

- **Code analysis:**

```cpp
#include <Wire.h>
```

**#include** is an instruction that introduces a header file. Here we use the <Wire.h> library, this library is included in Arduino IDE. 

```cpp
#include "MMA7660.h"
```

Represents the MMA766O.h header file that introduces the current path.

```cpp
MMA7660 accelemeter;
```

  Once the object is declared, you can use functions from the library.

```cpp
accelemeter.getXYZ(&x,&y,&z);
```

It is a function from the library, call this function, and you get the raw data of x,y and z.

```cpp
accelemeter.getAcceleration(&ax,&ay,&az);
```

By calling this function, you get the triaxial acceleration information converted to the unit gravity "g".

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
int tonepin=6;
int ledp=5;

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
int lightPin = A1; //Analog light sensor is to be attached to analog
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

## More Learning

- [LSTM for live IoT data prediction](https://github.com/256ericpan/LSTM_IoT)

## Tech Support
Please submit any technical issue into our [forum](http://forum.seeedstudio.com/)<br /><p style="text-align:center"><a href="https://www.seeedstudio.com/act-4.html?utm_source=wiki&utm_medium=wikibanner&utm_campaign=newproducts" target="_blank"><img src="https://github.com/SeeedDocument/Wiki_Banner/raw/master/new_product.jpg" /></a></p>