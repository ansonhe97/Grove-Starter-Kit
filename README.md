# Grove Starter Kit For Arduino

Grove Starter Kit for Arduino is one of the best Arduino Starter Kit for beginners. It includes one Arduino compatible Board and 10 additional Arduino sensors and all in one-piece of PCB design. All the modules have been connected to the Seeeduino through the PCB stamp holes so no Grove cables are needed to connect. Of course, you can also take the modules out and use Grove cables to connect the modules. You can build any Arduino project you like with this Grove Starter Kit.

## Hardware Overview

![](https://raw.githubusercontent.com/ansonhe97/rawimages/master/img/20200206211934.png)

1. **[Grove - LED](http://wiki.seeedstudio.com/Grove-Red_LED/):** Simple LED module
2. **[Grove - Buzzer](http://wiki.seeedstudio.com/Grove-Buzzer/):** Piezo Buzzer
3. **[Grove - OLED Display 0.96"](http://wiki.seeedstudio.com/Grove-OLED_Display_0.96inch/):** 128×64 dot resolution High brightness,self-emission and high contrast ratio Big screen on a compact design Low power consumption.
4. **[Grove - Button](http://wiki.seeedstudio.com/Grove-Button/):** Push button for human input interfaces
5. **[Grove - Rotary Potentiometer](http://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/):** Rotary knob for human input interfaces
6. **[Grove - Light](http://wiki.seeedstudio.com/Grove-Light_Sensor/):** Detects surrounding light intensity
7. **[Grove - Sound](http://wiki.seeedstudio.com/Grove-Sound_Sensor/):** Detects surrounding sound intensity
8. **[Grove - Temperature & Humidity Sensor](http://wiki.seeedstudio.com/Grove-TemperatureAndHumidity_Sensor/):** Detects surrounding temperature and humidity values
9. **[Grove - Barometer Sensor](http://wiki.seeedstudio.com/Grove-Temperature_Humidity_Pressure_Gas_Sensor_BME680/):** Detects surrounding atmospheric pressure
10. **[Grove - 3-Axis Accelerator](http://wiki.seeedstudio.com/Grove-3-Axis_Digital_Accelerometer-1.5g/):** Detects object acceleration
11. **[Seeeduino Lotus](http://wiki.seeedstudio.com/Seeeduino_Lotus/):** Arduino Compatible Board with Grove Ports 

### Unlock Your First Open Source Hardware

Connect your Board to Power, it becomes a Smart indoor environmental data sensing prototype!

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
|OLED Display|x1|
|LED|x1|
|Buzzer|x1|
|**Output Modules**||
|Rotary Potentiometer|x1|
|Button|x1|
|Grove Cables|x10|
|Micro USB Cable|x1|

## Learning Objectives

- Basics of Open Source Hardware Systems.
- Basic Arduino Programming.
- Communication principles and methods for sensors.
- Hands-on implementation of Open Source Hardware projects.

## Arduino Background and Preparations

### What is Arduino

#### 1. The birth of Arduino

- Arduino is a kind of convenient, flexible and easy-to-use open source electronic platform, including hardware (Arduino board of various models) and software (Arduino IDE). It was designed by Italian teacher Massimo Banzi and Spain traditional engineer David Cuartielles. It solved the problem of the students can't find cheap microprocessors. Massimo Banzi liked to go to a bar called "di Re Arduino", this is according to the Italian king named after about one thousand years ago, to mark the place, they also name their board Arduino.

- It is not only suitable for rapid prototyping by engineers, but also for artists, designers, hobbyists and friends who are interested in interaction, and it is almost a must-have tool for modern makers!

#### 2. Introduction to Seeeduino and & Difference with Arduino

- **Seeeduino** is an Arduino-compatible board, designed and manufactured by Seeed Studio in Shenzhen, China. It is fully compatible with Arduino IDE, shields and other accessories.

- Based on an early Arduino design from Diecimila, it is available in two models with ATmega168 and ATmega328 microprocessors, compatible with the pin layout and dimensions of Diecimila. Improvements have been made in both auto-sensing USB and external power supply, as well as better power supply circuits on board, significantly improving flexibility and user experience.

### How to get started with Arduino IDE

#### Install the Arduino IDE

  - Arduino IDE is an integrated development environment for Arduino, which is used for single-chip microcomputer software programming, downloading, testing and so on.
  - Download [Arduino IDE](https://www.arduino.cc/en/Main/Software) for your desired operating system here.

#### Install the USB driver

  - Arduino connects to the PC via a USB cable. The USB driver depends on the type of USB chip you're using on your Arduino. *Note: USB chips are usually printed on the back of the development board.*

    - Download the [CP2102 USB Driver](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers). Note: Download according to your OS.
    - After the driver installation is completed, connect Arduino to the USB port of PC with USB cable. For Windows users, You can see it in `My Computer` -> `Properties` -> `Hardware` - `Device Management`. A `COM` will appear.
    - If the driver is not installed, or if the driver is installed incorrectly (not matching the chip model), it will appear as an "unknown device" in the device manager. At this point, the driver should be reinstalled.

#### Start the Arduino IDE

1. Open the Arduino IDE on your PC.
2. Click on `Tools` -> `Board` to select the correct Development Board Model. Select **Arduino/Genuino Uno** as Board.

![](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/board.png)

3. Click `Tools` -> `Port` to select the correct Port (the Serial Port shown in Device Manager in the previous step). In this case, `COM6` is selected.

![](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/port.png)

4. Create a new Arduino file `Hello.ino`, then copy the following code into it:

```Cpp
void setup() {
Serial.begin(9600); // initializes the serial port with a baud rate of 9600
}
void loop() {
Serial.println("hello, world"); // prints a string to a serial port
delay(1000); //delay of 1 second
}
```

5. In the upper left corner of the Arduino IDE, there are two buttons, Compile and Upload. First, press the compile button(Left) to compile. After the compilation is successful, press the upload button(Right).

![](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/func.png)

6. Navigate to `Tools` -> `Serial Monitor`, or click the Serial Monitor in the upper right corner, you can see the program running results

![](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/print.png)

## Building The First Open Source Control System

### 1. The minimum composition of control system

- Start with a simple example, the automatic door. When someone approaches it, the distance sensor senses it, sends a signal, the controller receives the signal, the signal is output to the actuator, the actuator then opens the door. So the control system is made up of three parts:

  - **Inputs:** Obviously, Modules like Temperature and Humidity sensors, 3-Axis Accelerator, Barometer Sensor, Light sensor, and Sound sensors can all act as Inputs, transmitting the perceived temperature, humidity, acceleration, or light signal data to the controller.
  
  - **Control:** the controller, the microprocessor, is the development board module, according to the input signal to issue instructions, so that the actuator to make the corresponding actions and response.

  - **Output:**  OLED Display, LED or buzzer can be used as an output module that converts electrical energy into sound, light or to OLED display screen. For example, it can display different patterns, which is its output.

- How does the controller know to open the door when it receives input signal from the sensor? This involves the new concept of "code"."Code" is the equivalent of "thought" in our brain. It can process the input signal so that the input system can communicate with the controller, and it can also process the output signal so that the controller can communicate with the output system.

- In general, the input unit, the control unit, and the output unit are the hardware, the body of our device, and the code is the software.

### 2. Analog, Digital and Protocol Signals

We already know that the input system, the controller and the output system communicate by processing the input and output signals in code. The signals here are equivalent to the "nerve impulses" of the brain. They are divided into digital signals and analog signals.

- **Analog signals:** Signals vary continuously in time and value, and the amplitude, frequency, or phase of the signal changes continuously at any time, such as the current broadcast sound signal, or image signal, etc.The analog signal has sine wave and triangle wave and so on.The analog pins of your microcontroller can have between 0V and 5V is mapped to a range between 0 and 1023 where 1023 is mapped as 5V and 512 is mapped as 2.5v and etc.

  ![Alt text](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/analog.png)

- **Digital signal:** Digital signal refers to the value of the amplitude is discrete, the amplitude is limited to a finite number of values. In our controller, the digital signal has two states: LOW(0V) for 0; HIGH(5V) for 1.

  ![Alt text](https://raw.githubusercontent.com/littletwany/PICTURE/master/img/digital.png)

- **Protocol signal:** the protocol signal we use is I2C, so here is a brief introduction to I2C. I2C bus just need two wires in the transmission of information connection between the devices: the SDA (Serial Data Line) and SCL (Serial Clock line). These two lines are bidirectional I/O lines, the main component used to start the bus transfer data, and generate the clock to open transmission device, any devices that are addressing at this time is considered from the device. The relationship between master and slave, sender and receiver on the bus is not constant, but depends on the direction of data transmission. If the host wants to send data to the slave device, the host first addresses the slave device, then actively sends data to the slave device, and finally terminates the data transmission by the host.If the host is to receive data from the slave, the slave is first addressed by the master. The host then receives the data sent from the device, and the host terminates the receiving process. In this case.The host is responsible for generating the timing clock and terminating the data transfer.

### 3. Seeeduino's interface distribution

- Take a look at the Seeeduino development board in the middle of the suite. The Seeeduino has ports for **Analog, Digital, I2C and UART Signals**.

- In addition, we have separate Grove Cables, and you can also use Grove Cables to connect modules to the corresponding interfaces on the Seeeduino development board.

## Section 1: Output Control

### 1. Turn on the LED Light

- **Project description:** how to light up the LED module

We have completed the output "Hello world" program.Now let's learn how to light the LED module. We know the three basic components of a control system: Input, Control, and Output. But lighting up LED uses only the output, not the input. Seeeduino is the control unit, LED module is the output unit and the output signal is digital signal.

- **List of materials:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Grove Cable(If Broken out)

- **Hardware connection:**
  - **Module connection**
    1. Default connection by PCB stamp hole.
    2. Use a Grove cable to connect the Grove LED to Seeeduino Lotus's digital interface **D4**.  
  - Connect the Seeeduino to the computer through the USB cable.
  - Make sure the cables are connected correctly, or you may damage the boards.
  - **Hardware Analysis:** The development board is the control unit and the LED module is the output unit.
  
- **Software code:**
  - Open Arduino IDE. 
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors. Verify that there are no errors, and you can upload the code.

```C
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

  - **set up()**<br>It is used primarily to write code that initializes. The setup() function is called only once after the MCU is started.
  - **loop()**<br>A cyclic function. After the setup() function is completed, the MCU will call the loop() function, and the loop() function will be run again and again.
  - **int ledPin = 4;**<br>Here, we assigned the variable name "ledPin" to pin 4, that is to say, ledPin stands for pin4 on the Seeeduino.
  - **pinMode(ledPin, OUTPUT);**<br>Set ledPin to output mode.
  - **digitalWrite(ledPin, HIGH);**<br>When we set the ledPin as output, HIGH means sending high level to the pin, LED turns on.
  - **digitalWrite(ledPin, LOW);**<br>When we set the led as output, low stands for sending low level to the pin, LED turns off.
  - **delay(1000);**<br>The number in parentheses represents the number of milliseconds of delay.1000 milliseconds is 1 second, which means the delay is 1 second.

- **Practice**: Adjust how quickly led flashing of lights.
  - Functional: By modifying the software code to adjust LED module to  flicker quicker.
  - The hardware connection and the analysis of hardware are the same as the above.
  - Try adjusting the Numbers in brackets to change the frequency at which the bulb flashes. Such as "delay(500);",turn the LED on for 500ms and off for 500ms.

## Section 2: Input Control

By now, we should have a great understanding of both the control module and the output module. In this section, we will add the input module to understand the controlling of the output. Doesn't it sound very interesting? Try it now!

### 1. Button

The first thing we need to know is that the input of the button is a digital signal, and there are only two states, 0 or 1, so we can control the output based on those two states.

- **Practice:** Use button to turn ON and OFF the LED module

- **List of materials:**
    1. Seeeduino Lotus
    2. Grove LED
    3. Grove Button
    4. Grove Cables(If broken out)

- **Hardware connection:**
  - **Module connection:**
    1. Default connection by PCB stamp hole.
    2. Use a Grove cable to connect the Grove LED to Seeeduino Lotus's digital interface **D4**. Connect the Grove Button to digital interface **D6**.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Hardware analysis**:
  - Input: Button
  - Control: Seeeduino
  - Output: LED module

Both the sensor and the LED use digital signals, so they should be connected to digital interfaces.

- **Software code**:
  - Open Arduino IDE.
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

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

  - **pinMode(ledPin, OUTPUT);**

    Define LED as the output unit.

  - **pinMode(buttonPin, INPUT);**
  
    Define button as the input unit.

  - **buttonState = digitalRead(buttonPin);**

    This function is used to read the states of digital pins, either HIGH or LOW. When the button is pressed, the state is HIGH, otherwise is lOW.

  - **if (buttonState == HIGH) {**<br>  **digitalWrite(ledPin, HIGH);**<br>**} else {**<br>**digitalWrite(ledPin, LOW);**  <br> **}**<br>The usage of the statement is: if the logical expression in parentheses is true, execute the statement in curly braces after **if**, if not, execute the statement in curly braces after the **else**.  
  If the state of the button is high, the LED pin outputs a high level and turn the LED  on, else turn LED off.

### 2. Rotary Potentiometer

- **Practice:** Turn the knob to control the frequency of LED flashing.

Unlike buttons, the signals entered by Potentiometer are analog signals. Unlike digital signals, analog signals range in value from 0 to 1023. We can control the frequency of led flashing based on this signal.

- **List of materials:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Grove Rotary Switch
  4. Grove Cables(If broken out)

- **Hardware connection:**
  - **Module connection:**
    1. Default connection by PCB stamp hole.
    2. Use a Grove cable to connect LED to Seeeduino Lotus's digital interface **D4**, and a Grove cable to connect the Grove Rotary Switch to analog signal interface **A0**.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Hardware analysis:**

  - Input: Rotary Potentiometer
  - Control: Seeeduino Lotus
  - Output: LED module

The input is an analog signal, so it is connected to the analog signal interface, the LED module is connected to the digital signal interface.

- **Software code:**
  - Open Arduino IDE.
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

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

  - **rotaryValue = analogRead(rotaryPin);**

     This function is used to read the value of Analog  pins, the range of values is: 0 ~ 1023.

  - **delay(rotaryValue);**

     Realize delay function, The millisecond duration of the delay is the value in parentheses. Because the value is the value of the analog signal of the knob pin being read, so the delay time can be controlled by the knob.

## Section 3: Functional Sensors

In the previous section, we have used buttons and rotary potentiometer as control modules, but there are still many sensors that have not been used, so let's take a look at each one.

### 1. Light Sensor

The light sensor contains a photosensitive resistor to measure the intensity of light. The resistance of the photosensitive resistor decreases with the increase of light intensity. The output signal is the analog value, the brighter the light source, the larger the analog value. Based on this property, you can use it to make a light switch.

- **Exercise:** As the environment slowly brightens, the LED lights will lighten. As the light slowly dimmed, the LED dimmed. The LED will go from dark to light or from light to dark. To achieve this, we will use pulse width modulation(PWM).
  
- **List of materials:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Grove Light Sensor
  4. Grove Cable(If broken out)

- **Hardware connection:**
  - **Module connection:**
    1. Default connection by PCB stamp hole.
    2. Use Grove Cable to connect the Grove LED to Seeeduino Lotus's digital signal interface **D4**,connect the Grove Light Sensor to Seeeduino Lotus's analog signal interface **A1**.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Hardware analysis:**

  - Input: Light Sensor
  - Control: Seeeduino Lotus
  - Output: LED module

- **Software code:**
  - Open Arduino IDE.
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

   ```Cpp
   // Light Switch
   int sensorpin = A1; // Analog input pin that the sensor is attached to
   int ledPin = 4;    // LED port
   int sensorValue = 0;        // value read from the port
   int outputValue = 0;        // value output to the PWM (analog out)

   void setup() {
   pinMode(ledPin,OUTPUT);
   }

   void loop() {
   // read the analog in value:
   sensorValue = analogRead(sensorpin);
   // map it to the range of the analog out:
   outputValue = map(sensorValue, 0, 1023, 0, 255);
   // change the analog out value:
   analogWrite(ledPin, outputValue);
   delay(30);
   }
   ```

- **Code Analysis:**

  - **PWM(Pulse Width Modulation)**

    - It is based on a series of Pulse Width Modulation, the equivalent of the required waveform (including shape and amplitude), the analog signal level for digital encoding, that is to say, by adjusting the duty ratio of the change to the change of the signal to adjust, and energy, duty cycle is to point to in a cycle, the signal at a high level of the percentage of the time to occupy the whole signal cycle, for example, the square wave duty ratio is 50%.
    - There are six digital pins on your Seeeduino that are marked with the symbol “~”, which means they can send out a PWM signal : 3,5,6,9,10,11. They are celled PWM pins.

  - **outputValue = map(sensorValue, 0, 1023, 0, 255);**</br>
    Mapping light sensor analog signal(0 to 1023)to brightness value of LED(0 to 255).

  - **outputValue**</br>
     keep equal and potentiometer after mapping the value of the time. **map** has five parameters, which in turn is: to map the original value, the original value of the minimum value, original value maximum, minimum value after the mapping, mapping the maximum. In this way, the data returned by the sensor can be mapped from its original value of 0-1023 to 0-255.

  - **analogWrite(ledPin,outputvalue)**</br>
     The function is used to write an analog value between 0 - 255 a PWM pin. analogWrite() can only be used for PWM pins. The new mapping data in the previous statement can be output to ledPin to lighten / dim the LED.

### 2. Sound Sensor

The sound sensor can detect the sound intensity of the environment, and its output is also simulated. I'm sure you've all been exposed to the sound control lights, but now we can do one ourselves, and with the basics, this experiment will be easy for you.

- **Practice:** The LED lights light up when the sound is made. When there is no sound and it is very quiet, the LED lights go off.

- **List of materials:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Grove Sound Sensor
  4. Grove cable(If broken out)

- **Hardware connection:**
  - **Module connection:**
    1. Default connection by PCB stamp hole.
    2. Use Grove cables to connect the Grove LED to Seeeduino Lotus's digital signal interface **D4**, Connect the Grove Sound Sensor to Seeeduino Lotus's analog signal interface **A2**.
    - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors. Verify that there are no errors, and you can upload the code.

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

- **Code analysis:**

  - **Serial.begin(9600);**</br>
   The software running on the computer communicates with the development board, and the baud rate is 9600.

  - **Serial.print(" ");**</br>
   This function is used to output data from the serial port, the output is what is contained in the double quotation marks.

  - **Serial.println( );**</br>
   This statement is similar to the one above, except that **serial.println** has a newline return.
  - **Serial.println(soundState);**</br>
   Serial port print the sound sensor’s value.  So you open the **serial monitor** on the IED interface, and you see the value of the output sensor.

### 3. Temperature and Humidity Sensor

Have you ever wondered about the temperature and humidity of your surroundings? Want to know the exact number? Want to wear a skirt or coat today depending on the temperature?Let's make a temperature meter!

- **Practice:** Let your OLED display display the current ambient temperature and humidity.
- **List of materials:**
  1. Seeeduino Lotus
  2. Grove OLED
  3. Grove Temperature and Temperature Sensor
  4. Grove cable(If broken out)

- **Hardware connection:**
  - **Module connection:**
    1. Default connection by PCB stamp hole.
    2. Use Grove cable to connect the OLED to Seeeduino Lotus's **I2C** interface (note: I2C's default address is 0x78). Connect the Grove Temperature and Humidity Sensor to Seeeduino Lotus's digital signal interface **D3**.
    - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Download the  [Seeed DHT library](https://github.com/Seeed-StudioGrove_Temperature_And_Humidity_Sensor)  from Github. Clink on **Sketch** > **Include library** > **Add .ZIP library**, import the library into the IDE.
  - Install the **U8g2 library**: Search for the keyword "U8g2" in the Library Manager, then install.
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

   ```Cpp
   //Temperature and Humidity Sensor
   #include "DHT.h"
   #include <Arduino.h>
   #include <U8g2lib.h>
   #include <SPI.h>
   #include <Wire.h>

   #ifdef U8X8_HAVE_HW_SPI
   #endif
   #ifdef U8X8_HAVE_HW_I2C
   #endif
   #define DHTPIN 3     // what pin we're connected to
   #define DHTTYPE DHT11   // DHT 11 

   DHT dht(DHTPIN, DHTTYPE);

   U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

   void setup(void) {
  
    Serial.begin(9600); 
    Serial.println("DHTxx test!");
    Wire.begin();

    dht.begin();
    u8g2.begin();  
   }

   void loop(void) {

   float temp_hum_val[2] = {0};
   dht.readTempAndHumidity(temp_hum_val);

   u8g2.firstPage();
   {

    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0,50);
    u8g2.print("Humidity:");
    u8g2.print(temp_hum_val[0]);
    u8g2.print("%");
    u8g2.setCursor(0, 33);
    u8g2.print("Temperature:");
    u8g2.print(temp_hum_val[1]);
    u8g2.print("*C");

   }
   while (u8g2.nextPage());
    delay(1000);  
   }
   ```

- **Code analysis:**
  
  - **#include <>**</br>
  **#include** is an instruction that introduces a header file.
  Here we use the DHT.h, <Arduino.h>, <U8g2lib.h>, <SPI.h>, <Wire.h> library, these library are included in Arduino IDE. 

  - **#ifdef instruction**</br>
  If the preprocessor has already defined the following identifier, execute all instructions and compile the C code until the next #else or #endif appears (regardless of who appears first).

  - **#define**</br>
  Force a variable to be the value you want.

  - **DHT dht(DHTPIN, DHTTYPE);**</br>
  **U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /\*reset=*\/ U8X8_PIN_NONE);**</br>
  Once the object is declared, you can use functions from the library.

  - **float temp_hum_val[2] = {0};**</br>
  Defines an array of floating-point types. Floating point Numbers use exponents to allow the position of the decimal point to rise and fall as needed.  An array is a large number of combinations, of the same type, that are stored sequentially in memory.

  - **dht.readTempAndHumidity(temp_hum_val);**</br>
  Call this function to read the temperature and humidity.

  - **u8g2.firstPage();**

  - **while (u8g2.nextPage());**</br>
  Will change the current page position to 0. Modifying content is between firstPage and nextPage, rerendering everything each time.

  - **setCursor()** </br>
  Sets the draw cursor position.

  - **u8g2.setFont()**</br>
  Set font set.

  - **u8g2.print()**</br>
  Draw the content on the OLED.

### 4. Barometer

tmp

### 5. 3-Axis Accelerometer

This is the last sensor, the triaxial accelerometer, and with this module, you can easily add motion monitoring to your design. So we can do a lot of interesting little experiments on the basis of motion.

- **Practice:** when motion is detected, the buzzer gives an alarm indicating that the object is in motion.

- **List of materials:**
  1. Seeeduino Lotus
  2. Grove 3-axis Accelerometer
  3. Grove cable

- **Hardware connection:**
  - **Module connection:**
     1. Default connection by PCB stamp hole.
     2. Use Grove cable to connect Grove 3-axis Accelerometer to Seeeduino Lotus's **I2C** interface using a Grove cable (note: I2C default address is 0x4c).
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - Download the [3-Axis Digital Accelerometer(±1.5g)](https://github.com/Seeed-Studio/Accelerometer_MMA7660)  from Github. Click on **Sketch** > **Include library** > **Add .ZIP library**, import the library into the IED.
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.
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
  - **#include <Wire.h>**</br>
   **#include** is an instruction that introduces a header file.
   Here we use the <Wire.h> library, this library is included in Arduino IDE. 

  - **#include "MMA7660.h"**</br>
   Represents the MMA766O.h header file that introduces the current path.

  - **MMA7660 accelemeter;**</br>
   Once the object is declared, you can use functions from the library.

  - **accelemeter.getXYZ(&x,&y,&z);**</br>
   It is a function from the library, call this function, and you get the raw data of x,y and z.

  - **accelemeter.getAcceleration(&ax,&ay,&az);**</br>
   By calling this function, you get the triaxial acceleration information converted to the unit gravity "g".

## Section 4: Colorful Experimental Projects

### 1. Music dynamic rhythm lamp

- **Project description:** In this experiment, we will make the buzzer play pleasant music and the led lights flash according to the music frequency and beat.

- **List of materials:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Buzzer
  4. Grove Cables(if broken out)

- **Hardware connection:**
  - **Module connection:**
    1. Default connection by PCB stamp hole.
    2. Connect Grove LED to Seeeduino Lotus's digital signal interface **D4**, connect Buzzer to Seeeduino Lotus's digital signal interface **D5**.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

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
  
  - **#define NTD**</br>
    Here is the definition of the frequency of the D key, which is divided into bass, alto, and treble.

  - **#define WHOLE 1**
  - **#define HALF 0.5**
  - **#define QUARTER 0.25**
  - **#define EIGHTH 0.25**
  - **#define SIXTEENTH 0.625**</br>
   Note rhythm is divided into one beat, half beat, 1/4 beat, 1/8 beat, we specify a beat note time is 1;Half beat is 0.5;1/4 beat is 0.25;1/8 of 0.125.

  - **int tune[]=**</br>
   List the frequencies according to the spectrum.

  - **float durt[]=**</br>
   List the beats according to the spectrum.

  - **delay(100*durt[x]);**</br>
   Control LED lights on and off respectively.

### 2. Make an intelligent sound-light induction desk lamp

- **Project description:** as the name implies, this project is to make a small lamp controlled by Sound and Light. We need to use LED module. Of course, Light Sensor and Sound Sensor are also indispensable. In this way, you can achieve the function of the smart desk lamp: when the sound, the lamp will light up;If the environment turns dark, the lamp will automatically turn brighter.

- **list of materials:**
  1. Seeeduino Lotus
  2. Grove LED
  3. Light Sensor
  4. Sound Sensor
  5. Grove cable(If broken out)

- **Hardware connection:**
  - **Module connection:**
    1. Default connection by PCB stamp hole.
    2. Connect the Grove LED to Seeeduino Lotus's digital signal interface **D4**, Connect the Light Sensor to Seeeduino Lotus's analog signal interface **A1**. Connect the Sound Sensor to Seeeduino Lotus's analog signal interface **A2** using a Grove cable.
  - The Seeeduino is then connected to the computer via a USB cable.

- **Software code:**
  - Open Arduino IDE.
  - You can copy the code directly to Arduino IDE. Although we encourage you to type the code out by hand to develop your skills.
  - When you're done, click Verify to check for syntax errors.Verify that there are no errors, and you can upload the code.

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

  - **if (soundState > 50 || lightState < 10) { }**

    In parentheses is a logical expression.

    Both **&&** and **||** are commonly used in logical expressions. The common usage is **if (expression 1 || expression 2)** and **if (expression 1 && expression 2)**.

    **||** represents "**or**", satisfies one of them, the whole expression is true, and satisfies the condition of the if judgment.

    **&&** means "**and**", the statement in if{} is executed only if all expressions in parentheses are true.