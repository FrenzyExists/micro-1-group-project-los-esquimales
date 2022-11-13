---
Names: [Angel L Garcia, Samuel, Alex]
Tags: [Microprocessors, IoT, Esp32]
---

## Intro 

Hello There! These notes are more than just notes, these are the documentation on the architecture of our project *Is the A/C ON?* 

The general idea is that a user in their smartphone call out siri, cortana, google, bitsby, or any other service and say "is the air on?". The service then replies by asking: "At what classroom number? ", and you say which Stefani building classroom is and the service answers if the a/c is on or off along with how cold or warm it is and if its on it also gives the current temperature of the room.

This seemingly simple project uses Amazon's AWS services and a ESP32 that takes the temperature readings (hence why this is an IoT project).

The team "Los Esquimales" consists of Angel, Alex and Samuel (Goodman) are all computer engineering students that unanimously decided to choose this project and not the trolley one because we want to keep our sanity in check (nothing personal).

Project repo can be found [here](https://github.com/FrenzyExists/micro-1-group-project-los-esquimales)

To organize ourselves we used clickup originally, but because the app simply didn't suit our needs we switched to Trello. Trello link can be found [here]()

---

Table of Contents

# High Level Architecture

As said, this is an IoT project, so lets start with the high-level stuff. The way this works is that we use an ESP32 microcontroller by esspressif connected to a [] termistor. We wanted to also use a DHT22, but our professor said no. Since he's the manager, the manager's word is the law. Another constrain our professor gave was to not use the function `analogRead` and instead use the more low-level functions that analogRead utilized. In simple words, implement our own version of `analogRead`. Refer to the *Hardware Architecture* section to further know the details on how we are working around this constrain.

We place the device in an ESP32 over a classroom and using a keypad we asign the room number. Then we connect the ESP32 to the campus internet. This is so we can connect with a remote cloud computer that has an MQTT Broker protocol. More of that on *Cloud Architecture*, but for simplicity, we use it to retrieve data and send data in form of a RESTful API so our phones and computers can access said data. The RESTful nature allow us to access the data in JSON format, which is similar to a python dictionary. We can use this on our phones with a shortcut to access the data via some voice command service like Siri or Bitsbi.

## Tools
We used a couple of tools to get this done, for coding we're `VSCode` with the `PlatformIO` extension for embedded system development and the `C/C++` for syntax highlighting. For the implementation of the MQTT Broker we're using NodeRed as its easier to develop there due to its low-code nature. For testing we're using different college classrooms as well as our houses/apartments that could have an A/C around. To organize ourselves we're using Google Meets for quick reunions and Trello to see our tasks in both kanban boards and deadlines gantt charts to track of everything

Todo:
- Go back to Shutz crazy lectures and check about DOMAIN stuff
- That's it

# Cloud Architecture

Todo:
- Explain what a cloud architecture is
- Explain a bit about the MQTT Broker and what is
- Explain the planned GET, PUT, DELETE and UPDATE requests
- Explain how NodeRed is set up

# Hardware Architecture

Todo:

- Explain with code example how the AnalogRead constrain is dealt
- Talk about the technique the professor explained to work with temperature
- Explain in detail the components and show circuit diagram of planned design
- Talk a bit about what components could be replaced or added should one wanted to expand this project

