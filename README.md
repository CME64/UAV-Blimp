# UAV-Blimp
---
_UAV Blimp controlled by Arduino Mega using web interface_


### Dependencies
* Arduino Mega
* Sensors and motors _(details included in the ino file comments)_
* Node.JS
* jQuery
* jQuery UI
* Flot.js
* serialport2 (Node.JS Library)
* socket.io.js
* Google Maps v3


### Steps:
1. Build the Arduino sketch and uploade it to the board (using arduino's application)
2. Connect the sensors and motors accordingly (refer to the code comments)
3. Install Node.JS
4. install the dependencies on Node.JS
4. Add the provided server & client files to Node.JS (NodeJS_Server.js & client.html)
5. Run the server

Notes:
* You can host it over a local/public network
* Controlling the motors using the web interface controls is incomplete but easy to implement
* When adding files to the server check the corresponding dependencies names and versions
