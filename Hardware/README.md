# Hardware
All of the files given are the latest files and no previous versions are provided. That is mainly because the previous versions do not provide any value anymore.  
## PCB
It is important to note that the PCB contains a chip for regulating battery voltage to 5V,h however this chip seems to only work for about ~60 minutes and then causes a short circuit, killing itself and yhe battery. 
It is recommended to either use external power bank via the USB-C connector or you can redesign the PCB to have proper battery management.  
## ESP32 Code
The current code contains numerous functions that can be commented or uncommetned, depending on the desired data output. It is recommended to have a quick look over the main loop() function and its comments.
## CAD
The CAD files are given as STL and Siemens NX. A lot of the components are attached to the glove using velcro and velcro straps.
