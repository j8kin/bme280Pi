# BME 280 driver for Raspberry Pi
## Preconditions
1. i2c should be enabled on RaspberryPi: https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all
2. Install i2c-tools: `sudo apt-get install -y i2c-tools`
3. Java 11 or above should be installed on Raspberry Pi: `sudo apt install default-jdk`
4. Maven 3.6.3 or above need to be installed on Raspberry Pi: `sudo apt-get install maven`
5. Unfortunatly default JDK is not support i2c-dio and custom JDK need to be installed: https://bell-sw.com/pages/downloads-embedded/#/java-11-lts (download full SDK and install deb-package manually)
## Setup Visual Studio Code to work via SSH
It is really convinient to install Raspbian Lite without GUI on Raspberry Pi (it use less resources of it) and use Visual Studio Code from regular PC (for example under Windows) to connect to Raspberry Pi via SSH. In this case Raspberry Pi is used as DEV-machine and code exeuted on it but modified on desctop station. Also it is possible to debug develoiped application directly from Visual Studio Code even if the code executed on Raspberry Pi.
Visual Studio Code via SSH: https://code.visualstudio.com/docs/remote/ssh
1. Set static ip for your Raspberry Pi on you home router.
2. Install Visual Studio Code on your desctop: https://code.visualstudio.com/download
3. Install Visual Studio Code Remote Development Extension Pack: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack
4. Install Java Extention Pack on Remote-SSH (Raspberry-Pi)
## Open this project on Remote-SSH from GitHub
