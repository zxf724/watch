boards.txt: board config file.
stk500v2: source code


1. use wirst watch project 20190423\mcu\bootloader\boards.txt  file  replace C:\Program Files\Arduino\hardware\arduino\avr\boards.txt file.

2. use wirst watch project 20190423\mcu\bootloader\stk500v2 folder to replace C:\Program Files\Arduino\hardware\arduino\avr\bootloaders\stk500v2 folder

3. Install WinAVR-20100110-install.exe (Open Source command line compiler)

4. open cmd£¬into C:\Program Files\Arduino\hardware\arduino\avr\bootloaders\stk500v2£¬run make clean£¬than make could create the hex

5£ºtake the stk500boot_v2_mega2560.hex  file install in MCU, use ICSP interface compiler 

6. when download bootloader finished, fuse position need to reboot , use ICSP interface cpmpiler configure in MCU.

7. after that, MCU would start running in bootloader every time. LED would flicker 3 seconds, and than code will turn into app.


