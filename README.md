# Final-Project
A final project for the Logic Design Lab. <br><br>
【Useless Machine, Useful Application.】
-----------------------

Detailed information for this machine? <br>
--> See "Useless Machine, Useful Application.pptx"

----------------------
Things you need:
1. useless.bit  (bitfile)
2. Useless_Machine_Control.apk (Smartphone App)
3. Your own useless machine.
------------------------
Your own machine should connect with the following things correctly:
1. IR sensors should be put very close to the switch of the machine.
2. Ultrasound sensors should be put at both end of the machine. 
3. Motors for flipping arms and the remote-controlled car. 
4. Bluetooth module is needed if you want to control it with app. 
5. FPGA board (or something else that works.) 
6. Other things needed to make the machine works (battery... etc.)
------------------------
If you want to look into the code, things below are needed for compiled:
  1. Useless Machine.v  (Top Module)
  2. useless machine.xdc (Constraints File)
  3. lfsr_random.v
  4. bluetooth_control.v
  5. servo_control.v
  6. motor_control.v
  7. display_driver.v
  8. pwn_gen.v
  9. debounce onepulse.v
  10. sample_code/sonic.v 
  11. uart8/BaudRateGenerator.v
  12. uart8/Uart8Receiver.v
  13. uart8/Uart8Transmitter.v
  
Others are used for testing functions.
