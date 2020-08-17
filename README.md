Made this beer balancing segway to impress with a "party trick" ;) It's not more usefull than that!

I used Arduino Nano to run the control system, MPU6050 gyroscope and two stepper motors. 
The plan was to use HC05 for bluetooth control, but since iPhone can't connect to that spesific hardware, I'm controlling it with WiFi (Blynk).
I made a PID class, that I use to control angle position (pitch), angle velocity (pitch and yaw) and linear velocity. 
The segway is controlled by changing the linear velocity and yaw angle velocity.

Really interesting and fun project!

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/URvtSx9A38s/0.jpg)](https://www.youtube.com/watch?v=URvtSx9A38s)

https://youtu.be/URvtSx9A38s
