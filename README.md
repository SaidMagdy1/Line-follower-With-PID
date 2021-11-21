# PID controlled line follower arduino code #

the algorithm used is controlling position of robot by PID control (input is position and output is correctio) 

position is calculated by taking weighted signed average of IR sensors' readings 
as we have 4 sensors : - left sensors are positively weighted and right sensors are negatively wrighted so at set point average should be near to 0 
                       
   
                       - side sensors are weighted by the double of center sensors so if side sensors read black line the correction will be very high
                          to correct the position quickly

correction value is subtracted from right speed and added to the left speed (because of sensors' weight signs)

the configurable parameters are speed and PID constants 

maximum PWM after correction is 250 anyway so adjust speed and parameters to insure best response 

project report and code are attached
