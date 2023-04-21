//How to Correctly trun on the setup...............

1. Turn on the Elmo and MOVO2 Drives.
2. Change analog output parameters of Elmo drive to 1 : 0.2 A and rest the offset.
3. Get the code and run it once without running any motor.
4. Put Elmo drive to Run mode.
5. Type MASK->SVON->G to turn on movo driven motors.
6. Get the code "s826_example_Working_surface_reproduction.c" to "s826_example".
7. Run the motor in constant force mode.
8. Run the motor in position control mode with position as 0.
9. Run the motor in Position tracking mode with the .csv data.
10. If all above tests are completed sucessfully then get the code "s826_example_final_code_PR2.c".
11. Place the tip very close to the surface brfore running.
12. Run the code.
 
