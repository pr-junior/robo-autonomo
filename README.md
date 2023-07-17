# robo-autonomus

Project of the Embedded Systems course in the Electronics Engineering program at the Federal Technological University of Paraná.

This robot is capable of identifying obstacles and circumventing them until it reaches its destination. The obstacles can be placed dynamically, and the goal can be anywhere.

During the internal competition among the students, this robot was the winner, managing to reach the destination in just 6 seconds.

An STM32 microcontroller, ultrasonic and infrared sensors, and encoders were used in the implementation. Using STM32's FreeRTOS, tasks were created to handle sensor readings, and a state machine was developed to make decisions based on the sensor information.
