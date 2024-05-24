# StarSat
StarSat was built as a group project for the FloatSat course at University Würzburg. The goal was to use a camera utilising ORPE (Optical Relative Pose Estimation) and an extentable arm to reliably dock with another rotating satellite. StarSat can be commanded and monitored witha groundstation over WiFi.

## Mission
StarSat will rotate at a constant rate and using the camera will search for the LED markers placed onto the Mockup satellite. Once the LED markers are identified it will slow down, once the mockups relative pose is tracked StarSat will point towards the mockup and extend the arm stopping shortly before docking. 
During this time StarSat will predict the correct time to do the final arm extension. Once docking, StarSat uses the camera to confirm dokcing was successfull and retry if failed. Once docked StarSat retracts the arm to complete the docking procedure.

## FloatSat
[FloatSat](https://www.informatik.uni-wuerzburg.de/aerospaceinfo/forschung-und-entwicklung-prof-dr-marco-schmidt-kopie-1/wissenschaft-forschung-deprecated/floatsat/) is a group project at University of Würzburg to build a working satellite model and complete a mission. The Satellites sit on in a bowl with an almost frictionless air cushion and a have one degree on freedom around the vertical rotation axis. This must be controlled using a reaction wheel and the satellite must have a datalink for telemetry and telecommands.

## ORPE
ORPE (Optical Relative Pose Estimation) was developed for use with cubesats. It uses low cost parts such as LEDs, Raspberry Pi camera module 2 and Raspberry Pi zero 2 (<100$) to estimate the relative positions and orientation of another object in the camera view at distances from 10 centimeters upto 15 meters at a rate 
of 10Hz with <3% accuracy and <1% precision.
ORPE is planned to be used on the Tamariw cubesat mission to test docking in LEO. (https://de.wikipedia.org/wiki/Tamariw)





