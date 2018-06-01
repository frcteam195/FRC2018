# The Dark Knight
Code for the 2018 FRC Season converted to use gradleRIO

After downloading this repo, use `./gradlew build` to download all dependencies and build the source

Then you can open the IntelliJ project and use all the internal gradle controls

# Feature Summary
* Robot diagnostic logging
* Robot self test mode
* Critical systems monitoring and real-time encoder and system fault detection
* Real-time communication diagnostics
* Autonomous odometry for use with path planning
* Path planning interface to generate auto paths (accurate to within two inches)
* Autonomous Path adapting for adapting to real-world field measurements as the robot drives, with path regeneration accounting for robot error
* 3 Cube autos with Scale vision to determine the height of the scale plate for placing cubes in autonomous
* TalonSRX and device wrappers (drivers) to extend features and simplify use
* --Added set called gain scheduling for fast switching PID profile slots
* Mobile diagnostic reporting to a phone app for quick checking systems
* Collision interference avoidance between elevator and arm subsystems
* PDP Breaker current modeling to roughly estimate the trip point of a breaker and limit current output before breakers trip
* LED Strip Controller for message display through a morse code translator