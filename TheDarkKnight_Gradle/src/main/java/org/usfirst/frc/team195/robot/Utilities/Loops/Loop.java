package org.usfirst.frc.team195.robot.Utilities.Loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope
 * calibration, etc.)
 */
public interface Loop {

    void onFirstStart(double timestamp);

    void onStart(double timestamp);

    void onLoop(double timestamp, boolean isAuto);

    void onStop(double timestamp);
}
