package org.usfirst.frc.team195.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.CustomSubsystem;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;

import java.util.List;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator extends Thread implements CustomSubsystem {
    private static final int MIN_ROBOT_ESTIMATOR_LOOP_TIME = 5;
    static RobotStateEstimator instance = null;
    boolean runThread = false;
    PathFollowerRobotState robot_state_ = PathFollowerRobotState.getInstance();
    DriveBaseSubsystem drive_ = DriveBaseSubsystem.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;
    private DriverStation ds;
    private ThreadRateControl threadRateControl = new ThreadRateControl();

    RobotStateEstimator() {
        ds = DriverStation.getInstance();
    }

    public static RobotStateEstimator getInstance() {
        if(instance == null) {
            try {
                instance = new RobotStateEstimator();
            } catch (Exception ex) {
                ConsoleReporter.report(ex, MessageLevel.DEFCON1);
            }
        }

        return instance;
    }

    public static RobotStateEstimator getInstance(List<CustomSubsystem> subsystemList) {
        subsystemList.add(getInstance());
        return instance;
    }

    @Override
    public void init() {

    }

    @Override
    public void subsystemHome() {
        left_encoder_prev_distance_ = drive_.getLeftDistanceInches();
        right_encoder_prev_distance_ = drive_.getRightDistanceInches();
    }

    @Override
    public void start() {
        runThread = true;
        if (!super.isAlive())
            super.start();
    }

    @Override
    public void terminate() {
        ConsoleReporter.report("CAN'T STOP, WON'T STOP, DON'T CALL ME!", MessageLevel.ERROR);
//        runThread = false;
//        try {
//            super.join(Constants.kThreadJoinTimeout);
//        } catch (Exception ex) {
//            ConsoleReporter.report(ex);
//        }
    }

    @Override
    public void run() {
        while (!ds.isEnabled()) {try{Thread.sleep(20);}catch(Exception ex) {}}
        subsystemHome();
        threadRateControl.start();

        while (runThread) {
            final double left_distance = drive_.getLeftDistanceInches();
            final double right_distance = drive_.getRightDistanceInches();
            final Rotation2d gyro_angle = drive_.getGyroAngle();
            final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                    left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
            final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftVelocityInchesPerSec(),
                    drive_.getRightVelocityInchesPerSec());
            robot_state_.addObservations(Timer.getFPGATimestamp(), odometry_velocity, predicted_velocity);
            left_encoder_prev_distance_ = left_distance;
            right_encoder_prev_distance_ = right_distance;

            threadRateControl.doRateControl(MIN_ROBOT_ESTIMATOR_LOOP_TIME);
        }
    }

}
