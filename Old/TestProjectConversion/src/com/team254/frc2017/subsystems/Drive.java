package com.team254.frc2017.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

import com.team254.frc2017.Constants;
import com.team254.frc2017.Kinematics;
import com.team254.frc2017.RobotState;
import com.team254.frc2017.ShooterAimingParameters;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import com.team254.lib.util.control.Lookahead;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.control.PathFollower;
import com.team254.lib.util.drivers.CANTalonFactory;
import com.team254.lib.util.drivers.NavX;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Twist2d;

import java.util.Arrays;
import java.util.Optional;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons, one solenoid and 2 pistons to shift gears,
 * and a navX board. The Drive subsystem has several control methods including open loop, velocity control, and position
 * control. The Drive subsystem also has several methods that handle automatic aiming, autonomous path driving, and
 * manual control.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
        PATH_FOLLOWING, // used for autonomous driving
        AIM_TO_GOAL, // turn to face the boiler
        TURN_TO_HEADING, // turn in place
        DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
        DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
    }

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state) {
        if (state == DriveControlState.AIM_TO_GOAL ||
                state == DriveControlState.TURN_TO_HEADING ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH) {
            return true;
        }
        return false;
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster;//, mLeftSlave, mRightSlave, mLeftSlave2, mRightSlave2;
    private final VictorSPX mLeftSlave, mRightSlave, mLeftSlave2, mRightSlave2;
    private final DoubleSolenoid mShifter;
    private final NavX mNavXBoard;

    // Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;

    // Hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;

    // Logging
    private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
                setVelocitySetpoint(0, 0);
                mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
                    return;
                case VELOCITY_SETPOINT:
                    return;
                case PATH_FOLLOWING:
                    if (mPathFollower != null) {
                        updatePathFollower(timestamp);
                        mCSVWriter.add(mPathFollower.getDebug());
                    }
                    return;
                case AIM_TO_GOAL:
//                    if (!Superstructure.getInstance().isShooting()) {
//                        updateGoalHeading(timestamp);
//                    }
                    // fallthrough intended
                case TURN_TO_HEADING:
                    updateTurnToHeading(timestamp);
                    return;
                case DRIVE_TOWARDS_GOAL_COARSE_ALIGN:
                    updateDriveTowardsGoalCoarseAlign(timestamp);
                    return;
                case DRIVE_TOWARDS_GOAL_APPROACH:
                    updateDriveTowardsGoalApproach(timestamp);
                    return;
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
    };

    private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = CANTalonFactory.createDefaultTalonSRX(Constants.kLeftDriveMasterId);
        mLeftMaster.set(ControlMode.PercentOutput, 0);
        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kLowGearPositionControlSlot, Constants.kTimeoutMs);
        mLeftMaster.setSensorPhase(true);
        mLeftMaster.setInverted(false);

        boolean leftSensorPresent = mLeftMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
        if (!leftSensorPresent) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }

        mLeftSlave = CANTalonFactory.createPermanentVictorSlaveToTalonSRX(Constants.kLeftDriveSlaveId, mLeftMaster);
        mLeftSlave.setInverted(false);
        mLeftSlave2 = CANTalonFactory.createPermanentVictorSlaveToTalonSRX(Constants.kLeftDriveSlaveId2, mLeftMaster);
        mLeftSlave2.setInverted(false);
        mLeftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

        mRightMaster = CANTalonFactory.createDefaultTalonSRX(Constants.kRightDriveMasterId);
        mRightMaster.set(ControlMode.PercentOutput, 0);
        mRightMaster.setSensorPhase(true);
        mRightMaster.setInverted(true);
        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kLowGearPositionControlSlot, Constants.kTimeoutMs);

        boolean rightSensorPresent = mRightMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
        if (!rightSensorPresent) {
            DriverStation.reportError("Could not detect right encoder: " + rightSensorPresent, false);
        }

        mRightSlave = CANTalonFactory.createPermanentVictorSlaveToTalonSRX(Constants.kRightDriverSlaveId, mRightMaster);
        mRightSlave.setInverted(true);
        mRightSlave2 = CANTalonFactory.createPermanentVictorSlaveToTalonSRX(Constants.kRightDriverSlaveId2, mRightMaster);
        mRightSlave2.setInverted(true);
        mRightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

        mLeftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs);
        mLeftMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs);
        mRightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs);
        mRightMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs);

        mShifter = new DoubleSolenoid(Constants.kShifterSolenoidId1, Constants.kShifterSolenoidId2);

        reloadGains();

        mIsHighGear = false;
        setHighGear(true);
        setOpenLoop(DriveSignal.NEUTRAL);

        // Path Following stuff
        mNavXBoard = new NavX(SPI.Port.kMXP);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv",
                PathFollower.DebugOutput.class);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, 0);
            mRightMaster.set(ControlMode.PercentOutput, 0);
//            mLeftMaster.configNominalOutputVoltage(0.0, 0.0);
//            mRightMaster.configNominalOutputVoltage(0.0, 0.0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }

        mRightMaster.set(mRightMaster.getControlMode(), signal.getRight());
        mLeftMaster.set(mLeftMaster.getControlMode(), signal.getLeft());
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            //mShifter.set(wantsHighGear ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
            mShifter.set(wantsHighGear ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            mRightMaster.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
            mRightSlave.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
            mRightSlave2.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
            mLeftMaster.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
            mLeftSlave.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
            mLeftSlave2.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
//        final double left_speed = getLeftVelocityInchesPerSec();
//        final double right_speed = getRightVelocityInchesPerSec();
//        SmartDashboard.putNumber("left voltage (V)", mLeftMaster.getOutputVoltage());
//        SmartDashboard.putNumber("right voltage (V)", mRightMaster.getOutputVoltage());
//        SmartDashboard.putNumber("left speed (ips)", left_speed);
//        SmartDashboard.putNumber("right speed (ips)", right_speed);
//        if (usesTalonVelocityControl(mDriveControlState)) {
//            SmartDashboard.putNumber("left speed error (ips)",
//                    rpmToInchesPerSecond(mLeftMaster.getSetpoint()) - left_speed);
//            SmartDashboard.putNumber("right speed error (ips)",
//                    rpmToInchesPerSecond(mRightMaster.getSetpoint()) - right_speed);
//        } else {
//            SmartDashboard.putNumber("left speed error (ips)", 0.0);
//            SmartDashboard.putNumber("right speed error (ips)", 0.0);
//        }
//        synchronized (this) {
//            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
//                SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
//                SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
//            } else {
//                SmartDashboard.putNumber("drive CTE", 0.0);
//                SmartDashboard.putNumber("drive ATE", 0.0);
//            }
//        }
//        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getPosition());
//        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getPosition());
//        SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
//        SmartDashboard.putNumber("gyro pos", getGyroAngle().getDegrees());
//        SmartDashboard.putBoolean("drive on target", isOnTarget());
    }

    public synchronized void resetEncoders() {
        mLeftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
        mRightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
        //mLeftSlave.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
        //mRightSlave.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
        mNavXBoard.zeroYaw();
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(mDriveControlState)) {
            // We entered a velocity control state.
            mLeftMaster.set(ControlMode.Velocity, 0);
            mLeftMaster.selectProfileSlot(0, kHighGearVelocityControlSlot);

            mRightMaster.set(ControlMode.Velocity, 0);
            mRightMaster.selectProfileSlot(0, kHighGearVelocityControlSlot);
            setBrakeMode(true);
        }
    }

    /**
     * Configures talons for position control
     */
    private void configureTalonsForPositionControl() {
        if (!usesTalonPositionControl(mDriveControlState)) {
            // We entered a position control state.
            mLeftMaster.selectProfileSlot(0, kLowGearPositionControlSlot);
            mLeftMaster.set(ControlMode.MotionMagic, mLeftMaster.getSelectedSensorPosition(kLowGearPositionControlSlot));

            mRightMaster.selectProfileSlot(0, kLowGearPositionControlSlot);
            mRightMaster.set(ControlMode.MotionMagic, mRightMaster.getSelectedSensorPosition(kLowGearPositionControlSlot));
            setBrakeMode(true);
        }
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(mDriveControlState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            mLeftMaster.set(mLeftMaster.getControlMode(), Util.convertRPMToNativeUnits(inchesPerSecondToRpm(left_inches_per_sec * scale)));
            mRightMaster.set(mRightMaster.getControlMode(), Util.convertRPMToNativeUnits(inchesPerSecondToRpm(right_inches_per_sec * scale)));
            //System.out.println("Requested Drive Velocity Left/Right: " + left_inches_per_sec + "/" + right_inches_per_sec);
            //System.out.println("Actual Drive Velocity Left/Right: " + getLeftVelocityInchesPerSec() + "/" + getRightVelocityInchesPerSec());
        } else {
            System.out.println("Hit a bad velocity control state");
            mLeftMaster.set(mLeftMaster.getControlMode(), 0);
            mRightMaster.set(mRightMaster.getControlMode(), 0);
        }
    }

    /**
     * Adjust position setpoint (if already in position mode)
     * 
     * @param left_position_inches
     * @param right_position_inches
     */
    private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
        if (usesTalonPositionControl(mDriveControlState)) {
            mLeftMaster.set(mLeftMaster.getControlMode(), inchesToRotations(left_position_inches) * 4096);
            mRightMaster.set(mRightMaster.getControlMode(), inchesToRotations(right_position_inches) * 4096);
        } else {
            System.out.println("Hit a bad position control state");
            mLeftMaster.set(mLeftMaster.getControlMode(), 0);
            mRightMaster.set(mRightMaster.getControlMode(), 0);
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(mLeftMaster.getSelectedSensorPosition(0)/4096);
    }

    public double getRightDistanceInches() {
        return rotationsToInches(mRightMaster.getSelectedSensorPosition(0)/4096);
    }

    public double getLeftVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0))); }

    public double getRightVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0))); }

    public synchronized Rotation2d getGyroAngle() {
        return mNavXBoard.getYaw();
    }

    public synchronized NavX getNavXBoard() {
        return mNavXBoard;
    }

    public synchronized void setGyroAngle(Rotation2d angle) {
        mNavXBoard.reset();
        mNavXBoard.setAngleAdjustment(angle);
    }

    public synchronized double getGyroVelocityDegreesPerSec() {
        return mNavXBoard.getYawRateDegreesPerSec();
    }

    /**
     * Update the heading at which the robot thinks the boiler is.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateGoalHeading(double timestamp) {
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        if (aim.isPresent()) {
            mTargetHeading = aim.get().getRobotToGoal();
        }
    }

    /**
     * Turn the robot to a target heading.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateTurnToHeading(double timestamp) {
//        if (Superstructure.getInstance().isShooting()) {
//            // Do not update heading while shooting - just base lock. By not updating the setpoint, we will fight to
//            // keep position.
//            return;
//        }
        final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();

        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

        // Check if we are on target
        final double kGoalPosTolerance = 0.75; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
                && Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
                && Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            return;
        }

        Kinematics.DriveVelocity wheel_delta = Kinematics
                .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
                wheel_delta.right + getRightDistanceInches());
    }

    /**
     * Essentially does the same thing as updateTurnToHeading but sends the robot into the DRIVE_TOWARDS_GOAL_APPROACH
     * state if it detects we are not at an optimal shooting range
     */
    private void updateDriveTowardsGoalCoarseAlign(double timestamp) {
        updateGoalHeading(timestamp);
        updateTurnToHeading(timestamp);
        mIsApproaching = true;
        if (mIsOnTarget) {
            // Done coarse alignment.

            Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
            if (aim.isPresent()) {
                final double distance = aim.get().getRange();

                if (distance < Constants.kShooterOptimalRangeCeiling &&
                        distance > Constants.kShooterOptimalRangeFloor) {
                    // Don't drive, just shoot.
                    mDriveControlState = DriveControlState.AIM_TO_GOAL;
                    mIsApproaching = false;
                    mIsOnTarget = false;
                    updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
                    return;
                }
            }

            mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH;
            mIsOnTarget = false;
        }
    }

    /**
     * Drives the robot straight forwards until it is at an optimal shooting distance. Then sends the robot into the
     * AIM_TO_GOAL state for one final alignment
     */
    private void updateDriveTowardsGoalApproach(double timestamp) {
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        mIsApproaching = true;
        if (aim.isPresent()) {
            final double distance = aim.get().getRange();
            double error = 0.0;
            if (distance < Constants.kShooterOptimalRangeFloor) {
                error = distance - Constants.kShooterOptimalRangeFloor;
            } else if (distance > Constants.kShooterOptimalRangeCeiling) {
                error = distance - Constants.kShooterOptimalRangeCeiling;
            }
            final double kGoalPosTolerance = 1.0; // inches
            if (Util.epsilonEquals(error, 0.0, kGoalPosTolerance)) {
                // We are on target. Switch back to auto-aim.
                mDriveControlState = DriveControlState.AIM_TO_GOAL;
                RobotState.getInstance().resetVision();
                mIsApproaching = false;
                updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
                return;
            }
            updatePositionSetpoint(getLeftDistanceInches() + error, getRightDistanceInches() + error);
        } else {
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
        }
    }

    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
            //System.out.println(mPathFollower.getDebug());
            //System.out.println("Left: " + inchesPerSecondToRpm(setpoint.left) + ", Right: " + inchesPerSecondToRpm(setpoint.right));
            //System.out.println("Left Actual: " + Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0)) + ", Right Actual: " + Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0)));
        } else {
            //System.out.println("Done");
            updateVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isOnTarget() {
        // return true;
        return mIsOnTarget;
    }

    public synchronized boolean isAutoAiming() {
        return mDriveControlState == DriveControlState.AIM_TO_GOAL;
    }

    /**
     * Configures the drivebase for auto aiming
     */
    public synchronized void setWantAimToGoal() {
        if (mDriveControlState != DriveControlState.AIM_TO_GOAL) {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.AIM_TO_GOAL;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            mTargetHeading = getGyroAngle();
        }
        setHighGear(false);
    }

    /**
     * Configures the drivebase for auto driving
     */
    public synchronized void setWantDriveTowardsGoal() {
        if (mDriveControlState != DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN &&
                mDriveControlState != DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH &&
                mDriveControlState != DriveControlState.AIM_TO_GOAL) {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            mTargetHeading = getGyroAngle();
        }
        setHighGear(false);
    }

    /**
     * Configures the drivebase to turn to a desired heading
     */
    public synchronized void setWantTurnToHeading(Rotation2d heading) {
        if (mDriveControlState != DriveControlState.TURN_TO_HEADING) {
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.TURN_TO_HEADING;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
        }
        if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
            mTargetHeading = heading;
            mIsOnTarget = false;
        }
        setHighGear(false);
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    public boolean isApproaching() {
        return mIsApproaching;
    }

    public synchronized boolean isDoneWithTurn() {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING) {
            return mIsOnTarget;
        } else {
            System.out.println("Robot is not in turn to heading mode");
            return false;
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public synchronized void reloadGains() {
//        setPIDGains(mLeftMaster, Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
//                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
//                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate, kLowGearPositionControlSlot);
        mLeftMaster.configMotionCruiseVelocity(Util.convertRPMToNativeUnits(Constants.kDriveLowGearMaxVelocity), Constants.kTimeoutMs);
        mLeftMaster.configMotionAcceleration(Util.convertRPMToNativeUnits(Constants.kDriveLowGearMaxAccel), Constants.kTimeoutMs);
//        setPIDGains(mRightMaster, Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
//                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
//                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
//                kLowGearPositionControlSlot);
        mRightMaster.configMotionCruiseVelocity(Util.convertRPMToNativeUnits(Constants.kDriveLowGearMaxVelocity), Constants.kTimeoutMs);
        mRightMaster.configMotionAcceleration(Util.convertRPMToNativeUnits(Constants.kDriveLowGearMaxAccel), Constants.kTimeoutMs);
        //mLeftMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
        //mRightMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);

//        setPIDGains(mLeftMaster, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
//                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
//                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
//                kHighGearVelocityControlSlot);
//        setPIDGains(mRightMaster, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
//                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
//                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
//                kHighGearVelocityControlSlot);

        setPIDGains(mLeftMaster, 0.43, 0.4, 5, 0.385, 15, 0.25, kLowGearPositionControlSlot);
        setPIDGains(mLeftMaster, 0.43, 0.4, 5, 0.385, 15, 0.25, kHighGearVelocityControlSlot);
        setPIDGains(mRightMaster, 0.43, 0.4, 5, 0.385, 15, 0.25, kLowGearPositionControlSlot);
        setPIDGains(mRightMaster, 0.43, 0.4, 5, 0.385, 15, 0.25, kHighGearVelocityControlSlot);

        //mLeftMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
        //mRightMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
    }

    private void setPIDGains(TalonSRX talon, double kP, double kI, double kD, double kF, double kDriveLowGearPositionIZone, double kDriveLowGearPositionRampRate, int slotID) {
        talon.config_kP(slotID, kP, Constants.kTimeoutMs);
        talon.config_kI(slotID, kI, Constants.kTimeoutMs);
        talon.config_kD(slotID, kD, Constants.kTimeoutMs);
        talon.config_kF(slotID, kF, Constants.kTimeoutMs);
        talon.config_IntegralZone(slotID, (int)kDriveLowGearPositionIZone, Constants.kTimeoutMs);
        talon.configClosedloopRamp(kDriveLowGearPositionRampRate,  Constants.kTimeoutMs);
    }

    public synchronized double getAccelX() {
        return mNavXBoard.getRawAccelX();
    }

    @Override
    public void writeToLog() {
        mCSVWriter.write();
    }

    public boolean checkSystem() {
//        System.out.println("Testing DRIVE.---------------------------------");
//        final double kCurrentThres = 0.5;
//        final double kRpmThres = 300;
//
//        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
//        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Voltage);
//        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
//        mLeftSlave.changeControlMode(CANTalon.TalonControlMode.Voltage);
//
//        mRightMaster.set(0.0);
//        mRightSlave.set(0.0);
//        mLeftMaster.set(0.0);
//        mLeftSlave.set(0.0);
//
//        mRightMaster.set(-6.0f);
//        Timer.delay(4.0);
//        final double currentRightMaster = mRightMaster.getOutputCurrent();
//        final double rpmRightMaster = mRightMaster.getSpeed();
//        mRightMaster.set(0.0f);
//
//        Timer.delay(2.0);
//
//        mRightSlave.set(-6.0f);
//        Timer.delay(4.0);
//        final double currentRightSlave = mRightSlave.getOutputCurrent();
//        final double rpmRightSlave = mRightMaster.getSpeed();
//        mRightSlave.set(0.0f);
//
//        Timer.delay(2.0);
//
//        mLeftMaster.set(6.0f);
//        Timer.delay(4.0);
//        final double currentLeftMaster = mLeftMaster.getOutputCurrent();
//        final double rpmLeftMaster = mLeftMaster.getSpeed();
//        mLeftMaster.set(0.0f);
//
//        Timer.delay(2.0);
//
//        mLeftSlave.set(6.0f);
//        Timer.delay(4.0);
//        final double currentLeftSlave = mLeftSlave.getOutputCurrent();
//        final double rpmLeftSlave = mLeftMaster.getSpeed();
//        mLeftSlave.set(0.0);
//
//        mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
//        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
//
//        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
//        mRightSlave.set(Constants.kRightDriveMasterId);
//
//        mLeftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
//        mLeftSlave.set(Constants.kLeftDriveMasterId);
//
//        System.out.println("Drive Right Master Current: " + currentRightMaster + " Drive Right Slave Current: "
//                + currentRightSlave);
//        System.out.println(
//                "Drive Left Master Current: " + currentLeftMaster + " Drive Left Slave Current: " + currentLeftSlave);
//        System.out.println("Drive RPM RMaster: " + rpmRightMaster + " RSlave: " + rpmRightSlave + " LMaster: "
//                + rpmLeftMaster + " LSlave: " + rpmLeftSlave);
//
//        boolean failure = false;
//
//        if (currentRightMaster < kCurrentThres) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Master Current Low !!!!!!!!!!");
//        }
//
//        if (currentRightSlave < kCurrentThres) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Slave Current Low !!!!!!!!!!");
//        }
//
//        if (currentLeftMaster < kCurrentThres) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Master Current Low !!!!!!!!!!");
//        }
//
//        if (currentLeftSlave < kCurrentThres) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Slave Current Low !!!!!!!!!!");
//        }
//
//        if (!Util.allCloseTo(Arrays.asList(currentRightMaster, currentRightSlave), currentRightMaster,
//                5.0)) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Currents Different !!!!!!!!!!");
//        }
//
//        if (!Util.allCloseTo(Arrays.asList(currentLeftMaster, currentLeftSlave), currentLeftSlave,
//                5.0)) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Currents Different !!!!!!!!!!!!!");
//        }
//
//        if (rpmRightMaster < kRpmThres) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Master RPM Low !!!!!!!!!!!!!!!!!!!");
//        }
//
//        if (rpmRightSlave < kRpmThres) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Slave RPM Low !!!!!!!!!!!!!!!!!!!");
//        }
//
//        if (rpmLeftMaster < kRpmThres) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Master RPM Low !!!!!!!!!!!!!!!!!!!");
//        }
//
//        if (rpmLeftSlave < kRpmThres) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Slave RPM Low !!!!!!!!!!!!!!!!!!!");
//        }
//
//        if (!Util.allCloseTo(Arrays.asList(rpmRightMaster, rpmRightSlave, rpmLeftMaster, rpmLeftSlave),
//                rpmRightMaster, 250)) {
//            failure = true;
//            System.out.println("!!!!!!!!!!!!!!!!!!! Drive RPMs different !!!!!!!!!!!!!!!!!!!");
//        }
//
//        return !failure;
        return false;
    }
}
