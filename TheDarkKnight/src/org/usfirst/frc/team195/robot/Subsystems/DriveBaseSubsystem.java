package org.usfirst.frc.team195.robot.Subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Drivers.NavX;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TuneablePID;
import org.usfirst.frc.team195.robot.Utilities.Loops.Loop;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

public class DriveBaseSubsystem implements CriticalSystemStatus, CustomSubsystem, DiagnosableSubsystem, Reportable {
	private static final int kLowGearPIDSlot = 0;
	private static final int kHighGearPIDSlot = 1;
	private static DriveBaseSubsystem instance = null;
	private static ReentrantLock _subsystemMutex = new ReentrantLock();
	private PathFollowerRobotState mRobotState = PathFollowerRobotState.getInstance();
	private DriveControlState mControlMode;
	private TalonSRX mLeftMaster, mRightMaster;
	private BaseMotorController leftDriveSlave1, leftDriveSlave2, rightDriveSlave1, rightDriveSlave2;
	private DriverStation ds;
	private NavX mNavXBoard;
	private ShiftHelper shiftHelper;
	private boolean mPrevShiftVal;
	private boolean mPrevBrakeModeVal;
	private double leftDriveSpeed, rightDriveSpeed;
	private Path mCurrentPath = null;
	private PathFollower mPathFollower;
	private TuneablePID tuneableLeftDrive;
	private TuneablePID tuneableRightDrive;
	private SetpointValue leftSetpointValue = new SetpointValue();
	private SetpointValue rightSetpointValue = new SetpointValue();
	private Rotation2d mTargetHeading = new Rotation2d();
	private boolean mIsOnTarget = false;


	private boolean emergencySafetyRequired = false;

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (DriveBaseSubsystem.this) {
				subsystemHome();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (DriveBaseSubsystem.this) {
				setDriveOpenLoop(DriveMotorValues.NEUTRAL);
				setBrakeMode(false);
				setDriveVelocity(new DriveMotorValues(0, 0));
			}
		}

		@Override
		public void onLoop(double timestamp, boolean isAuto) {
			synchronized (DriveBaseSubsystem.this) {
				switch (mControlMode) {
					case OPEN_LOOP:
						break;
					case VELOCITY:
						break;
					case TURN_TO_HEADING:
						updateTurnToHeading(timestamp);
						break;
					case PATH_FOLLOWING:
						if (mPathFollower != null) {
							updatePathFollower(timestamp);
							//mCSVWriter.add(mPathFollower.getDebug());
						}
						break;
					default:
						ConsoleReporter.report("Unexpected drive control state: " + mControlMode);
						break;
				}

				if (mControlMode != DriveControlState.PATH_FOLLOWING)
					emergencySafetyRequired = mNavXBoard.isCollisionOccurring() || mNavXBoard.isTipping();

			}
		}
		@Override
		public void onStop(double timestamp) {
			setDriveOpenLoop(DriveMotorValues.NEUTRAL);
		}
	};

	@Override
	public void registerEnabledLoops(Looper in) {
		in.register(mLoop);
	}

	private DriveBaseSubsystem() throws Exception {
		ds = DriverStation.getInstance();
		_subsystemMutex = new ReentrantLock();

		Controllers robotControllers = Controllers.getInstance();
		mLeftMaster = robotControllers.getLeftDrive1();
		leftDriveSlave1 = robotControllers.getLeftDrive2();
		leftDriveSlave2 = robotControllers.getLeftDrive3();
		mRightMaster = robotControllers.getRightDrive1();
		rightDriveSlave1 = robotControllers.getRightDrive2();
		rightDriveSlave2 = robotControllers.getRightDrive3();
		mNavXBoard = robotControllers.getNavX();

		shiftHelper = robotControllers.getShiftHelper();
		if (shiftHelper != null)
			shiftHelper.configHighGear(false);
		mPrevShiftVal = false;
		setGear(false);

		leftDriveSpeed = 0;
		rightDriveSpeed = 0;

		mPrevBrakeModeVal = false;
		setBrakeMode(true);

		mControlMode = DriveControlState.PATH_FOLLOWING;

//		tuneableLeftDrive = new TuneablePID("Drive Tuning", mLeftMaster, mRightMaster, leftSetpointValue, 5808, true, false);
//		tuneableRightDrive = new TuneablePID("Right2Cube Drive Tuning", mRightMaster, rightSetpointValue, 5809, true, false);
//		tuneableLeftDrive.start();

	}

	public static DriveBaseSubsystem getInstance() {
		if(instance == null) {
			try {
				instance = new DriveBaseSubsystem();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		return instance;
	}

	public static DriveBaseSubsystem getInstance(List<CustomSubsystem> subsystemList) {
		subsystemList.add(getInstance());
		return instance;
	}

	@Override
	public void init() {
		mLeftMaster.setSensorPhase(true);

		mRightMaster.setSensorPhase(true);
		mRightMaster.setInverted(true);
		rightDriveSlave1.setInverted(true);
		rightDriveSlave2.setInverted(true);

		setBrakeMode(true);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mLeftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mLeftMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mRightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mRightMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		setSucceeded &= TalonHelper.setPIDGains(mLeftMaster, kLowGearPIDSlot, Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi, Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf, Constants.kDriveLowGearPositionRampRate, Constants.kDriveLowGearPositionIZone);
		setSucceeded &= TalonHelper.setPIDGains(mLeftMaster, kHighGearPIDSlot, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		setSucceeded &= TalonHelper.setPIDGains(mRightMaster, kLowGearPIDSlot, Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi, Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf, Constants.kDriveLowGearPositionRampRate, Constants.kDriveLowGearPositionIZone);
		setSucceeded &= TalonHelper.setPIDGains(mRightMaster, kHighGearPIDSlot, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		setSucceeded &= TalonHelper.setMotionMagicParams(mLeftMaster, (int)Constants.kDriveLowGearMaxVelocity, (int)Constants.kDriveLowGearMaxAccel);
		setSucceeded &= TalonHelper.setMotionMagicParams(mRightMaster, (int)Constants.kDriveLowGearMaxVelocity, (int)Constants.kDriveLowGearMaxAccel);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize DriveBaseSubsystem!!!", MessageLevel.DEFCON1);

		mNavXBoard.setCollisionJerkThreshold(Constants.kCollisionDetectionJerkThreshold);
		mNavXBoard.setTippingThreshold(Constants.kTippingThresholdDeg);

		isSystemFaulted();
	}

	@Override
	public void subsystemHome() {
		mNavXBoard.zeroYaw();

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mLeftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
			setSucceeded &= mRightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to zero DriveBaseSubsystem!!!", MessageLevel.DEFCON1);
	}

	@Override
	public String toString() {
		return generateReport();
	}

	@Override
	public String generateReport() {
		String retVal = "";

		retVal += "LeftDrivePos:" + mLeftMaster.getSelectedSensorVelocity(0) + ";";
		retVal += "LeftDriveVel:" + mLeftMaster.getSelectedSensorPosition(0) + ";";
		retVal += "LeftDriveOutput:" + leftDriveSpeed + ";";
		retVal += "LeftDrive1Current:" + mLeftMaster.getOutputCurrent() + ";";
		retVal += "LeftDrive2Current:" + leftDriveSlave1.getOutputCurrent() + ";";
		retVal += "LeftDrive3Current:" + leftDriveSlave2.getOutputCurrent() + ";";

		retVal += "RightDrivePos:" + mRightMaster.getSelectedSensorVelocity(0) + ";";
		retVal += "RightDriveVel:" + mRightMaster.getSelectedSensorPosition(0) + ";";
		retVal += "RightDriveOutput:" + rightDriveSpeed + ";";
		retVal += "RightDrive1Current:" + mRightMaster.getOutputCurrent() + ";";
		retVal += "RightDrive2Current:" + rightDriveSlave1.getOutputCurrent() + ";";
		retVal += "RightDrive3Current:" + rightDriveSlave2.getOutputCurrent() + ";";

		return retVal;
	}

	@Override
	public boolean runDiagnostics() {

		if (ds.isTest() && Constants.ENABLE_DRIVE_DIAG) {
			setControlMode(DriveControlState.TEST);
			Timer.delay(1);

			ConsoleReporter.report("Testing DRIVE---------------------------------");
			final double kLowCurrentThres = Constants.kDriveBaseTestLowCurrentThresh;
			final double kLowRpmThres = Constants.kDriveBaseTestLowRPMThresh;

			ArrayList<MotorDiagnostics> mAllMotorsDiagArr = new ArrayList<MotorDiagnostics>();
			ArrayList<MotorDiagnostics> mLeftDiagArr = new ArrayList<MotorDiagnostics>();
			ArrayList<MotorDiagnostics> mRightDiagArr = new ArrayList<MotorDiagnostics>();
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left2Cube Master", mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left2Cube Slave 1", leftDriveSlave1, mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left2Cube Slave 2", leftDriveSlave2, mLeftMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right2Cube Master", mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right2Cube Slave 1", rightDriveSlave1, mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right2Cube Slave 2", rightDriveSlave2, mRightMaster));

			mAllMotorsDiagArr.addAll(mLeftDiagArr);
			mAllMotorsDiagArr.addAll(mRightDiagArr);

			boolean failure = false;

			for (MotorDiagnostics mD : mAllMotorsDiagArr) {
				mD.setZero();
			}

			for (MotorDiagnostics mD : mAllMotorsDiagArr) {
				mD.runTest();

				if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
					failure = true;
				}

				if (mD.isRPMUnderThreshold(kLowRpmThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " RPM Low !!!!!!!!!!");
					failure = true;
				}

				if (!mD.isSensorInPhase()) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Sensor Out of Phase !!!!!!!!!!");
					failure = true;
				}
			}

			if (mLeftDiagArr.size() > 0 && mRightDiagArr.size() > 0 && mAllMotorsDiagArr.size() > 0) {
				List<Double> leftMotorCurrents = mLeftDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(leftMotorCurrents, leftMotorCurrents.get(0), Constants.kDriveBaseTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Left2Cube Currents Different !!!!!!!!!!");
				}

				List<Double> rightMotorCurrents = mRightDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(rightMotorCurrents, rightMotorCurrents.get(0), Constants.kDriveBaseTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Right2Cube Currents Different !!!!!!!!!!");
				}

				List<Double> driveMotorRPMs = mAllMotorsDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
				if (!Util.allCloseTo(driveMotorRPMs, driveMotorRPMs.get(0), Constants.kDriveBaseTestRPMDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!!! Drive RPMs different !!!!!!!!!!!!!!!!!!!");
				}
			} else {
				ConsoleReporter.report("Drive Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
			}

			return !failure;
		} else
			return true;
	}

	@Override
	public boolean isSystemFaulted() {
		boolean allSensorsPresent = true;

		boolean leftSensorPresent = mLeftMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		boolean rightSensorPresent = mRightMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		allSensorsPresent &= leftSensorPresent;
		allSensorsPresent &= rightSensorPresent;
		if (!leftSensorPresent || !rightSensorPresent) {
			String msg = "Could not detect encoder! \r\n\tLeft2Cube Encoder Detected: " + leftSensorPresent + "\r\n\tRight2Cube Encoder Detected: " + rightSensorPresent;
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}

		boolean navXPresent = mNavXBoard.isPresent();
		allSensorsPresent &= navXPresent;
		if (!navXPresent) {
			String msg = "Could not detect navX!";
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}

		checkMotorReset(mLeftMaster, "Left2Cube Drive Master");
		checkMotorReset(leftDriveSlave1, "Left2Cube Drive Slave 1");
		checkMotorReset(leftDriveSlave2, "Left2Cube Drive Slave 2");
		checkMotorReset(mRightMaster, "Right2Cube Drive Master");
		checkMotorReset(rightDriveSlave1, "Right2Cube Drive Slave 1");
		checkMotorReset(rightDriveSlave2, "Right2Cube Drive Slave 2");

		return !allSensorsPresent;
	}

	private boolean checkMotorReset(BaseMotorController motorController, String name) {
		if (motorController.hasResetOccurred()) {

			ConsoleReporter.report(name + " Talon has reset!", MessageLevel.DEFCON1);

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = true;
				setSucceeded &= motorController.clearStickyFaults(Constants.kTimeoutMsFast) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear " + name + " Reset !!!!!!", MessageLevel.DEFCON1);

			return true;
		}

		return false;
	}

	public boolean isHighGear() {
		return mPrevShiftVal;
	}

	public void setGear(boolean highGear) {
		try {
			if (shiftHelper != null) {
				_subsystemMutex.lock();
				if (mPrevShiftVal != highGear) {
					if (highGear) {
						ConsoleReporter.report("Setting drive gains for high gear!", MessageLevel.INFO);
						mLeftMaster.selectProfileSlot(0, kHighGearPIDSlot);
						mRightMaster.selectProfileSlot(0, kHighGearPIDSlot);
					} else {
						ConsoleReporter.report("Setting drive gains for low gear!", MessageLevel.INFO);
						mLeftMaster.selectProfileSlot(0, kLowGearPIDSlot);
						mRightMaster.selectProfileSlot(0, kLowGearPIDSlot);
					}

					shiftHelper.shift(highGear);
					mPrevShiftVal = highGear;
				}
				_subsystemMutex.unlock();
			}
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
	}

	public DriveControlState getControlMode() {
		return mControlMode;
	}

	public void setControlMode(DriveControlState controlMode) {
		if (controlMode != mControlMode) {
			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
				ConsoleReporter.report(ex);
			}
		}
	}

	public synchronized void setDriveOpenLoop(DriveMotorValues d) {
		setControlMode(DriveControlState.OPEN_LOOP);

		mLeftMaster.set(ControlMode.PercentOutput, d.leftDrive);
		mRightMaster.set(ControlMode.PercentOutput, d.rightDrive);
	}

	public synchronized void setDriveVelocity(DriveMotorValues d) {
		setDriveVelocity(d, true);
	}

	public synchronized void setDriveVelocity(DriveMotorValues d, boolean autoChangeMode) {
		if (autoChangeMode)
			setControlMode(DriveControlState.VELOCITY);
		mLeftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(d.leftDrive));
		mRightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(d.rightDrive));
	}

	public void setBrakeMode(boolean brakeMode) {
		if (mPrevBrakeModeVal != brakeMode) {
			_subsystemMutex.lock();
			mLeftMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftDriveSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftDriveSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mRightMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightDriveSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightDriveSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mPrevBrakeModeVal = brakeMode;
			_subsystemMutex.unlock();
		}
	}

	/**
	 * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
	 * pose, distance driven, and velocity, then updates the wheel velocity setpoints.
	 */
	private void updatePathFollower(double timestamp) {
		RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
		Twist2d command = mPathFollower.update(timestamp, robot_pose,
				PathFollowerRobotState.getInstance().getDistanceDriven(), PathFollowerRobotState.getInstance().getPredictedVelocity().dx);

		if (!mPathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			updatePathVelocitySetpoint(setpoint.left, setpoint.right);

			//ConsoleReporter.report(mPathFollower.getDebug());
			//ConsoleReporter.report("Left2Cube: " + inchesPerSecondToRpm(setpoint.left) + ", Right2Cube: " + inchesPerSecondToRpm(setpoint.right));
			//ConsoleReporter.report("Left2Cube Actual: " + Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0)) + ", Right2Cube Actual: " + Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0)));
		} else {
			updatePathVelocitySetpoint(0, 0);
			ConsoleReporter.report("Completed path!");
			setControlMode(DriveControlState.VELOCITY);
		}
	}

	private void updatePathVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
		final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;

		mLeftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(left_inches_per_sec * scale)));
		mRightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(right_inches_per_sec * scale)));

		//ConsoleReporter.report("Requested Drive Velocity Left2Cube/Right2Cube: " + left_inches_per_sec + "/" + right_inches_per_sec);
		//ConsoleReporter.report("Actual Drive Velocity Left2Cube/Right2Cube: " + getLeftVelocityInchesPerSec() + "/" + getRightVelocityInchesPerSec());
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
		return rotationsToInches(mLeftMaster.getSelectedSensorPosition(0)/Constants.kSensorUnitsPerRotation);
	}

	public double getRightDistanceInches() {
		return rotationsToInches(mRightMaster.getSelectedSensorPosition(0)/Constants.kSensorUnitsPerRotation);
	}

	public double getLeftVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0))); }

	public double getRightVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0))); }

	public synchronized Rotation2d getGyroAngle() {
		return mNavXBoard.getYaw();
	}

	public synchronized double getRoll() { return mNavXBoard.getRoll(); }

	public synchronized double getPitch() { return mNavXBoard.getPitch(); }

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
	 * Configures the drivebase to turn to a desired heading
	 */
	public synchronized void setWantTurnToHeading(Rotation2d heading) {
		if (mControlMode != DriveControlState.TURN_TO_HEADING) {
			mControlMode = DriveControlState.TURN_TO_HEADING;
			updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
		}
		if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
			mTargetHeading = heading;
			mIsOnTarget = false;
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
		SmartDashboard.putBoolean("TurnOnTarget", mIsOnTarget);
		final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();

		// Figure out the rotation necessary to turn to face the goal.
		final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

		// Check if we are on target
		final double kGoalPosTolerance = 1; // degrees
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
	 * Adjust position setpoint (if already in position mode)
	 *
	 * @param left_position_inches
	 * @param right_position_inches
	 */
	private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
		mLeftMaster.set(ControlMode.MotionMagic, inchesToRotations(left_position_inches) * 4096);
		mRightMaster.set(ControlMode.MotionMagic, inchesToRotations(right_position_inches) * 4096);
	}

	/**
	 * Configures the drivebase to drive a path. Used for autonomous driving
	 *
	 * @see Path
	 */
	public synchronized void setWantDrivePath(Path path, boolean reversed) {
		if (mCurrentPath != path || mControlMode != DriveControlState.PATH_FOLLOWING) {
			setControlMode(DriveControlState.PATH_FOLLOWING);
			PathFollowerRobotState.getInstance().resetDistanceDriven();
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

			mCurrentPath = path;
		} else {
			ConsoleReporter.report("Error setting path for drive!", MessageLevel.ERROR);
		}
	}

	public synchronized boolean isEmergencySafetyRequired() {
		return emergencySafetyRequired;
	}

	public synchronized boolean isDoneWithTurn() {
		if (mControlMode == DriveControlState.TURN_TO_HEADING) {
			return mIsOnTarget;
		} else {
			ConsoleReporter.report("Robot is not in turning mode");
			return true;
		}
	}

	public synchronized boolean isDoneWithPath() {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			return mPathFollower.isFinished();
		} else {
			ConsoleReporter.report("Robot is not in path following mode");
			return true;
		}
	}

	public synchronized void forceDoneWithPath() {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			mPathFollower.forceFinish();
		} else {
			ConsoleReporter.report("Robot is not in path following mode");
		}
	}

	public synchronized boolean hasPassedMarker(String marker) {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			return mPathFollower.hasPassedMarker(marker);
		} else {
			ConsoleReporter.report("Robot is not in path following mode");
			return false;
		}
	}
}
