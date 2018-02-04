package org.usfirst.frc.team195.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Drivers.NavX;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TuneablePID;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

public class DriveBaseSubsystem extends Thread implements CriticalSystemStatus, CustomSubsystem, DiagnosableSubsystem, Reportable {
	private static final int kLowGearPIDSlot = 0;
	private static final int kHighGearPIDSlot = 1;
	public static final int MIN_DRIVE_LOOP_TIME_STANDARD = 10;
	public static final int MIN_DRIVE_LOOP_TIME_MP = 3;
	private static DriveBaseSubsystem instance = null;
	private static ReentrantLock _subsystemMutex = new ReentrantLock();
	private PathFollowerRobotState mRobotState = PathFollowerRobotState.getInstance();
	private DriveControlState mControlMode;
	private DriveControlState mPrevControlMode;
	private TalonSRX mLeftMaster, mRightMaster;
	private BaseMotorController leftDriveSlave1, leftDriveSlave2, rightDriveSlave1, rightDriveSlave2;
	private DriverStation ds;
	private NavX mNavXBoard;
	private Solenoid shiftSol;
	private boolean mPrevShiftVal;
	private boolean mPrevBrakeModeVal;
	private double leftDriveSpeed, rightDriveSpeed;
	private boolean runThread;
	private Path mCurrentPath = null;
	private PathFollower mPathFollower;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private TuneablePID tuneableDrive;
	private SetpointValue setpointValue = new SetpointValue();

	private DriveBaseSubsystem() throws Exception {
		super();
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

		shiftSol = robotControllers.getShiftSol();
		mPrevShiftVal = false;
		setGear(true);

		leftDriveSpeed = 0;
		rightDriveSpeed = 0;

		mPrevBrakeModeVal = false;
		setBrakeMode(true);

		runThread = false;

		mControlMode = DriveControlState.PATH_FOLLOWING;
		mPrevControlMode = DriveControlState.OPEN_LOOP;

		tuneableDrive = new TuneablePID("Left Drive Tuning", mLeftMaster, null, 5808, true, true);
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


		mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		mLeftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs);
		mLeftMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs);

		mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		mRightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs);
		mRightMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs);

		TalonHelper.setPIDGains(mLeftMaster, kLowGearPIDSlot, 0.2, 0, 2, 0.1);
		TalonHelper.setPIDGains(mLeftMaster, kHighGearPIDSlot, 0.2, 0, 2, 0.121);
		TalonHelper.setPIDGains(mRightMaster, kLowGearPIDSlot, 0.2, 0, 2, 0.1);
		TalonHelper.setPIDGains(mRightMaster, kHighGearPIDSlot, 0.2, 0, 2, 0.121);

		isSystemFaulted();
	}

	@Override
	public void start() {
		runThread = true;
		tuneableDrive.start();
		if (!super.isAlive())
			super.start();
	}

	@Override
	public void terminate() {
		ConsoleReporter.report("CAN'T STOP, WON'T STOP, DON'T CALL ME!", MessageLevel.ERROR);
//		runThread = false;
//		try {
//			super.join(Constants.kThreadJoinTimeout);
//		} catch (Exception ex) {
//			ConsoleReporter.report(ex);
//		}
	}

	@Override
	public void run() {
		while (!ds.isEnabled()) {try{Thread.sleep(20);}catch(Exception ex) {}}
		subsystemHome();
		threadRateControl.start();

		while(runThread) {
			if(mPrevControlMode != mControlMode) {
				ConsoleReporter.report("Changing Control Modes!", MessageLevel.INFO);

				switch (mControlMode) {
					case PATH_FOLLOWING:
					case VELOCITY:
						mLeftMaster.set(ControlMode.Velocity, 0);
						mRightMaster.set(ControlMode.Velocity, 0);
						setBrakeMode(true);
						break;
					case POSITION:
						mLeftMaster.set(ControlMode.Position, mLeftMaster.getSelectedSensorPosition(0));
						mRightMaster.set(ControlMode.Position, mRightMaster.getSelectedSensorPosition(0));
						setBrakeMode(true);
						break;
					case TEST:
						break;
					case OPEN_LOOP:
					default:
						mLeftMaster.set(ControlMode.PercentOutput, 0);
						mRightMaster.set(ControlMode.PercentOutput, 0);
						setBrakeMode(false);
						break;
				}
				_subsystemMutex.lock();
				mPrevControlMode = mControlMode;
				_subsystemMutex.unlock();
			}

			switch (mControlMode) {
				case PATH_FOLLOWING:
					if (mPathFollower != null)
						updatePathFollower();
				case VELOCITY:
					//mLeftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(leftDriveSpeed));
					//mRightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(rightDriveSpeed));
					break;
				case POSITION:
					break;
				case TEST:
					break;
				case OPEN_LOOP:
				default:
					//mLeftMaster.set(ControlMode.PercentOutput, leftDriveSpeed);
					//mRightMaster.set(ControlMode.PercentOutput, rightDriveSpeed);
					break;
			}

			threadRateControl.doRateControl(mControlMode == DriveControlState.PATH_FOLLOWING ? MIN_DRIVE_LOOP_TIME_MP : MIN_DRIVE_LOOP_TIME_STANDARD);
		}
	}

	@Override
	public void subsystemHome() {
		mNavXBoard.zeroYaw();
		mLeftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		mRightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
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
		if (ds.isTest()) {
			setControlMode(DriveControlState.TEST);
			Timer.delay(1);

			ConsoleReporter.report("Testing DRIVE---------------------------------");
			final double kLowCurrentThres = 0.5;
			final double kLowRpmThres = 100;

			ArrayList<MotorDiagnostics> mAllMotorsDiagArr = new ArrayList<MotorDiagnostics>();
			ArrayList<MotorDiagnostics> mLeftDiagArr = new ArrayList<MotorDiagnostics>();
			ArrayList<MotorDiagnostics> mRightDiagArr = new ArrayList<MotorDiagnostics>();
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Master", mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Slave 1", leftDriveSlave1, mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Slave 2", leftDriveSlave2, mLeftMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Master", mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Slave 1", rightDriveSlave1, mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Slave 2", rightDriveSlave2, mRightMaster));

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
			}

			if (mLeftDiagArr.size() > 0 && mRightDiagArr.size() > 0 && mAllMotorsDiagArr.size() > 0) {
				List<Double> leftMotorCurrents = mLeftDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(leftMotorCurrents, leftMotorCurrents.get(0), 5.0)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Left Currents Different !!!!!!!!!!");
				}

				List<Double> rightMotorCurrents = mRightDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(rightMotorCurrents, rightMotorCurrents.get(0), 5.0)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Right Currents Different !!!!!!!!!!");
				}

				List<Double> driveMotorRPMs = mAllMotorsDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
				if (!Util.allCloseTo(driveMotorRPMs, driveMotorRPMs.get(0), 40)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!!! Drive RPMs different !!!!!!!!!!!!!!!!!!!");
				}
			} else {
				ConsoleReporter.report("Drive Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
			}

			return !failure;
		} else
			return false;
	}

	@Override
	public boolean isSystemFaulted() {
		boolean allSensorsPresent = true;

		boolean leftSensorPresent = mLeftMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		boolean rightSensorPresent = mRightMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		allSensorsPresent &= leftSensorPresent;
		allSensorsPresent &= rightSensorPresent;
		if (!leftSensorPresent || !rightSensorPresent) {
			String msg = "Could not detect encoder! \r\n\tLeft Encoder Detected: " + leftSensorPresent + "\r\n\tRight Encoder Detected: " + rightSensorPresent;
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

		return !allSensorsPresent;
	}

	public boolean isHighGear() {
		return mPrevShiftVal;
	}

	public void setGear(boolean highGear) {
		try {
			if (shiftSol != null) {
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

					shiftSol.set(highGear);
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
		setDriveSpeed(d);
	}

	public synchronized void setDriveVelocity(DriveMotorValues d) {
		setControlMode(DriveControlState.VELOCITY);
		setDriveSpeed(d);
	}

	private synchronized void setDriveSpeed(DriveMotorValues d) {
		setDriveSpeed(d.leftDrive, d.rightDrive, false);
	}

	private synchronized void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed) {
		setDriveSpeed(leftDriveSpeed, rightDriveSpeed, false);
	}

	private synchronized void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed, boolean slowDown) {
		try {
			_subsystemMutex.lock();

			this.leftDriveSpeed = leftDriveSpeed;
			this.rightDriveSpeed = rightDriveSpeed;

			if(slowDown) {
				this.leftDriveSpeed /= 2.2;
				this.rightDriveSpeed /= 2.2;
			}

			_subsystemMutex.unlock();
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
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
	private void updatePathFollower() {
		RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
		Twist2d command = mPathFollower.update(Timer.getFPGATimestamp(), robot_pose,
				PathFollowerRobotState.getInstance().getDistanceDriven(), PathFollowerRobotState.getInstance().getPredictedVelocity().dx);

		if (!mPathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			updatePathVelocitySetpoint(setpoint.left, setpoint.right);

			//ConsoleReporter.report(mPathFollower.getDebug());
			//ConsoleReporter.report("Left: " + inchesPerSecondToRpm(setpoint.left) + ", Right: " + inchesPerSecondToRpm(setpoint.right));
			//ConsoleReporter.report("Left Actual: " + Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0)) + ", Right Actual: " + Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0)));
		} else {
			updatePathVelocitySetpoint(0, 0);
			ConsoleReporter.report("Completed path!");
			setControlMode(DriveControlState.VELOCITY);
		}
	}

	private void updatePathVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
		final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
		try {
			_subsystemMutex.lock();
			leftDriveSpeed = Util.convertRPMToNativeUnits(inchesPerSecondToRpm(left_inches_per_sec * scale));
			rightDriveSpeed = Util.convertRPMToNativeUnits(inchesPerSecondToRpm(right_inches_per_sec * scale));
			_subsystemMutex.unlock();
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
		//ConsoleReporter.report("Requested Drive Velocity Left/Right: " + left_inches_per_sec + "/" + right_inches_per_sec);
		//ConsoleReporter.report("Actual Drive Velocity Left/Right: " + getLeftVelocityInchesPerSec() + "/" + getRightVelocityInchesPerSec());
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
