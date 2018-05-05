package com.team195.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team195.RobotMap;
import com.team195.utils.DriveMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Drive {
	private static Drive instance = null;

	private DriveMode currentDriveMode;

	private TalonSRX mLeftDrive;
	private TalonSRX mLeftSlave1;
	private TalonSRX mLeftSlave2;
	private TalonSRX mRightDrive;
	private TalonSRX mRightSlave1;
	private TalonSRX mRightSlave2;

	private DoubleSolenoid mShiftSol;

	private Drive() {
		currentDriveMode = DriveMode.JOYSTICK;

		mLeftDrive = new TalonSRX(RobotMap.LEFT_DRIVE_1);
		mLeftSlave1 = new TalonSRX(RobotMap.LEFT_DRIVE_2);
		mLeftSlave2 = new TalonSRX(RobotMap.LEFT_DRIVE_3);
		mRightDrive = new TalonSRX(RobotMap.RIGHT_DRIVE_1);
		mRightSlave1 = new TalonSRX(RobotMap.RIGHT_DRIVE_2);
		mRightSlave2 = new TalonSRX(RobotMap.RIGHT_DRIVE_3);

		mLeftDrive.setInverted(RobotMap.REVERSE_LEFT_DRIVE);
		mLeftSlave1.setInverted(!RobotMap.REVERSE_RIGHT_DRIVE);
		mLeftSlave2.setInverted(RobotMap.REVERSE_LEFT_DRIVE);
		mRightDrive.setInverted(RobotMap.REVERSE_RIGHT_DRIVE);
		mRightSlave1.setInverted(RobotMap.REVERSE_RIGHT_DRIVE);
		mRightSlave2.setInverted(RobotMap.REVERSE_RIGHT_DRIVE);

		mLeftSlave1.follow(mLeftDrive);
		mLeftSlave2.follow(mLeftDrive);
		mRightSlave1.follow(mRightDrive);
		mRightSlave2.follow(mRightDrive);

		final int kTimeoutMs = 20;
		final double rampRate = 0.3;

		mLeftDrive.configOpenloopRamp(rampRate, kTimeoutMs);
		mLeftSlave1.configOpenloopRamp(rampRate, kTimeoutMs);
		mLeftSlave2.configOpenloopRamp(rampRate, kTimeoutMs);
		mRightDrive.configOpenloopRamp(rampRate, kTimeoutMs);
		mRightSlave1.configOpenloopRamp(rampRate, kTimeoutMs);
		mRightSlave2.configOpenloopRamp(rampRate, kTimeoutMs);

		mShiftSol = new DoubleSolenoid(RobotMap.SHIFT_SOL_REVERSE, RobotMap.SHIFT_SOL_FORWARD);
	}

	public static Drive getInstance() {
		if (instance == null)
			instance = new Drive();

		return instance;
	}

	public synchronized void setDriveMode(DriveMode mode) {
		if (mode != currentDriveMode)
			currentDriveMode = mode;
	}
	
	public DriveMode getDriveMode() {
		return currentDriveMode;
	}

	public synchronized void setDrive(double leftSpeed, double rightSpeed) {
		setLeftDrive(leftSpeed);
		setRightDrive(rightSpeed);
	}

	public synchronized void setLeftDrive(double speed) {
		mLeftDrive.set(ControlMode.PercentOutput, speed);
	}

	public synchronized void setRightDrive(double speed) {
		mRightDrive.set(ControlMode.PercentOutput, speed);
	}

	public synchronized void shiftHigh() {
		if(mShiftSol.get() != DoubleSolenoid.Value.kForward)
			mShiftSol.set(DoubleSolenoid.Value.kForward);
	}

	public synchronized void shiftLow() {
		if(mShiftSol.get() != DoubleSolenoid.Value.kReverse)
			mShiftSol.set(DoubleSolenoid.Value.kReverse);
	}

	public boolean isHighGear() {
		return mShiftSol.get() == DoubleSolenoid.Value.kForward;
	}
}
