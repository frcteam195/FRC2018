package org.usfirst.frc.team195.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;


public class DriveBase {
	
	private CheesyDriveHelper mCheesyDriveHelper;
	private Joystick driveStick;
	private TalonSRX leftFront;
	private TalonSRX leftMid;
	private TalonSRX leftBack;
	private TalonSRX rightFront;
	private TalonSRX rightMid;
	private TalonSRX rightBack;
	private double x;
	private double y;
	
	public DriveBase() {
		driveStick = new Joystick(0);
		mCheesyDriveHelper = new CheesyDriveHelper();
		x = 0;
		y = 0;

		leftFront = new TalonSRX(1);
		leftMid = new TalonSRX(2);
		leftBack = new TalonSRX(3);
		rightFront = new TalonSRX(4);
		rightMid = new TalonSRX(5);
		rightBack = new TalonSRX(6);


		rightFront.setInverted(true);
		rightMid.setInverted(true);
		rightBack.setInverted(true);

		leftMid.follow(leftFront);
		leftBack.follow(leftFront);
		rightMid.follow(rightFront);
		rightBack.follow(rightFront);
		
	}
	public void run() {
		x = driveStick.getRawAxis(2);
		y = -driveStick.getRawAxis(1);
		
		DriveSignal mDriveSignal;

		mDriveSignal = mCheesyDriveHelper.cheesyDrive(y, x, driveStick.getRawButton(8), false);

		leftFront.set(ControlMode.PercentOutput,  mDriveSignal.getLeft());
		rightFront.set(ControlMode.PercentOutput, mDriveSignal.getRight());
	}
}
