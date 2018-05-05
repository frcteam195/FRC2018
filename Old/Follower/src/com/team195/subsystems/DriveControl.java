package com.team195.subsystems;

import com.team195.subsystems.PIDController;
import com.team195.RobotMap;
import com.team195.utils.DriveMode;
import com.team195.utils.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class DriveControl extends Thread {
	private static DriveControl instance;

	private Joystick driveJoystick;
	private Joystick driveJoystick2;

	private double x;
	private double y;
	private double left;
	private double right;
	private double normalLeft;
	private double normalRight;

	private DriveControl() {
		driveJoystick = new Joystick(RobotMap.DRIVE_JOYSTICK_PORT);
		driveJoystick2 = new Joystick(RobotMap.DRIVE_JOYSTICK_2_PORT);

		x = 0.0;
		y = 0.0;
		left = 0.0;
		right = 0.0;
		normalLeft = 0.0;
		normalRight = 0.0;
	}

	public static DriveControl getInstance() {
		if (instance == null)
			instance = new DriveControl();

		return instance;
	}

	@Override
	public void run() {
		while (true) {
			driveModeControl();

			if (Drive.getInstance().getDriveMode() == DriveMode.JOYSTICK) {
				driveWithJoystickControl();
				shiftControl();
			} else {
				driveWithVisionControl();
			}

			try {
				Thread.sleep(10);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	public void driveModeControl() {
//		if (driveJoystick.getPOV() == 0)
//			Drive.getInstance().setDriveMode(DriveMode.VISION);
//		else if (driveJoystick.getPOV() == 180)
//			Drive.getInstance().setDriveMode(DriveMode.JOYSTICK);
	}

	public void driveWithJoystickControl() {
		//  Allows for both joysticks to control the drivebase at the same time
		x = driveJoystick.getRawAxis(0) + driveJoystick2.getRawAxis(4);
		y = -driveJoystick.getRawAxis(1) - driveJoystick2.getRawAxis(1);
		left = y + x;
		right = y - x;
		double[] normalized = Util.normalizeDriveValues(left, right, 1);
		normalLeft = normalized[0];
		normalRight = normalized[1];
		Drive.getInstance().setDrive(normalLeft, normalRight);
	}

	public void shiftControl() {
		if(driveJoystick.getRawButton(6) || driveJoystick2.getRawButton(6))
			Drive.getInstance().shiftHigh();
		else if (driveJoystick.getRawButton(7) || driveJoystick2.getRawButton(5))
			Drive.getInstance().shiftLow();
	}

	public void driveWithVisionControl() {
		PIDController.getInstance().setOutputs();
	}
}
