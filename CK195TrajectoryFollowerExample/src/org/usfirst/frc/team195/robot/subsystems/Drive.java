package org.usfirst.frc.team195.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.OI;
import org.usfirst.frc.team195.robot.RobotMap;
//import Robot;
import org.usfirst.frc.team195.robot.commands.DriveCommand;
import org.usfirst.frc.team195.robot.utils.QuickMaths;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Drive extends Subsystem {

	private Joystick joy1 = OI.Driver;//RobotMap.joystick1;
	private DifferentialDrive driveTrain;
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public Drive()
	{
		System.out.print("drive constructed");
		driveTrain = RobotMap.driveTrain;
		driveTrain.setSafetyEnabled(false);
	}

	/**
	 * Method used for autonomous driving in DriveStraight auto
	 * @param x	Forward speed for the motors
	 * @param y	Turn speed for the motors
	 */
	public void arcadeDrive(double x, double y)
	{
		driveTrain.setSafetyEnabled(false);
		driveTrain.arcadeDrive(QuickMaths.limit(x, 1), QuickMaths.limit(y, 1));
	}

	/**
	 * Teleop drive command that uses normalized joystick values to control a tank drive
	 */
	public void drive_w_j()
	{
		//Don't interfere with driving in autonomous
		if (!DriverStation.getInstance().isAutonomous()) {
			double leftSpeed = -QuickMaths.normalizeJoystickWithDeadband(joy1.getRawAxis(RobotMap.LEFT_DRIVE_AXIS), 0.05);
			double rightSpeed = -QuickMaths.normalizeJoystickWithDeadband(joy1.getRawAxis(RobotMap.RIGHT_DRIVE_AXIS), 0.05);

			double maxOutput = 1;

			driveTrain.tankDrive(QuickMaths.limit(leftSpeed, maxOutput), QuickMaths.limit(rightSpeed, maxOutput));
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveCommand());
	}
}

