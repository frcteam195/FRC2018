/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team195.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.usfirst.frc.team195.robot.CyberPathSRXUtils.MPArcCapableRobotMap;
import org.usfirst.frc.team195.robot.CyberPathSRXUtils.TalonHelper;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final int frontLeftTalonID = 1;
	public static final int backRightTalonID = 2;
	public static final int backLeftTalonID = 3;
	public static final int frontRightTalonID = 4;
	public static final int intakeLeftTalonID = 5;
	public static final int intakeRightTalonID = 6;

	public static final int LEFT_DRIVE_AXIS = 1;
	public static final int RIGHT_DRIVE_AXIS = 5;

	public static final int kTimeoutMs = 10;
	public static final int kTalonRetryCount = 3;

	public static DifferentialDrive driveTrain;
	public static WPI_TalonSRX frontRightDrive;
	public static WPI_TalonSRX backRightDrive;
	public static WPI_TalonSRX frontLeftDrive;
	public static WPI_TalonSRX backLeftDrive;
	public static WPI_TalonSRX intakeMaster;
	public static WPI_TalonSRX intakeSlave;

	public static PigeonIMU pidgey;


	public void init() {
		// initialize

		// Drive Train Motors and Motor Groups
		//Setup sensors and motor controllers to support motion profiling as well
		//Configure for brake mode to make positional control more accurate
		frontRightDrive = new WPI_TalonSRX(frontRightTalonID);
		frontRightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
		frontRightDrive.setSensorPhase(false); /* keep sensor and motor in phase */
		frontRightDrive.setNeutralMode(NeutralMode.Brake);

		backRightDrive = new WPI_TalonSRX(backRightTalonID);
		backRightDrive.setNeutralMode(NeutralMode.Brake);

		frontLeftDrive = new WPI_TalonSRX(frontLeftTalonID);
		frontLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
		frontLeftDrive.setSensorPhase(true); /* keep sensor and motor in phase */
		frontLeftDrive.setNeutralMode(NeutralMode.Brake);

		backLeftDrive = new WPI_TalonSRX(backLeftTalonID);
		backLeftDrive.setNeutralMode(NeutralMode.Brake);

		//Create intake motors and set to brake to help hold object
		intakeMaster = new WPI_TalonSRX(intakeLeftTalonID);
		intakeMaster.setNeutralMode(NeutralMode.Brake);
		intakeSlave = new WPI_TalonSRX(intakeRightTalonID);
		intakeSlave.setInverted(true);
		intakeSlave.follow(intakeMaster);
		intakeSlave.setNeutralMode(NeutralMode.Brake);


		//Need to setup master -> slave mode for motion profiling
		backLeftDrive.follow(frontLeftDrive);
		backRightDrive.follow(frontRightDrive);

		driveTrain = new DifferentialDrive(frontLeftDrive, frontRightDrive);

		//Get the gyro from the backRightDrive TalonSRX
		pidgey = new PigeonIMU(backRightDrive);


		TalonHelper.configForMotionProfileArc(frontRightDrive, frontLeftDrive, backRightDrive);
		//Enter all slave motor controllers that do not have sensors attached into this method
		//This method wlil slow down the unused CAN frames
		//DO NOT PUT THE PIGEON TALON IN THIS LIST OR ANY TALON WITH AN ENCODER
		TalonHelper.setLightweightSlaves(backLeftDrive, intakeSlave);
	}

}
