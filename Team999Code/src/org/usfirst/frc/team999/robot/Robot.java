package org.usfirst.frc.team999.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	private static final double kWheelDiameter = 4.875;
	private static final double KSensorUnitsPerRotation = 4096;
	private static final double kRobotWidthInches = 34;
	private static final double kRobotLengthInches = 37;
	private static final int kTimeoutMs = 10;
	public static final double k100msPerMinute = 600.0;
	
	private static final double kIntakeTiltkP = 0.4;
	private static final double kIntakeTiltkI = 0;
	private static final double kIntakeTiltkD = 1;
	private static final double kIntakeTiltkF = 0.966796875;
	private static final double kIntakeTiltAccel = 300;	//Value in RPM
	private static final double kIntakeTiltCruiseVelocity = 150;	//Value in RPM/s
	
	private static final double kElevatorkP = 0.4;
	private static final double kElevatorkI = 0;
	private static final double kElevatorkD = 1;
	private static final double kElevatorkF = 0.399609375;
	private static final double kElevatorAccel = 700;	//Value in RPM
	private static final double kElevatorCruiseVelocity = 375;	//Value in RPM/s
	private static final double kElevatorMaxEnc = 13800;
	
	private FieldPacketReader fieldPacketReader;
	private FieldLayout fieldLayout;
	
	private TalonSRX DriveFrontLeftMotorController = new TalonSRX(1);
	private TalonSRX DriveMiddleLeftMotorController = new TalonSRX(2);
	private TalonSRX DriveBackLeftMotorController = new TalonSRX(3);
	
	private TalonSRX DriveFrontRightMotorController = new TalonSRX(4);
	private TalonSRX DriveMiddleRightMotorController = new TalonSRX(5);
	private TalonSRX DriveBackRightMotorController = new TalonSRX(6);
	
	private TalonSRX ElevatorMotorController = new TalonSRX(14);
	private TalonSRX IntakeTiltMotorController = new TalonSRX(13);
	
	private TalonSRX IntakeLeftMotorController = new TalonSRX(11);
	private TalonSRX IntakeRightMotorController = new TalonSRX(12);
	
	DoubleSolenoid IntakeCylinder = new DoubleSolenoid(0, 1);
	
	private Joystick leftStick;
	private Joystick rightStick;
	private Joystick copilotStick;
	
	private AHRS navX;
	
	private double startTime = 0;
	
	private AutoSwitchModes autoModeRightRightFromRight;

	@Override
	public void robotInit() {
		autoModeRightRightFromRight = AutoSwitchModes.DRIVE;
		
		navX = new AHRS(SPI.Port.kMXP);
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		copilotStick = new Joystick(2);
		
		DriveMiddleLeftMotorController.follow(DriveFrontLeftMotorController);
		DriveBackLeftMotorController.follow(DriveFrontLeftMotorController);
		
		DriveMiddleRightMotorController.follow(DriveFrontRightMotorController);
		DriveBackRightMotorController.follow(DriveFrontRightMotorController);
		
		IntakeTiltMotorController.setInverted(true);
		IntakeTiltMotorController.setSensorPhase(true);
		
		setBrake(true);
		int retryCounter = 0;
		boolean setSucceeded = true;
		do {
			setSucceeded &= DriveFrontLeftMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= ElevatorMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= IntakeTiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs) == ErrorCode.OK;
			
			setSucceeded &= IntakeTiltMotorController.config_kP(0, kIntakeTiltkP, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= IntakeTiltMotorController.config_kI(0, kIntakeTiltkI, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= IntakeTiltMotorController.config_kD(0, kIntakeTiltkD, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= IntakeTiltMotorController.config_kF(0, kIntakeTiltkF, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= IntakeTiltMotorController.configMotionCruiseVelocity(convertNativeUnitsFromRPM(kIntakeTiltCruiseVelocity), kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= IntakeTiltMotorController.configMotionAcceleration(convertNativeUnitsFromRPM(kIntakeTiltAccel), kTimeoutMs) == ErrorCode.OK;
			
			setSucceeded &= ElevatorMotorController.config_kP(0, kIntakeTiltkP, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= ElevatorMotorController.config_kI(0, kIntakeTiltkI, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= ElevatorMotorController.config_kD(0, kIntakeTiltkD, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= ElevatorMotorController.config_kF(0, kIntakeTiltkF, kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= ElevatorMotorController.configMotionCruiseVelocity(convertNativeUnitsFromRPM(kElevatorCruiseVelocity), kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= ElevatorMotorController.configMotionAcceleration(convertNativeUnitsFromRPM(kElevatorAccel), kTimeoutMs) == ErrorCode.OK;
			
		} while (!setSucceeded && retryCounter++ < 3);

		if (!setSucceeded)
			System.out.println("Error configuring encoders!");

		fieldPacketReader = FieldPacketReader.getInstance();
	}

	@Override
	public void autonomousInit() {
		fieldLayout = fieldPacketReader.getFieldConfiguration();
		navX.reset();
		
		boolean setSucceeded = true;
		
		setSucceeded &= DriveFrontLeftMotorController.setSelectedSensorPosition(0, 0, kTimeoutMs) == ErrorCode.OK;
		setSucceeded &= ElevatorMotorController.setSelectedSensorPosition(0, 0, kTimeoutMs) == ErrorCode.OK;
		setSucceeded &= IntakeTiltMotorController.setSelectedSensorPosition(0, 0, kTimeoutMs) == ErrorCode.OK;
		
		if (!setSucceeded)
			System.out.println("Error configuring encoders!");
		
		
	}

	@Override
	public void autonomousPeriodic() {
		switch (fieldLayout) {
			case LEFT_LEFT:
				break;
			case LEFT_RIGHT:
				break;
			case RIGHT_LEFT:
				break;
			case RIGHT_RIGHT:
				rightRightFromRightPeriodic();
				break;
			default:
				break;
		}
	}
	
	private void rightRightFromRightPeriodic() {
		double robotSpeed = 0.5;
		
		
		switch (autoModeRightRightFromRight) {
			case DRIVE:
				DriveFrontLeftMotorController.set(ControlMode.PercentOutput, robotSpeed);
				DriveFrontRightMotorController.set(ControlMode.PercentOutput, -robotSpeed);
				ElevatorMotorController.set(ControlMode.MotionMagic, kElevatorMaxEnc);
				
				if (DriveFrontLeftMotorController.getSelectedSensorPosition(0) >= convertDistanceToNativeEncoderUnits(150))
					autoModeRightRightFromRight = AutoSwitchModes.TURN;
				break;
			case TURN:
				DriveFrontLeftMotorController.set(ControlMode.PercentOutput, -robotSpeed);
				DriveFrontRightMotorController.set(ControlMode.PercentOutput, -robotSpeed);
				if (navX.getYaw() <= -87) {
					autoModeRightRightFromRight = AutoSwitchModes.DRIVE_TO_SWITCH;
					DriveFrontLeftMotorController.set(ControlMode.PercentOutput, 0);
					DriveFrontRightMotorController.set(ControlMode.PercentOutput, 0);
					int retryCounter = 0;
					boolean setSucceeded = true;
					do {
						setSucceeded &= DriveFrontLeftMotorController.setSelectedSensorPosition(0, 0, kTimeoutMs) == ErrorCode.OK;
					} while (!setSucceeded && retryCounter++ < 3);
				}
				break;
			case DRIVE_TO_SWITCH:
				DriveFrontLeftMotorController.set(ControlMode.PercentOutput, robotSpeed);
				DriveFrontRightMotorController.set(ControlMode.PercentOutput, -robotSpeed);
				if (DriveFrontLeftMotorController.getSelectedSensorPosition(0) >= convertDistanceToNativeEncoderUnits(37)) {
					autoModeRightRightFromRight = AutoSwitchModes.LAUNCH;
				}
				break;
			case LAUNCH:
				DriveFrontLeftMotorController.set(ControlMode.PercentOutput, 0);
				DriveFrontRightMotorController.set(ControlMode.PercentOutput, 0);
				
				IntakeTiltMotorController.set(ControlMode.MotionMagic, convertNativeUnitsFromRotations(0.3));
				
				if (Math.abs(IntakeTiltMotorController.getSelectedSensorPosition(0) - convertNativeUnitsFromRotations(0.3)) <= 0.07 && startTime == 0) {
					startTime = Timer.getFPGATimestamp();
					IntakeLeftMotorController.set(ControlMode.PercentOutput, -1);
					IntakeRightMotorController.set(ControlMode.PercentOutput, 1);
				}
				
				if (Timer.getFPGATimestamp() - startTime <= 3) {
					IntakeLeftMotorController.set(ControlMode.PercentOutput, 0);
					IntakeRightMotorController.set(ControlMode.PercentOutput, 0);
					autoModeRightRightFromRight = AutoSwitchModes.DONE;
				}
				break;
			default:
				break;
		}
		
	}
	
	private double convertDistanceToNativeEncoderUnits(double distanceInInches) {
		return distanceInInches * (KSensorUnitsPerRotation / (Math.PI * kWheelDiameter));
	}
	
	private void setBrake(boolean brakeMode) {
		DriveFrontLeftMotorController.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		DriveMiddleLeftMotorController.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		DriveBackLeftMotorController.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		DriveFrontRightMotorController.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		DriveMiddleRightMotorController.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		DriveBackRightMotorController.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
	}
	
	private int convertNativeUnitsFromRPM(double rpm) {
		return (int)(rpm * KSensorUnitsPerRotation / k100msPerMinute);
	}
	
	private double convertNativeUnitsFromRotations(double rotations) {
		return (int)(rotations * KSensorUnitsPerRotation);
	}
	
	@Override
	public void disabledInit() {
		super.disabledInit();
		setBrake(false);
	}

	@Override
	public void teleopPeriodic() {
		double forward = -leftStick.getRawAxis(1);
		double turn = leftStick.getRawAxis(0);
		double leftMotorSpeed = forward + turn;
		double rightMotorSpeed = forward - turn;
		
		DriveFrontLeftMotorController.set(ControlMode.PercentOutput, leftMotorSpeed);
		DriveFrontRightMotorController.set(ControlMode.PercentOutput, rightMotorSpeed);
		
		IntakeTiltMotorController.set(ControlMode.MotionMagic, convertNativeUnitsFromRotations(0)); //Value in rotations
		ElevatorMotorController.set(ControlMode.MotionMagic, convertNativeUnitsFromRotations(0)); //Value in rotations
	}

	@Override
	public void testPeriodic() {
	}
}
