package org.usfirst.frc.team195.robot.Utilities;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Drivers.CANSpeedControllerBuilder;
import org.usfirst.frc.team195.robot.Utilities.Drivers.NavX;

public class Controllers {
	private Compressor compressor;
	private PowerDistributionPanel powerDistributionPanel;
	private KnightJoystick driveJoystick;
	private CANSpeedControllerBuilder canSpeedControllerBuilder;
	private TalonSRX leftDrive1;
	private BaseMotorController leftDrive2;
	private BaseMotorController leftDrive3;
	private TalonSRX rightDrive1;
	private BaseMotorController rightDrive2;
	private BaseMotorController rightDrive3;

	private TalonSRX arm1Motor;
	private TalonSRX arm2Motor;
	
	private TalonSRX elevatorMotorMaster;
	private BaseMotorController elevatorMotorSlave;
	private TalonSRX intakeMotor;
	private TalonSRX climberMotorMaster;
	private BaseMotorController climberMotorSlave;

	private ShiftHelper shiftHelper = null;
	private Solenoid intakeSolenoid;
	private DoubleSolenoid climberLockSolenoid;

	private DigitalOutput rLED;
	private DigitalOutput gLED;
	private DigitalOutput bLED;

	private NavX navX;
	
	private static Controllers instance = null;
	
	public Controllers() {
		compressor = new Compressor();
		powerDistributionPanel = new PowerDistributionPanel();

		//Drive Joystick Setup
		driveJoystick = new KnightJoystick(0);

		canSpeedControllerBuilder = new CANSpeedControllerBuilder();

		//Choose whether to create Victor or Talon slaves here
		//Left Drive Setup
		leftDrive1 = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kLeftDriveMasterId, Constants.kLeftDriveMasterPDPChannel);
		leftDrive2 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kLeftDriveSlaveId, Constants.kLeftDriveSlave1PDPChannel, leftDrive1);
		leftDrive3 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kLeftDriveSlaveId2, Constants.kLeftDriveSlave2PDPChannel, leftDrive1);

		//Right Drive Setup
		rightDrive1 = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kRightDriveMasterId, Constants.kRightDriveMasterPDPChannel);
		rightDrive2 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kRightDriverSlaveId, Constants.kRightDriveSlave1PDPChannel, rightDrive1);
		rightDrive3 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kRightDriverSlaveId2, Constants.kRightDriveSlave2PDPChannel, rightDrive1);

		//Arm Motor Setup
		arm1Motor = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kArm1MotorId, Constants.kArm1MotorPDPChannel);
		arm2Motor = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kArm2MotorId, Constants.kArm2MotorPDPChannel);
//		intakeMotor = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kIntakeMotorId, Constants.kIntakeMotorPDPChannel);
//
//		//Elevator Motor Setup
//		elevatorMotorMaster = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kElevatorMasterId, Constants.kElevatorMasterPDPChannel);
//		elevatorMotorSlave = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kElevatorSlaveId, Constants.kElevatorSlavePDPChannel, elevatorMotorMaster);
//
//		//Climber Motor Setup
//		climberMotorMaster = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kClimberMasterId, Constants.kClimberMasterPDPChannel);
//		climberMotorSlave = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kClimberSlaveId, Constants.kClimberSlavePDPChannel, climberMotorMaster);

		intakeSolenoid = new Solenoid(Constants.kIntakeSolenoidId);
		climberLockSolenoid = new DoubleSolenoid(Constants.kClimberLockSolenoidId1, Constants.kClimberLockSolenoidId2);

		//NO SHIFTER THIS YEAR! Do not instantiate; leave set to null
		//shiftHelper = new ShiftHelper(Constants.kShifterSolenoidId, Constants.kShifterSolenoidId2);

		//LED Setup
		rLED = new DigitalOutput(Constants.kRedLEDId);
		gLED = new DigitalOutput(Constants.kGreenLEDId);
		bLED = new DigitalOutput(Constants.kBlueLEDId);

	    try {
	        navX = new NavX(SPI.Port.kMXP);
	    } catch (Exception ex) {
			ConsoleReporter.report(ex, MessageLevel.DEFCON1);
	    }
	}
	
	public static Controllers getInstance() {
		if(instance == null)
			instance = new Controllers();
		
		return instance;
	}
	
	public KnightJoystick getDriveJoystick() {
		return driveJoystick;
	}
	
	public TalonSRX getLeftDrive1() {
		return leftDrive1;
	}
	
	public BaseMotorController getLeftDrive2() {
		return leftDrive2;
	}
	
	public BaseMotorController getLeftDrive3() {
		return leftDrive3;
	}
	
	public TalonSRX getRightDrive1() {
		return rightDrive1;
	}
	
	public BaseMotorController getRightDrive2() {
		return rightDrive2;
	}
	
	public BaseMotorController getRightDrive3() {
		return rightDrive3;
	}

	public TalonSRX getArm1Motor() {
		return arm1Motor;
	}

	public TalonSRX getArm2Motor() {
		return arm2Motor;
	}

	public TalonSRX getIntakeMotor() {
		return intakeMotor;
	}

	public TalonSRX getElevatorMotorMaster() {
		return elevatorMotorMaster;
	}
	
	public BaseMotorController getElevatorMotorSlave() {
		return elevatorMotorSlave;
	}

	public TalonSRX getClimberMotorMaster() {
		return climberMotorMaster;
	}

	public BaseMotorController getClimberMotorSlave() {
		return climberMotorSlave;
	}

	public NavX	getNavX() {
		return navX;
	}

	public DigitalOutput getRedLED() {
		return rLED;
	}

	public DigitalOutput getGreenLED() {
		return gLED;
	}

	public DigitalOutput getBlueLED() {
		return bLED;
	}

	public PowerDistributionPanel getPowerDistributionPanel() {
		return powerDistributionPanel;
	}

	public Compressor getCompressor() {
		return compressor;
	}

	public ShiftHelper getShiftHelper() {
		return shiftHelper;
	}

	public Solenoid getIntakeSolenoid() {
		return intakeSolenoid;
	}

	public Solenoid getGinoSol() {
		return ginoSol;
	}
}