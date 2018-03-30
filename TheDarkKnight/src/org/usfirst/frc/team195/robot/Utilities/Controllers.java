package org.usfirst.frc.team195.robot.Utilities;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Drivers.*;

public class Controllers {
	private Compressor compressor;
	private PowerDistributionPanel powerDistributionPanel;
	private KnightJoystick driveJoystickThrottle;
	private KnightJoystick driveJoystickWheel;
	private KnightJoystick armControlJoystick;
	private KnightJoystick buttonBox1;
	private KnightJoystick buttonBox2;
	private TalonSRX leftDrive1;
	private BaseMotorController leftDrive2;
	private BaseMotorController leftDrive3;
	private TalonSRX rightDrive1;
	private BaseMotorController rightDrive2;
	private BaseMotorController rightDrive3;

	private TalonSRX arm1Motor;

	private TalonSRX elevatorMotorMaster;
	private BaseMotorController elevatorMotorSlave;
	private BaseMotorController elevatorMotorSlave2;
	private BaseMotorController elevatorMotorSlave3;
	private TalonSRX intakeMotor;
	private TalonSRX intake2Motor;
	private TalonSRX climberMotorMaster;
	private TalonSRX climberMotorSlave;

	private ShiftHelper shiftHelper = null;
	private Solenoid intakeSolenoid;
	private Solenoid climberLockSolenoid;

	private CANifier canifierLED;
//	private PigeonDriver pigeonIMU;

//	private DigitalOutput rLED;
//	private DigitalOutput gLED;
//	private DigitalOutput bLED;

	private KnightDigitalInput elevatorHomeSwitch;
	private KnightDigitalInput cubeSensor;

	private NavX navX;
	
	private static Controllers instance = null;
	
	public Controllers() {
		compressor = new Compressor();
		powerDistributionPanel = new PowerDistributionPanel();

		//Drive Joystick Setup
		driveJoystickThrottle = new KnightJoystick(0);
		//driveJoystickWheel = new KnightJoystick(1);
		armControlJoystick = new KnightJoystick(1);
		buttonBox1 = new KnightJoystick(2);
		buttonBox2 = new KnightJoystick(3);

		//Choose whether to create Victor or Talon slaves here
		//Left2Cube Drive Setup
		leftDrive1 = CANSpeedControllerBuilder.createFastMasterTalonSRX(Constants.kLeftDriveMasterId, Constants.kLeftDriveMasterPDPChannel);
		leftDrive2 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kLeftDriveSlaveId, Constants.kLeftDriveSlave1PDPChannel, leftDrive1);
		leftDrive3 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kLeftDriveSlaveId2, Constants.kLeftDriveSlave2PDPChannel, leftDrive1);

		//Right2Cube Drive Setup
		rightDrive1 = CANSpeedControllerBuilder.createFastMasterTalonSRX(Constants.kRightDriveMasterId, Constants.kRightDriveMasterPDPChannel);
		rightDrive2 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kRightDriverSlaveId, Constants.kRightDriveSlave1PDPChannel, rightDrive1);
		rightDrive3 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kRightDriverSlaveId2, Constants.kRightDriveSlave2PDPChannel, rightDrive1);

		//Arm Motor Setup
		arm1Motor = CANSpeedControllerBuilder.createDefaultTalonSRX(Constants.kArmMotorId, Constants.kArmMotorPDPChannel);
		intakeMotor = CANSpeedControllerBuilder.createDefaultTalonSRX(Constants.kIntakeMotorId, Constants.kIntakeMotorPDPChannel);
		intake2Motor = CANSpeedControllerBuilder.createDefaultTalonSRX(Constants.kIntake2MotorId, Constants.kIntake2MotorPDPChannel);

		//Elevator Motor Setup
		elevatorMotorMaster = CANSpeedControllerBuilder.createMasterTalonSRX(Constants.kElevatorMasterId, Constants.kElevatorMasterPDPChannel);
		elevatorMotorSlave = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kElevatorSlaveId, Constants.kElevatorSlavePDPChannel, elevatorMotorMaster);
		elevatorMotorSlave2 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kElevatorSlave2Id, Constants.kElevatorSlave2PDPChannel, elevatorMotorMaster);
		elevatorMotorSlave3 = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kElevatorSlave3Id, Constants.kElevatorSlave3PDPChannel, elevatorMotorMaster);

		//Climber Motor Setup
		climberMotorMaster = CANSpeedControllerBuilder.createMasterTalonSRX(Constants.kClimberMasterId, Constants.kClimberMasterPDPChannel);
		climberMotorSlave = CANSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kClimberSlaveId, Constants.kClimberSlavePDPChannel, climberMotorMaster);

		intakeSolenoid = new Solenoid(Constants.kIntakeSolenoidId);
		climberLockSolenoid = new Solenoid(Constants.kClimberLockSolenoidId);

		//NO SHIFTER THIS YEAR! Do not instantiate; leave set to null
		//shiftHelper = new ShiftHelper(Constants.kShifterSolenoidId, Constants.kShifterSolenoidId2);

		//LED Setup
//		rLED = new DigitalOutput(Constants.kRedLEDId);
//		gLED = new DigitalOutput(Constants.kGreenLEDId);
//		bLED = new DigitalOutput(Constants.kBlueLEDId);

		canifierLED = new CANifier(Constants.kCANifierLEDId);
//		pigeonIMU = new PigeonDriver(2);

		elevatorHomeSwitch = new KnightDigitalInput(Constants.kElevatorHomeSwitchId);
		cubeSensor = new KnightDigitalInput(Constants.kCubeSensorId);

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
	
	public KnightJoystick getDriveJoystickThrottle() {
		return driveJoystickThrottle;
	}

	public KnightJoystick getDriveJoystickWheel() {
		return driveJoystickWheel;
	}

	public KnightJoystick getArmControlJoystick() {
		return armControlJoystick;
	}

	public KnightJoystick getButtonBox1() { return buttonBox1; }

	public KnightJoystick getButtonBox2() { return buttonBox2; }

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

	public TalonSRX getIntake2Motor() {
		return intake2Motor;
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

	public BaseMotorController getElevatorMotorSlave2() {
		return elevatorMotorSlave2;
	}

	public BaseMotorController getElevatorMotorSlave3() {
		return elevatorMotorSlave3;
	}

	public TalonSRX getClimberMotorMaster() {
		return climberMotorMaster;
	}

	public TalonSRX getClimberMotorSlave() {
		return climberMotorSlave;
	}

	public NavX	getNavX() {
		return navX;
	}

	public CANifier getCANifierLED() {
		return canifierLED;
	}

//	public DigitalOutput getRedLED() {
//		return rLED;
//	}
//
//	public DigitalOutput getGreenLED() {
//		return gLED;
//	}
//
//	public DigitalOutput getBlueLED() {
//		return bLED;
//	}

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

	public Solenoid getClimberLockSolenoid() {
		return climberLockSolenoid;
	}

	public KnightDigitalInput getElevatorHomeSwitch() {
		return elevatorHomeSwitch;
	}

	public KnightDigitalInput getCubeSensor() {
		return cubeSensor;
	}

//	public PigeonDriver getPigeonIMU() {
//		return pigeonIMU;
//	}
}