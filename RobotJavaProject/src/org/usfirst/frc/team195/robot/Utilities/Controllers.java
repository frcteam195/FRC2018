package org.usfirst.frc.team195.robot.Utilities;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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
	private Solenoid shiftSol;
	private Solenoid ginoSol;

	private DigitalOutput rLED;
	private DigitalOutput gLED;
	private DigitalOutput bLED;
	
	private TalonSRX liftMotor;
	private VictorSPX liftMotorSlave;
	private TalonSRX intakeMotor;
	private TalonSRX intakeMotor2;
	private TalonSRX intakeShoulderMotor;
	private TalonSRX intakeElbowMotor;
	
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
		leftDrive1 = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kLeftDriveMasterId);
		leftDrive2 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kLeftDriveSlaveId, Constants.kLeftDriveSlave1PDPChannel, leftDrive1);
		leftDrive3 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kLeftDriveSlaveId2, Constants.kLeftDriveSlave2PDPChannel, leftDrive1);

		//Right Drive Setup
		rightDrive1 = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kRightDriveMasterId);
		rightDrive2 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kRightDriverSlaveId, Constants.kRightDriveSlave1PDPChannel, rightDrive1);
		rightDrive3 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(Constants.kRightDriverSlaveId2, Constants.kRightDriveSlave2PDPChannel, rightDrive1);

		//Shift Solenoid Setup
		shiftSol = new Solenoid(Constants.kShifterSolenoidId);
		ginoSol = new Solenoid(Constants.kShifterSolenoidId2);

		//LED Setup
		rLED = new DigitalOutput(Constants.kRedLEDId);
		gLED = new DigitalOutput(Constants.kGreenLEDId);
		bLED = new DigitalOutput(Constants.kBlueLEDId);

		//Elevator setup
		
		//intakeMotor = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kIntakeLeftId);
		//intakeMotor2 = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kIntakeRightId);

//		liftMotor = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kElevatorMasterId);
//		liftMotorSlave = canSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(Constants.kElevatorSlaveId, liftMotor);
//
//		intakeShoulderMotor = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kShoulderMotorId);
//		intakeElbowMotor = canSpeedControllerBuilder.createDefaultTalonSRX(Constants.kElbowMotorId);

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
	
	public Solenoid getShiftSol() {
		return shiftSol;
	}
	
	public Solenoid getGinoSol() {
		return ginoSol;
	}

	public TalonSRX getLiftMotor() {
		return liftMotor;
	}
	
	public VictorSPX getLiftMotorSlave() {
		return liftMotorSlave;
	}
	
	public TalonSRX getIntakeMotor() {
		return intakeMotor;
	}
	
	public TalonSRX getIntakeMotor2() {
		return intakeMotor2;
	}
	
	public TalonSRX getIntakeShoulderMotor() {
		return intakeShoulderMotor;
	}
	
	public TalonSRX getIntakeElbowMotor() {
		return intakeElbowMotor;
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
}