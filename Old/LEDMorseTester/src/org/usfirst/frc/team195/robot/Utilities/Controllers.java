package org.usfirst.frc.team195.robot.Utilities;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
	private BaseMotorController climberMotorSlave;

	private ShiftHelper shiftHelper = null;
	private Solenoid intakeSolenoid;
	private DoubleSolenoid climberLockSolenoid;

	private CANifier canifierLED;

	private DigitalOutput rLED;
	private DigitalOutput gLED;
	private DigitalOutput bLED;

	private KnightDigitalInput elevatorHomeSwitch;
	private KnightDigitalInput cubeSensor;

	private NavX navX;
	
	private static Controllers instance = null;
	
	public Controllers() {
		//LED Setup
		rLED = new DigitalOutput(Constants.kRedLEDId);
		gLED = new DigitalOutput(Constants.kGreenLEDId);
		bLED = new DigitalOutput(Constants.kBlueLEDId);

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

	public BaseMotorController getClimberMotorSlave() {
		return climberMotorSlave;
	}

	public NavX	getNavX() {
		return navX;
	}

	public CANifier getCANifierLED() {
		return canifierLED;
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

	public DoubleSolenoid getClimberLockSolenoid() {
		return climberLockSolenoid;
	}

	public KnightDigitalInput getElevatorHomeSwitch() {
		return elevatorHomeSwitch;
	}

	public KnightDigitalInput getCubeSensor() {
		return cubeSensor;
	}
}