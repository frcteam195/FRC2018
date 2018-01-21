package org.usfirst.frc.team195.robot.Utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;

public class Controllers {
	private KnightJoystick driveJoystick;
	private CANSpeedControllerBuilder canSpeedControllerBuilder;
	private TalonSRX leftDrive1;
	private TalonSRX leftDrive2;
	private TalonSRX leftDrive3;
	private TalonSRX rightDrive1;
	private TalonSRX rightDrive2;
	private TalonSRX rightDrive3;
	private DoubleSolenoid shiftSol;
	
	private TalonSRX liftMotor;
	private VictorSPX liftMotorSlave;
	private TalonSRX intakeMotor;
	private TalonSRX intakeMotorSlave;
	private TalonSRX intakeShoulderMotor;
	private TalonSRX intakeElbowMotor;
	
	private AHRS navX;
	
	private static Controllers instance = null;
	
	public Controllers() {
		//Drive Joystick Setup
		driveJoystick = new KnightJoystick(0);

		canSpeedControllerBuilder = new CANSpeedControllerBuilder();

		//Left Drive Setup
		leftDrive1 = canSpeedControllerBuilder.createDefaultTalonSRX(1);
		leftDrive2 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(2, leftDrive1);
		leftDrive3 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(3, leftDrive1);

		//Right Drive Setup
		rightDrive1 = canSpeedControllerBuilder.createDefaultTalonSRX(4);
		rightDrive2 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(5, rightDrive1);
		rightDrive3 = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(6, rightDrive1);

		//Shift Solenoid Setup
		shiftSol = new DoubleSolenoid(0, 1);

		//Elevator setup
		
		intakeMotor = canSpeedControllerBuilder.createDefaultTalonSRX(7);
		intakeMotorSlave = canSpeedControllerBuilder.createPermanentSlaveTalonSRX(8, intakeMotor);
		/*
		liftMotor = canSpeedControllerBuilder.createDefaultTalonSRX(7);
		liftMotorSlave = canSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(8, liftMotor);
		
		intakeShoulderMotor = canSpeedControllerBuilder.createDefaultTalonSRX(11);
		intakeElbowMotor = canSpeedControllerBuilder.createDefaultTalonSRX(12);
	*/
	    try {
	        navX = new AHRS(Port.kMXP);
	    } catch (Exception ex) {
	        String err_string = "Error instantiating navX MXP:  ";
	        err_string += ex.toString();
	        DriverStation.reportError(err_string, false);
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
	
	public TalonSRX getLeftDrive2() {
		return leftDrive2;
	}
	
	public TalonSRX getLeftDrive3() {
		return leftDrive3;
	}
	
	public TalonSRX getRightDrive1() {
		return rightDrive1;
	}
	
	public TalonSRX getRightDrive2() {
		return rightDrive2;
	}
	
	public TalonSRX getRightDrive3() {
		return rightDrive3;
	}
	
	public DoubleSolenoid getShiftSol() {
		return shiftSol;
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
	
	public TalonSRX getIntakeMotorSlave() {
		return intakeMotorSlave;
	}
	
	public TalonSRX getIntakeShoulderMotor() {
		return intakeShoulderMotor;
	}
	
	public TalonSRX getIntakeElbowMotor() {
		return intakeElbowMotor;
	}

	public AHRS	getNavX() {
		return navX;
	}
}