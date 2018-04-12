package org.usfirst.frc.team195.robot.CyberPathSRXUtils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team195.robot.RobotMap;

public class TalonHelper {

	public static boolean setPIDFGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF, int iZone, int timeout) {
		boolean setSucceeded = true;
		int retryCounter = 0;
		if (timeout > 0) {
			do {
				setSucceeded = true;

				setSucceeded &= talon.config_kP(slotID, kP, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_kI(slotID, kI, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_kD(slotID, kD, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_kF(slotID, kF, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_IntegralZone(slotID, iZone, timeout) == ErrorCode.OK;

			} while (!setSucceeded && retryCounter++ < RobotMap.kTalonRetryCount);
		} else {
			talon.config_kP(slotID, kP, timeout);
			talon.config_kI(slotID, kI, timeout);
			talon.config_kD(slotID, kD, timeout);
			talon.config_kF(slotID, kF, timeout);
			talon.config_IntegralZone(slotID, iZone, timeout);
		}
		return retryCounter < RobotMap.kTalonRetryCount && setSucceeded;
	}

	/**
	 * Configure motion profile arc talons
	 * @param masterArcTalon	The master controller for the arc mode, usually right side master
	 * @param otherDriveTalon	The other drive controller with an encoder, usually left side master
	 * @param pigeonTalon		The talon that has the PigeonIMU sensor connected
	 * @return	Returns true if all commands did not receive errors
	 */
	public static boolean configForMotionProfileArc(TalonSRX masterArcTalon, TalonSRX otherDriveTalon, TalonSRX pigeonTalon) {
		boolean setSucceeded = true;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			//Setup sensor summing and angle calculations for motion profiling
			setSucceeded &= masterArcTalon.configRemoteFeedbackFilter(otherDriveTalon.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, 0, RobotMap.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= masterArcTalon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= masterArcTalon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, RobotMap.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= masterArcTalon.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 0, RobotMap.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= masterArcTalon.configSelectedFeedbackCoefficient(0.5, 0, RobotMap.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= masterArcTalon.configRemoteFeedbackFilter(pigeonTalon.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 1, RobotMap.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= masterArcTalon.configSelectedFeedbackCoefficient(3600.0/8192.0, 1, RobotMap.kTimeoutMs) == ErrorCode.OK;

		} while (!setSucceeded && retryCounter++ < RobotMap.kTalonRetryCount);

		setSucceeded &= setPIDFGains(masterArcTalon, TrajectoryGainConstants.driveProfileSlot, TrajectoryGainConstants.drivekP, TrajectoryGainConstants.drivekI, TrajectoryGainConstants.drivekD, TrajectoryGainConstants.driveF, TrajectoryGainConstants.driveIZone, RobotMap.kTimeoutMs);
		setSucceeded &= setPIDFGains(masterArcTalon, TrajectoryGainConstants.turnProfileSlot, TrajectoryGainConstants.turnkP, TrajectoryGainConstants.turnkI, TrajectoryGainConstants.turnkD, TrajectoryGainConstants.turnF, TrajectoryGainConstants.turnIZone, RobotMap.kTimeoutMs);

		setSucceeded &= setPIDFGains(otherDriveTalon, TrajectoryGainConstants.driveProfileSlot, TrajectoryGainConstants.drivekP, TrajectoryGainConstants.drivekI, TrajectoryGainConstants.drivekD, TrajectoryGainConstants.driveF, TrajectoryGainConstants.driveIZone, RobotMap.kTimeoutMs);
		setSucceeded &= setPIDFGains(otherDriveTalon, TrajectoryGainConstants.turnProfileSlot, TrajectoryGainConstants.turnkP, TrajectoryGainConstants.turnkI, TrajectoryGainConstants.turnkD, TrajectoryGainConstants.turnF, TrajectoryGainConstants.turnIZone, RobotMap.kTimeoutMs);


		return retryCounter < RobotMap.kTalonRetryCount && setSucceeded;
	}

	/**
	 * Pass all slave controllers into this except the ones that contain sensors.
	 * This method will open up more CAN BUS bandwidth by slowing down rate of transmission of unused CAN frames
	 * @param motorControllers The list of slaves to adjust frame rate on
	 */
	public static void setLightweightSlaves(IMotorController...motorControllers) {
		for (IMotorController motorController: motorControllers) {
			motorController.setStatusFramePeriod(StatusFrame.Status_1_General, 100, 10);
			motorController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100, 10);
		}
	}

}
