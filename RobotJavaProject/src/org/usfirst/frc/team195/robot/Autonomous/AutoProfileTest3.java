package org.usfirst.frc.team195.robot.Autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystemOld;
import org.usfirst.frc.team195.robot.Utilities.CustomAuto;
import org.usfirst.frc.team195.robot.Utilities.SplineMotion.CyberPath;
import org.usfirst.frc.team195.robot.Utilities.SplineMotion.SRX.SRXDriveBaseTrajectory;

@Deprecated
public class AutoProfileTest3 implements CustomAuto {
	private DriveBaseSubsystemOld driveBaseSubsystem;
	private CyberPath cp;
	private SRXDriveBaseTrajectory preprocessedPoints;

	 public AutoProfileTest3() {
	 	driveBaseSubsystem = DriveBaseSubsystemOld.getInstance();
	 	cp = new CyberPath();
	 	cp.addPoint(0, 0, 0);
	 	cp.addPoint(18, 0, 0);
	 	cp.addPoint(65, -36, 0);
	 	cp.addPoint(80, -36, 0);

		preprocessedPoints = cp.getSRXTrajectory();
		if (preprocessedPoints.getLeftWheelTrajectory().length > 350 || preprocessedPoints.getRightWheelTrajectory().length > 350)
			ConsoleReporter.report("Trajectory too large!", MessageLevel.DEFCON1);
		ConsoleReporter.report("Finished generating auto trajectory!", MessageLevel.INFO);
	}

	@Override
	public void start() {
		driveBaseSubsystem.setMotionProfileTrajectory(preprocessedPoints);
		driveBaseSubsystem.setControlMode(ControlMode.MotionProfile);
		driveBaseSubsystem.setBrakeMode(true);
		driveBaseSubsystem.setGear(false);
		driveBaseSubsystem.startMPTrajectory();
	}

	private String printTalonTrajectory(String side, double[][] points) {
		String retVal = "public static double[][] kMotionProfile"+side+" = new double[][] {";
		for (int i = 0; i < points.length; i++) {
			retVal += "{" + points[i][0] + ",\t" + points[i][1] + ",\t" + points[i][2] + "},\n";
		}
		retVal += "};";
		return retVal;
	}

}
