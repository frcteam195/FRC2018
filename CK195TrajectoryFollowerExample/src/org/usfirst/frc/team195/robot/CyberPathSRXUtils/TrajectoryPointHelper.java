package org.usfirst.frc.team195.robot.CyberPathSRXUtils;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import edu.wpi.first.wpilibj.DriverStation;

public class TrajectoryPointHelper {

	public static TrajectoryDuration valueOf(int val) {
		TrajectoryDuration[] vals = TrajectoryDuration.values();

		for(int i = 0; i < vals.length; ++i) {
			TrajectoryDuration td = vals[i];
			if (td.value == val) {
				return td;
			}
		}

		DriverStation.reportError("Invalid Motion Profile Point Time. Defaulting, but not accurate.", true);
		System.out.println("Invalid Motion Profile Point Time. Defaulting, but not accurate.");
		return TrajectoryDuration.Trajectory_Duration_100ms;
	}
}
