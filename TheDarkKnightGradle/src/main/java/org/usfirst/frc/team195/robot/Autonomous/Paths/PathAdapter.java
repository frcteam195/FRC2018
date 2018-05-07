package org.usfirst.frc.team195.robot.Autonomous.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Autonomous.Paths.Fields.*;
import org.usfirst.frc.team195.robot.Autonomous.Paths.Robots.CompBot;
import org.usfirst.frc.team195.robot.Autonomous.Paths.Robots.PracticeBot;
import org.usfirst.frc.team195.robot.Autonomous.Paths.Robots.RobotProfile;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

public class PathAdapter {
	private static final DriverStation ds = DriverStation.getInstance();

	private static FieldProfile kReferenceField = new ReferenceField();
	private static FieldProfile kCurrentField = new EnergyEinsteinField();

	//TODO: Change inversion of values to be correct for near or far, not left or right. Is not correct now
	private static RobotProfile kRobotProfile = new CompBot();

	private static Translation2d leftBlueSwitchTransform = kCurrentField.getLeftBlueSwitch().translateBy(kReferenceField.getLeftBlueSwitch().inverse());
	private static Translation2d rightBlueSwitchTransform = kCurrentField.getRightBlueSwitch().translateBy(kReferenceField.getRightBlueSwitch().inverse());
	private static Translation2d leftBlueScaleTransform = kCurrentField.getLeftBlueScale().translateBy(kReferenceField.getLeftBlueScale().inverse());
	private static Translation2d rightBlueScaleTransform = kCurrentField.getRightBlueScale().translateBy(kReferenceField.getRightBlueScale().inverse());

	private static Translation2d leftRedSwitchTransform = kCurrentField.getLeftRedSwitch().translateBy(kReferenceField.getLeftRedSwitch().inverse());
	private static Translation2d rightRedSwitchTransform = kCurrentField.getRightRedSwitch().translateBy(kReferenceField.getRightRedSwitch().inverse());
	private static Translation2d leftRedScaleTransform = kCurrentField.getLeftRedScale().translateBy(kReferenceField.getLeftRedScale().inverse());
	private static Translation2d rightRedScaleTransform = kCurrentField.getRightRedScale().translateBy(kReferenceField.getRightRedScale().inverse());

	public static Waypoint getAdaptedLeftSwitchWaypoint(Waypoint input) {
		if (ds.getAlliance() == DriverStation.Alliance.Blue)
			input.setPosition(input.getPosition().translateBy(leftBlueSwitchTransform).translateBy(kRobotProfile.getTransform()));
		else
			input.setPosition(input.getPosition().translateBy(leftRedSwitchTransform).translateBy(kRobotProfile.getTransform()));

		return input;
	}

	public static Waypoint getAdaptedRightSwitchWaypoint(Waypoint input) {
		if (ds.getAlliance() == DriverStation.Alliance.Blue)
			input.setPosition(input.getPosition().translateBy(rightBlueSwitchTransform).translateBy(kRobotProfile.getTransform().invertY()));
		else
			input.setPosition(input.getPosition().translateBy(rightRedSwitchTransform).translateBy(kRobotProfile.getTransform().invertY()));

		return input;
	}

	public static Waypoint getAdaptedLeftScaleWaypoint(Waypoint input) {
		if (ds.getAlliance() == DriverStation.Alliance.Blue)
			input.setPosition(input.getPosition().translateBy(leftBlueScaleTransform).translateBy(kRobotProfile.getTransform()));
		else
			input.setPosition(input.getPosition().translateBy(leftRedScaleTransform).translateBy(kRobotProfile.getTransform()));

		return input;
	}

	public static Waypoint getAdaptedRightScaleWaypoint(Waypoint input) {
		if (ds.getAlliance() == DriverStation.Alliance.Blue)
			input.setPosition(input.getPosition().translateBy(rightBlueScaleTransform).translateBy(kRobotProfile.getTransform().invertY()));
		else
			input.setPosition(input.getPosition().translateBy(rightRedScaleTransform).translateBy(kRobotProfile.getTransform().invertY()));

		return input;
	}

	public static double getAdaptedWheelDiameter() {
		return kRobotProfile.getWheelDiameter();
	}

}
