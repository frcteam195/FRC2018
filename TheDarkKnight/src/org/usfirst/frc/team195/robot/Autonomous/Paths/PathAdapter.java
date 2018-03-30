package org.usfirst.frc.team195.robot.Autonomous.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Autonomous.Paths.Fields.*;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

import java.sql.Driver;

public class PathAdapter {
	private static final DriverStation ds = DriverStation.getInstance();

	private static FieldProfile kReferenceField = new ReferenceField();
	private static FieldProfile kCurrentField = new PracticeField();

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
			input.setPosition(input.getPosition().translateBy(leftBlueSwitchTransform));
		else
			input.setPosition(input.getPosition().translateBy(leftRedSwitchTransform));

		return input;
	}

	public static Waypoint getAdaptedRightSwitchWaypoint(Waypoint input) {
		if (ds.getAlliance() == DriverStation.Alliance.Blue)
			input.setPosition(input.getPosition().translateBy(rightBlueSwitchTransform));
		else
			input.setPosition(input.getPosition().translateBy(rightRedSwitchTransform));

		return input;
	}

	public static Waypoint getAdaptedLeftScaleWaypoint(Waypoint input) {
		if (ds.getAlliance() == DriverStation.Alliance.Blue)
			input.setPosition(input.getPosition().translateBy(leftBlueScaleTransform));
		else
			input.setPosition(input.getPosition().translateBy(leftRedScaleTransform));

		return input;
	}

	public static Waypoint getAdaptedRightScaleWaypoint(Waypoint input) {
		if (ds.getAlliance() == DriverStation.Alliance.Blue)
			input.setPosition(input.getPosition().translateBy(rightBlueScaleTransform));
		else
			input.setPosition(input.getPosition().translateBy(rightRedScaleTransform));

		return input;
	}


}
