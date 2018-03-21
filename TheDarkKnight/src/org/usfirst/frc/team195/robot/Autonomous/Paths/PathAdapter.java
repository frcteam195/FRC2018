package org.usfirst.frc.team195.robot.Autonomous.Paths;

import org.usfirst.frc.team195.robot.Autonomous.Paths.Fields.CompetitionTheoreticalField;
import org.usfirst.frc.team195.robot.Autonomous.Paths.Fields.FieldProfile;
import org.usfirst.frc.team195.robot.Autonomous.Paths.Fields.PracticeField;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

public class PathAdapter {
	private static FieldProfile kReferenceField = new PracticeField();
	private static FieldProfile kCurrentField = new CompetitionTheoreticalField();

	private static double switchXTransform = kCurrentField.getSwitchX() - kReferenceField.getSwitchX();
	private static double scaleXTransform = kCurrentField.getScaleX() - kReferenceField.getScaleX();

	public static Waypoint getAdaptedSwitchWaypoint(Waypoint input) {
		input.setPosition(input.getPosition().translateBy(new Translation2d(switchXTransform, 0)));
		return input;
	}

	public static Waypoint getAdaptedScaleWaypoint(Waypoint input) {
		input.setPosition(input.getPosition().translateBy(new Translation2d(scaleXTransform, 0)));
		return input;
	}
}
