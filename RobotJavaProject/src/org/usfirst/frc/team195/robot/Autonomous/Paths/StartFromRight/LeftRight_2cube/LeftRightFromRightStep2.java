package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftRight_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftRightFromRightStep2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(286,88,0,0));
        sWaypoints.add(new Waypoint(270,76,15,40,"PreparePickupCube"));
        sWaypoints.add(new Waypoint(240,76,15,40));
        sWaypoints.add(new Waypoint(240,200,15,60));
        sWaypoints.add(new Waypoint(235,230,10,40));
        sWaypoints.add(new Waypoint(222,230,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(286, 88), Rotation2d.fromDegrees(0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":286,"y":88},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":270,"y":76},"speed":40,"radius":15,"marker":"PreparePickupCube","comment":""},{"position":{"x":240,"y":76},"speed":40,"radius":15,"marker":"","comment":""},{"position":{"x":240,"y":200},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":235,"y":230},"speed":40,"radius":10,"marker":"","comment":""},{"position":{"x":222,"y":230},"speed":20,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftRightFromRightStep2
}