package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromLeftStep1 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,276,0,0));
        sWaypoints.add(new Waypoint(60,276,0,130));
        sWaypoints.add(new Waypoint(180,276,45,130,"PreparePlaceCube"));
        sWaypoints.add(new Waypoint(286,240,0,100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 276), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":276},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":60,"y":276},"speed":130,"radius":0,"marker":"","comment":""},{"position":{"x":180,"y":276},"speed":130,"radius":45,"marker":"PreparePlaceCube","comment":""},{"position":{"x":286,"y":240},"speed":100,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: LeftLeftFromLeftStep1
}