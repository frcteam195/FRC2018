package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep1 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,48,0,0));
        sWaypoints.add(new Waypoint(70,48,15,80));
        sWaypoints.add(new Waypoint(120,55,15,80));
        sWaypoints.add(new Waypoint(168,58,20,60));
        sWaypoints.add(new Waypoint(168,30,0,30));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 48), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":48},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":70,"y":48},"speed":130,"radius":15,"marker":"","comment":""},{"position":{"x":120,"y":64},"speed":130,"radius":15,"marker":"","comment":""},{"position":{"x":168,"y":64},"speed":100,"radius":10,"marker":"","comment":""},{"position":{"x":168,"y":50},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightLeftFromRightStep1
}