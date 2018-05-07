package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep4 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(146,44,0,0));
        sWaypoints.add(new Waypoint(210,52,15,100));
        sWaypoints.add(new Waypoint(244,73,15,120));
        sWaypoints.add(new Waypoint(244,200,15,100, "LowerArm"));
        sWaypoints.add(new Waypoint(238,230,10,80));
        sWaypoints.add(new Waypoint(222,230,0,40));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(146, 44), Rotation2d.fromDegrees(0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":146,"y":44},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":210,"y":52},"speed":100,"radius":15,"marker":"","comment":""},{"position":{"x":244,"y":73},"speed":100,"radius":15,"marker":"","comment":""},{"position":{"x":244,"y":200},"speed":100,"radius":15,"marker":"","comment":""},{"position":{"x":238,"y":230},"speed":80,"radius":10,"marker":"","comment":""},{"position":{"x":222,"y":230},"speed":40,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightLeftFromRightStep4
}