package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromLeftStep2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(286,248,0,0));
        sWaypoints.add(new Waypoint(257,252,15,60));
        sWaypoints.add(new Waypoint(228,244,0,30));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(286, 248), Rotation2d.fromDegrees(0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":286,"y":248},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":257,"y":252},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":228,"y":244},"speed":30,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftLeftFromLeftStep2
}