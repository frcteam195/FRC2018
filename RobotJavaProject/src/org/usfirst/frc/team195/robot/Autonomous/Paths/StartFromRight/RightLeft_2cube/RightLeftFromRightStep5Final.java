package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep5Final implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(222,230,0,0));
        sWaypoints.add(new Waypoint(240,230,15,60));
        sWaypoints.add(new Waypoint(263,255,15,60,"PreparePlaceCube"));
        sWaypoints.add(new Waypoint(294,238,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(222, 230), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":222,"y":230},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":240,"y":230},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":263,"y":255},"speed":60,"radius":15,"marker":"PreparePlaceCube","comment":""},{"position":{"x":294,"y":238},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightLeftFromRightStep5Final
}