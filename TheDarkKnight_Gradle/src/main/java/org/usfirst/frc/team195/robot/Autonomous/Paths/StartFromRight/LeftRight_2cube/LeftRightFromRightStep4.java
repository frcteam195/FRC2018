package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftRight_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftRightFromRightStep4 implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(225,66,0,0));
        sWaypoints.add(new Waypoint(240,103,15,60));
        sWaypoints.add(new Waypoint(240,130,0,60));
        sWaypoints.add(new Waypoint(240,150,15,60,"PreparePlaceCube"));
        sWaypoints.add(new Waypoint(222,206,0,60));
        sWaypoints.add(new Waypoint(218,206,0,30));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(225, 66), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":225,"y":66},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":240,"y":103},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":240,"y":130},"speed":60,"radius":0,"marker":"","comment":""},{"position":{"x":240,"y":150},"speed":60,"radius":15,"marker":"PreparePlaceCube","comment":""},{"position":{"x":222,"y":206},"speed":60,"radius":0,"marker":"","comment":""},{"position":{"x":218,"y":206},"speed":30,"radius":0,"marker":"","comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: LeftRightFromRightStep4
}