package com.team319;

import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.io.VelocityOnlyFileSerializer;
import com.team319.trajectory.BobPath;
import com.team319.trajectory.BobPathGenerator;
import com.team319.trajectory.SrxTrajectory;
import com.team319.trajectory.SrxTranslator;
import com.team319.trajectory.SrxTranslatorConfig;

/**
 * Forked from 254's 2014 Trajectory library just a comment to make a change
 * 
 * @author Jared341
 * @author ttremblay
 */
public class Main {

	public static void main(String[] args) {
		//SrxTranslator translator = new SrxTranslator();
		SrxTranslatorConfig standardConfig = new SrxTranslatorConfig();
		
		//Standard configs between all trajectories
		standardConfig.name = "StandardConfig";
		standardConfig.dt = .01;
		standardConfig.max_acc = 60.0;
		standardConfig.max_jerk = 720.0;
		standardConfig.max_vel = 84.0; // gearbob was 6.0
		standardConfig.wheelbase_width_feet = 24.5;
		standardConfig.wheel_dia_inches = 4.88;
		standardConfig.scale_factor = 1; //0.899 // gearbob is 2.35
		standardConfig.encoder_ticks_per_rev = 4096;
		
		SrxTranslatorConfig slowConfig = new SrxTranslatorConfig(standardConfig);
		slowConfig.max_vel = 4.0;
		
		
		
		BobPath ThreeFeet = new BobPath(standardConfig, "ThreeFeet", 1);
		ThreeFeet.addWaypoint(new WaypointSequence.Waypoint(0, 0, 0));
		ThreeFeet.addWaypoint(new WaypointSequence.Waypoint(48, -36, Math.toRadians(-10.0)));
		ThreeFeet.addWaypoint(new WaypointSequence.Waypoint(96, 0, Math.toRadians(30.0)));
		ThreeFeet.addWaypoint(new WaypointSequence.Waypoint(120, 24, Math.toRadians(10.0)));
		
		BobPath FourFeet = new BobPath(standardConfig, "FourFeet", 1);
		FourFeet.addWaypoint(new WaypointSequence.Waypoint(0, 0, 0));
		FourFeet.addWaypoint(new WaypointSequence.Waypoint(4.0, 4.0, Math.toRadians(15.0)));
		FourFeet.addWaypoint(new WaypointSequence.Waypoint(8.0, 5.0, Math.toRadians(0.0)));
		FourFeet.addWaypoint(new WaypointSequence.Waypoint(8.0, 5.0, Math.toRadians(0.0)));
		
//		BobPath OneFoot = new BobPath(standardConfig, "OneFoot", 1);
//		OneFoot.addWaypoint(new WaypointSequence.Waypoint(0, 0, 0));
//		OneFoot.addWaypoint(new WaypointSequence.Waypoint(1.0, 0.0, Math.toRadians(0.0)));
//		
		SrxTranslator sx = new SrxTranslator();
		SrxTrajectory st = sx.getSrxTrajectoryFromChezyPath(BobPathGenerator.makePath(ThreeFeet), standardConfig);
		System.out.println(st.leftProfile.getTalonTrajectory("Left"));
		System.out.println(st.rightProfile.getTalonTrajectory("Right"));

				BobPathGenerator.exportPath("Paths", ThreeFeet);
				//BobPathGenerator.exportPath("Paths", FourFeet);
				
				while (true) {;}
		
		//BobPathGenerator.appendAndExportPaths("Paths", "appendedPath", false, blueHopperThenShootAutoLeftSidePt2, toAppend);
		//BobPathGenerator.appendAndExportPaths("Paths", "appendedAndFlippedPath", true, blueHopperThenShootAutoLeftSidePt2, toAppend); 
		//redGear.exportPathWithSerializer(new VelocityOnlyFileSerializer(), "Paths");
	}
}
