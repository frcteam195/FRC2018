package org.usfirst.frc.team195.robot.Autonomous.Paths.Fields;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

public abstract class FieldProfile {
	double mRedFrontLeftSwitchToBackWall;
	double mRedBackLeftSwitchToBackWall;
	double mRedLeftSwitchToSideWall;
	double mRedLeftScaleToBackWall;
	double mRedLeftScaleToSideWall;

	double mRedFrontRightSwitchToBackWall;
	double mRedBackRightSwitchToBackWall;
	double mRedRightSwitchToSideWall;
	double mRedRightScaleToBackWall;
	double mRedRightScaleToSideWall;

	double mBlueFrontLeftSwitchToBackWall;
	double mBlueBackLeftSwitchToBackWall;
	double mBlueLeftSwitchToSideWall;
	double mBlueLeftScaleToBackWall;
	double mBlueLeftScaleToSideWall;

	double mBlueFrontRightSwitchToBackWall;
	double mBlueBackRightSwitchToBackWall;
	double mBlueRightSwitchToSideWall;
	double mBlueRightScaleToBackWall;
	double mBlueRightScaleToSideWall;

	double mSideWalltoSideWall;


	public Translation2d getLeftBlueSwitch() {
		return new Translation2d(mBlueBackLeftSwitchToBackWall, mSideWalltoSideWall - mBlueLeftSwitchToSideWall);
	}

	public Translation2d getRightBlueSwitch() {
		return new Translation2d(mBlueBackRightSwitchToBackWall, mBlueRightSwitchToSideWall);
	}

	public Translation2d getLeftBlueScale() {
		return new Translation2d(mBlueLeftScaleToBackWall, mSideWalltoSideWall - mBlueLeftScaleToSideWall);
	}

	public Translation2d getRightBlueScale() {
		return new Translation2d(mBlueRightScaleToBackWall, mBlueRightScaleToSideWall);
	}

	public Translation2d getLeftRedSwitch() {
		return new Translation2d(mRedBackLeftSwitchToBackWall, mSideWalltoSideWall - mRedLeftSwitchToSideWall);
	}

	public Translation2d getRightRedSwitch() {
		return new Translation2d(mRedBackRightSwitchToBackWall, mRedRightSwitchToSideWall);
	}

	public Translation2d getLeftRedScale() {
		return new Translation2d(mRedLeftScaleToBackWall, mSideWalltoSideWall - mRedLeftScaleToSideWall);
	}

	public Translation2d getRightRedScale() {
		return new Translation2d(mRedRightScaleToBackWall, mRedRightScaleToSideWall);
	}

}
