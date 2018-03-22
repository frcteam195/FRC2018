import java.util.ArrayList;

public class PointFinder {
	private double minA1Angle = 0;
	private double maxA1Angle = 180;
	private double minA2Angle = -160;
	private double maxA2Angle = 160;
	private double kArm1Length = 8.25;
	private double kArm2Length = 8.5;

	private double acceptableRDeviation = 0.5;
	private double acceptableThetaDeviation = 2;

	private ArrayList<PresetPoint> presetPoints;

	public PointFinder() {
		presetPoints = new ArrayList<PresetPoint>();
		presetPoints.add(new PresetPoint(new PolarCoordinate(4, 50), new ArmConfiguration(20, 30)));
	}

	public ArmConfiguration getArmConfiguration(PolarCoordinate polarCoordinate, boolean invertArm) {

		for (PresetPoint p : presetPoints) {
			if (p.polarCoordinate.compareTo(polarCoordinate) == 0)
				return p.armConfiguration;
		}

		double c = 0;
		double r = 0;
		double alpha = 0;
		boolean solutionFound = false;

		for (double aTmp = maxA2Angle; aTmp >= minA2Angle; aTmp -= 0.25) {
			c = aTmp > 0 ? 180 - aTmp : 180 + aTmp;
			r = Math.sqrt(Math.pow(kArm1Length, 2) + Math.pow(kArm2Length, 2)
					- 2 * kArm1Length * kArm2Length * Math.cos(Math.toRadians(c)));
			if (Math.abs(polarCoordinate.r - r) < acceptableRDeviation) {
				alpha = aTmp;
				solutionFound = true;
				//ConsoleReporter.report("Solution Found for radius!");
				break;
			}
		}

		if (!solutionFound) {
			return null;
		}


		boolean prevInvertArmCal = invertArm;

		do {
			if (invertArm) {
				alpha = -alpha;
				c = -c;
			}
			solutionFound = false;

			double theta = 0;
			double phi = 0;
			double b = Math.toDegrees(Math.asin(Math.sin(Math.toRadians(c)) * kArm2Length / r));
//			ConsoleReporter.report("B: " + b);
			for (double pTmp = maxA1Angle; pTmp >= minA1Angle; pTmp -= 0.5) {
				theta = pTmp + b;
//				ConsoleReporter.report("Phi: " + pTmp);
//				ConsoleReporter.report("Desired Angle: " + polarCoordinate.theta);
//				ConsoleReporter.report("Actual Angle: " + theta);
				if (Math.abs(polarCoordinate.theta - theta) < acceptableThetaDeviation) {
					phi = pTmp;
					solutionFound = true;
					//ConsoleReporter.report("Solution Found for theta!");
					break;
				}
			}
			//ConsoleReporter.report("Time elapsed: " + (Timer.getFPGATimestamp() - tStart));

			if (solutionFound) {
				return new ArmConfiguration(phi, alpha);
			} else {
				invertArm = !invertArm;
			}
		} while (prevInvertArmCal==invertArm);
		return null;
	}

	public void setA1AngleRange(double minA1Angle, double maxA1Angle) {
		this.minA1Angle = minA1Angle;
		this.maxA1Angle = maxA1Angle;
	}

	public void setA2AngleRange(double minA2Angle, double maxA2Angle) {
		this.minA2Angle = minA2Angle;
		this.maxA2Angle = maxA2Angle;
	}

	public void setAllowedDeviation(double acceptableRDeviation, double acceptableThetaDeviation) {
		this.acceptableRDeviation = acceptableRDeviation;
		this.acceptableThetaDeviation = acceptableThetaDeviation;
	}



}
