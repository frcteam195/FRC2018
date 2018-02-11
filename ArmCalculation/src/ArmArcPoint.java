import java.util.ArrayList;

public class ArmArcPoint {
	private PolarCoordinate polarCoordinate;
	private PolarCoordinate finalPoint = null;
	private ArrayList<ArmArcPoint> arrPoints = null;

	public ArmArcPoint(PolarCoordinate polarCoordinate) {
		this.polarCoordinate = polarCoordinate;
		arrPoints = null;
		finalPoint = null;
	}

	public ArmArcPoint(PolarCoordinate polarCoordinate, ArrayList<ArmArcPoint> arrPoints) {
		this.polarCoordinate = polarCoordinate;
		this.arrPoints = arrPoints;
		finalPoint = null;
	}

	public ArrayList<ArmArcPoint> getArrPoints() {
		return arrPoints;
	}

	public PolarCoordinate getPolarCoordinate() {
		return polarCoordinate;
	}

	public PolarCoordinate getFinalPoint() {
		return finalPoint;
	}

	public void setArrPoints(ArrayList<ArmArcPoint> arrPoints) {
		this.arrPoints = arrPoints;
	}

	public void setFinalPoint(PolarCoordinate finalPoint) {
		this.finalPoint = finalPoint;
	}

	@Override
	public String toString() {
		String retVal = "";
//		retVal += "R: " + polarCoordinate.r + "\n";
//		retVal += "ø: " + polarCoordinate.theta + "\n";
//		retVal += "\n";
		if (finalPoint != null) {
			retVal += polarCoordinate.r + ",";
			retVal += polarCoordinate.theta + ",";
			retVal += finalPoint.r + ",";
			retVal += finalPoint.theta + "\n";
		}
		return retVal;
	}
}
