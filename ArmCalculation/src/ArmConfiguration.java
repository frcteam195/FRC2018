public class ArmConfiguration {
	public double a1Angle = 0;
	public double a2Angle = 0;

	public ArmConfiguration(double a1Angle, double a2Angle) {
		this.a1Angle = a1Angle;
		this.a2Angle = a2Angle;
	}

	@Override
	public String toString() {
		return "Angle1: " + a1Angle + ", Angle2: " + a2Angle;
	}
}
