public class PolarCoordinate implements Comparable<PolarCoordinate> {
	public double r;
	public double theta;

	public PolarCoordinate(double r, double theta) {
		this.r = r;
		this.theta = theta;
	}

	@Override
	public int compareTo(PolarCoordinate o) {
		if (o.r == r && o.theta == theta)
			return 0;
		return -1;
	}
}
