public class CSVMaker {
	private double[] x;
	private double[] y;
	private String csv;

	public CSVMaker(double[] x, double[] y) {
		this.x = x;
		this.y = y;
		csv = "";
	}

	public String getCsv() {
		for(int i = 0; i < x.length; i++) {
			csv += Double.toString(x[i]) + "," + Double.toString(y[i]) + "\n";
		}

		return csv;
	}
}
