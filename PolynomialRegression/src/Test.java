public class Test {
	public static String getCsv(double[] x, double[] y) {
		String csv = "";
		for(int i = 0; i < x.length; i++) {
			csv += Double.toString(x[i]) + "," + Double.toString(y[i]) + "\n";
		}

		return csv;
	}

	public static void main(String args[]) {
		//Right turn - 2nd degree
		//double[] x = {0, 3, 6, 8, 12};
		//double[] y = {2, 4, 8, 9, 11};

		//Right then left - 3rd degree
		double[] x = {0, 0.5, 1, 1.5, 2, 3, 4, 5.5, 7, 8, 8.5, 9};
		double[] y = {0, 1, 2, 3, 4, 5, 6, 6.5, 7, 8, 9, 10};

		//Square root - 2nd degree (eh)
		/*double[] x = {0, 1, 4, 9, 16};
		double[] y = {0, 1, 2, 3, 4};*/

		//Quadratic - 2d degree (duh)
		//double[] x = {1, 2, 3};
		//double[] y = {1, 4, 9};

		PolynomialRegression p = new PolynomialRegression(x, y, 3);

		int size = (int)(x[x.length - 1] * 10 + 1);

		double[] xs = new double[size];
		double[] ys = new double[size];

		for(int i = 0; i < size; i++) {
			xs[i] = i / 10.0;
			ys[i] = p.predict(i / 10.0);
		}

		System.out.println("Predicted: " + p.predict(4));
		System.out.println(p);
		System.out.println(getCsv(xs, ys));
		System.out.println("***********\n");
		System.out.println(getCsv(x, y));
	}
}
