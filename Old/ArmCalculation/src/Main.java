import java.io.FileWriter;
import java.io.IOException;

public class Main {

    public static void main(String[] args) {

        PointFinder pointFinder = new PointFinder();
        pointFinder.setAllowedDeviation(0.5, 2);
        pointFinder.setA1AngleRange(0, 180);
        pointFinder.setA2AngleRange(-160, 160);
        //ArmConfiguration myConfig = pointFinder.getArmConfiguration(new PolarCoordinate(4, 50));
        //if (myConfig != null)
        //   System.out.println(myConfig);

//        //System.out.println("Hello World!");
//        ArrayList<ArmArcPoint> a1Points = new ArrayList<ArmArcPoint>();
//
//        final double kA1 = 10;
//        final double kA2 = 8;
        FileWriter f = null;

        try {
            f = new FileWriter("/Users/roberthilton/Desktop/ArmOut.csv", false);
            f.write("r,theta,phi,alpha\n");
        } catch (Exception ex) {
            ex.printStackTrace();
            System.exit(1);
        }

        for (int i = 0; i <= 16.75; i++) {
            for (int j = 0; j <= 180; j++) {
                ArmConfiguration armConfiguration = pointFinder.getArmConfiguration(new PolarCoordinate(i, j), true);
                if (armConfiguration!=null) {
                    try {
                        f.write( i + "," + j + "," + armConfiguration.a1Angle + "," + armConfiguration.a2Angle + "\n");
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                } else {

                }
            }
        }

//
//        for (int i = 0; i < 180; i++) {
//
//            ArrayList<ArmArcPoint> a2Points = new ArrayList<ArmArcPoint>();
//            for (int j = -60; j < 240; j++) {
//                ArmArcPoint a2Point = new ArmArcPoint(new PolarCoordinate(kA2, j));
//                a2Point.setFinalPoint(getFinalPoint(kA1, i, kA2, j));
//                a2Points.add(a2Point);
//                try {
//                    f.write( kA1 + "," + i + "," + kA2 + "," + j + "," + a2Point.getFinalPoint().r + "," + a2Point.getFinalPoint().theta + "\n");
//                } catch (IOException e) {
//                    e.printStackTrace();
//                }
//            }
//
//            a1Points.add(new ArmArcPoint(new PolarCoordinate(kA1, i), a2Points));
//        }
//
//
//        //Check our construction
////        String s = "a1,phi,a2,alpha,r,theta\n";
////        for (ArmArcPoint a : a1Points) {
////            //System.out.println(a);
////            for (ArmArcPoint b : a.getArrPoints()) {
////                String printVal = a.getPolarCoordinate().r + "," + a.getPolarCoordinate().theta + ",";
////                printVal += b.toString();
////                s += printVal;
////            }
////        }
//
        try {
            f.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
//
//        /*
//        try {
//            FileWriter f = new FileWriter("/Users/roberthilton/Desktop/ArmOut.csv", false);
//            f.write(s);
//            f.close();
//        } catch (Exception ex) {
//            ex.printStackTrace();
//        }
//        */

    }

//    private static PolarCoordinate getFinalPoint(double a1, double phi, double a2, double alpha) {
//        phi = Math.toRadians(phi);
//        alpha = Math.toRadians(alpha);
//        double x = a1 * Math.cos(phi) + a2 * Math.cos(phi - alpha);
//        double y = a1 * Math.sin(phi) + a2 * Math.sin(phi - alpha);
//        double r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
//        double theta = Math.toDegrees(Math.atan2(y, x));
//
//        if (theta < -60)
//            theta += 360;
////        if (x < 0 && y < 0)
////            theta += 180;
////        else if (x > 0 && y < 0)
////            theta -= 180;
//        return new PolarCoordinate(r, theta);
//    }
}
