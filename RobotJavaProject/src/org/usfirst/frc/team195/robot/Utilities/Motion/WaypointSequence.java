package org.usfirst.frc.team195.robot.Utilities.Motion;

import java.util.ArrayList;

/**
 * A WaypointSequence is a sequence of Waypoints.  #whatdidyouexpect
 *
 * @author Art Kalb
 * @author Stephen Pinkerton
 * @author Jared341
 */
public class WaypointSequence {

  public static class Waypoint {

    public Waypoint(double x, double y, double theta) {
      this.x = x;
      this.y = y;
      this.theta = theta;
    }
    
    public Waypoint(Waypoint tocopy) {
      this.x = tocopy.x;
      this.y = tocopy.y;
      this.theta = tocopy.theta;
    }

    public double x;
    public double y;
    public double theta;
  }

  private ArrayList<Waypoint> waypoints_;

  public WaypointSequence() {
    waypoints_ = new ArrayList<Waypoint>();
  }

  public void addWaypoint(Waypoint w) {
    waypoints_.add(w);
  }

  public int getNumWaypoints() {
    return waypoints_.size();
  }

  public Waypoint getWaypoint(int index) {
    if (index >= 0 && index < getNumWaypoints()) {
      return waypoints_.get(index);
    } else {
      return null;
    }
  }
  
  public WaypointSequence invertY() {
    WaypointSequence inverted = new WaypointSequence();
    for (int i = 0; i < waypoints_.size(); ++i) {
      Waypoint w = waypoints_.get(i);
      Waypoint invertedWaypoint = new Waypoint(w.x, w.y * -1, ChezyMath.boundAngle0to2PiRadians(2*Math.PI - w.theta));
      inverted.addWaypoint(invertedWaypoint);
    }
    
    return inverted;
  }
}
