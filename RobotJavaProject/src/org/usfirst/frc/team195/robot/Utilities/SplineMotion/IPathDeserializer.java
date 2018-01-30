package org.usfirst.frc.team195.robot.Utilities.SplineMotion;

/**
 * Interface for methods that deserializes a Path or SplineMotion.
 * 
 * @author Jared341
 */
public interface IPathDeserializer {
  
  public Path deserialize(String serialized);
}
