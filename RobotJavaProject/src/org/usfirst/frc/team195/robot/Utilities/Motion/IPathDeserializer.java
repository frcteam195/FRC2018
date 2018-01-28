package org.usfirst.frc.team195.robot.Utilities.Motion;

/**
 * Interface for methods that deserializes a Path or Motion.
 * 
 * @author Jared341
 */
public interface IPathDeserializer {
  
  public Path deserialize(String serialized);
}
