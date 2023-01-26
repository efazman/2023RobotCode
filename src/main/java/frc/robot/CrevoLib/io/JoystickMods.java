package frc.robot.CrevoLib.io;

public class JoystickMods {
    /*Linear Interpolation on the Joystick Axis, makes the movement softer and accurate */
    public static double interpolateJoystickAxis(double axis, double deadband){
        if(axis < -deadband) {
          axis = (axis+deadband)/(1-deadband);
        }
        else if (axis > deadband){
          axis = (axis-deadband)/(1-deadband);
        }
        return axis;
      }
}
