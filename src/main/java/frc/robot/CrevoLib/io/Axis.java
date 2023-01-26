package frc.robot.CrevoLib.io;

import edu.wpi.first.wpilibj.Joystick;

import java.util.function.DoubleFunction;

/**
 * Represents an axis on a XBox Controller
 */
public class Axis
{
    /**
     * The name of the axis
     */
    public enum AxisID {
        LEFT_X(0), LEFT_Y(1), RIGHT_X(4), RIGHT_Y(5), LEFT_TRIGGER(2), RIGHT_TRIGGER(3);

        private final int id;

        AxisID(int id) {
            this.id = id;
        }

        /**
         * Returns the integer that corresponds with the getRawAxis() function
         *
         * @return id
         */
        public int getID() {
            return id;
        }
    }

    private final Joystick joystick;
    private final AxisID id;
    private final double deadband;
    private final DoubleFunction<Double> inputShaper;

    Axis(int channel, AxisID id, double deadband) {
        joystick = new Joystick(channel);
        this.id = id;
        this.deadband = deadband;
        this.inputShaper = (x) -> x;
    }

    Axis(int channel, AxisID id, double deadband, DoubleFunction<Double> shaper) {
        joystick = new Joystick(channel);
        this.id = id;
        this.deadband = deadband;
        this.inputShaper = shaper;
    }

    private double applyDeadband(double input) {
        input = (Math.abs(input) < deadband) ? 0 : input;
        return Math.copySign((Math.abs(input) - deadband) / (1 - deadband), input);
    }

    double get() {
        return applyDeadband(inputShaper.apply(joystick.getRawAxis(id.getID())));
    }
}
