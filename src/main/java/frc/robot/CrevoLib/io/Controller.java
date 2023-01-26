package frc.robot.CrevoLib.io;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.function.DoubleFunction;

/**
 * Represents a XBox controller
 */
public class Controller {
    private HashMap<Button.ButtonID, Button> mButtons = new HashMap<>();
    private HashMap<Axis.AxisID, Axis> mAxis = new HashMap<>();
    private final int mChannel;

    public Controller(int channel) {
        this.mChannel = channel;
    }

    public boolean get(Button.ButtonID id) throws MissingIoObjectException {
        if (!mButtons.containsKey(id)) {
            throw new MissingIoObjectException();
        }
        return mButtons.get(id).get();
    }

    public double get(Axis.AxisID id) throws MissingIoObjectException {
        if (!mAxis.containsKey(id)) {
            throw new MissingIoObjectException();
        }
        return mAxis.get(id).get();
    }

    public void addAxis(Axis.AxisID id, double deadband) {
        if (!mAxis.containsKey(id)) {
            mAxis.put(id, new Axis(mChannel, id, deadband));
        }
    }

    public void addAxis(Axis.AxisID id, double deadband, DoubleFunction<Double> inputShaper) {
        if (!mAxis.containsKey(id)) {
            mAxis.put(id, new Axis(mChannel, id, deadband, inputShaper));
        }
    }

    public void addButton(Button.ButtonID id, Button.ButtonMode mode) {
        if (!mButtons.containsKey(id)) {
            mButtons.put(id, new Button(mChannel, id, mode));
        }
    }

    public void setToggleState(Button.ButtonID id, boolean state) throws MissingIoObjectException {
        if (!mAxis.containsKey(id)) {
            throw new MissingIoObjectException();
        }

        mButtons.get(id).setToggleState(state);
    }

    public boolean isControllerConnected() {
        return DriverStation.getStickAxisCount(mChannel) <= 0;
    }
}