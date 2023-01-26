package frc.robot.CrevoLib.io;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Represents a button on a XBox controller
 */
public class Button {
    private enum ButtonType {
        NORMAL, SIMUL_AXIS, POV
    }

    /**
     * The operation mode of the button.
     *
     * <ul>
     * <li>Toggle: Toggles state on press</li>
     * <li>On Press: Returns true when first pressed</li>
     * <li>On Release: Returns true when first released</li>
     * <li>Raw: Returns the raw state of the button</li>
     * </ul>
     */
    public enum ButtonMode {
        TOGGLE, ON_PRESS, ON_RELEASE, RAW
    }

    /**
     * The name of the button
     */
    public enum ButtonID {
        A(1), B(2), X(3), Y(4), START(8), SELECT(7), LEFT_BUMPER(5), RIGHT_BUMPER(6),
        LEFT_JOYSTICK(9), RIGHT_JOYSTICK(10), D_UP(0, ButtonType.POV), D_DOWN(180, ButtonType.POV),
        D_LEFT(270, ButtonType.POV), D_RIGHT(90, ButtonType.POV), LEFT_TRIGGER(2, ButtonType.SIMUL_AXIS),
        RIGHT_TRIGGER(3, ButtonType.SIMUL_AXIS);

        private final int id;
        private final ButtonType type;

        ButtonID(int id) {
            this.type = ButtonType.NORMAL;
            this.id = id;
        }

        ButtonID(int id, ButtonType type) {
            this.type = type;
            this.id = id;
        }

        /**
         * Returns the integer that corresponds with the getRawButton() function
         *
         * @return id
         */
        public int getID() {
            return id;
        }
    }

    private Joystick joystick;
    private ButtonID id;
    private ButtonMode mode;
    private boolean isToggled = false, prevState = false;

    private final double AXIS_THRESHOLD = 0.15;

    /**
     * Creates an button (digital joystick input)
     *
     * @param channel The channel the joystick is on (defined by the Driver Station)
     * @param id      The ID of the button
     * @param mode    The behavior of the button
     */
    Button(int channel, ButtonID id, ButtonMode mode) {
        joystick = new Joystick(channel);
        this.id = id;
        this.mode = mode;
    }

    /**
     * Returns the adjusted state of the button
     *
     * @return button state
     */
    boolean get() {
        boolean state = false;
        if (id.type == ButtonType.NORMAL) state = joystick.getRawButton(id.id);
        else if (id.type == ButtonType.SIMUL_AXIS) state = Math.abs(joystick.getRawAxis(id.id)) > AXIS_THRESHOLD;
        else if (id.type == ButtonType.POV) state = joystick.getPOV() == id.id;

        switch (mode) {
            case RAW: {
                return state;
            }

            case TOGGLE: {
                if (state && !prevState) {
                    isToggled = !isToggled;
                }
                prevState = state;
                return isToggled;
            }

            case ON_PRESS: {
                boolean isPressed = state && !prevState;
                prevState = state;
                return isPressed;
            }

            case ON_RELEASE: {
                boolean isReleased = false;
                if (!state && prevState) isReleased = true;
                prevState = state;
                return isReleased;
            }

            default: {
                return false;
            }
        }
    }

    void setToggleState(boolean state) {
        isToggled = state;
    }
}
