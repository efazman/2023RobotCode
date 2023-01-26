package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.CrevoLib.io.JoystickMods;
import frc.robot.subsystems.SwerveDrivetrain;

public class TeleopCustomRotation extends CommandBase {
    private double m_rotation;
    private Translation2d m_translation;
    private boolean m_fieldRelative;
    private boolean m_openLoop;
    
    private SwerveDrivetrain m_swerveDrivetrain;
    private XboxController m_driverController;
    private int m_driveAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;

    public TeleopCustomRotation(SwerveDrivetrain swerveDrivetrain, XboxController driverController, int driveAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

        m_driverController = driverController;
        m_driveAxis = driveAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;
    }

    @Override
    public void execute() {
      double yAxis = m_driverController.getRawAxis(m_driveAxis);
      double xAxis = m_driverController.getRawAxis(m_strafeAxis);
      double rAxis = m_driverController.getRawAxis(m_rotationAxis);
      
      /* Deadbands */
      yAxis = (Math.abs(yAxis) < JoystickConstants.STICK_DEADBAND) ? 0 : yAxis;
      xAxis = (Math.abs(xAxis) < JoystickConstants.STICK_DEADBAND) ? 0 : xAxis;
      rAxis = (Math.abs(rAxis) < JoystickConstants.STICK_DEADBAND) ? 0 : rAxis;

      /*Softer Movement, Greater Accuracy, Interpolation Method */
      yAxis = JoystickMods.interpolateJoystickAxis(yAxis, JoystickConstants.STICK_DEADBAND);
      xAxis = JoystickMods.interpolateJoystickAxis(xAxis, JoystickConstants.STICK_DEADBAND);
      rAxis = JoystickMods.interpolateJoystickAxis(rAxis, JoystickConstants.STICK_DEADBAND);

      m_translation = new Translation2d(yAxis, xAxis).times(SwerveDrivetrainConstants.MAX_SPEED);
      m_rotation = rAxis * SwerveDrivetrainConstants.MAX_ANGULAR_VELOCITY;
      m_swerveDrivetrain.driveCustomCenterOfRotation(m_translation, m_rotation, m_fieldRelative, m_openLoop);
  }
}
