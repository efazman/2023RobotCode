package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.CrevoLib.defaultconfigs.CTREConfigs;
import frc.robot.CrevoLib.math.Conversions;
import frc.robot.CrevoLib.util.CTREModuleState;


public class SwerveModule {
    //SINGULAR SWERVE MODULE CONSTRUCTION CLASS
    public int m_moduleNumber;
    private double m_offset;
    private TalonFX m_angleMotor, m_driveMotor;
    private CANCoder m_canCoder;
    private double m_lastAngle;
    private boolean m_driveInverted, m_angleInverted, m_canCoderInverted;
    public CTREConfigs ctreConfigs = new CTREConfigs();

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
                                        SwerveDrivetrainConstants.DRIVE_kS, 
                                        SwerveDrivetrainConstants.DRIVE_kV, 
                                        SwerveDrivetrainConstants.DRIVE_kA);
    
    public SwerveModule(int moduleNumber, double offset, int angleMotor, int driveMotor, int canCoder, 
    boolean angleInverted, boolean driveInverted, boolean cancoderInverted) {
        m_moduleNumber = moduleNumber;
        m_offset = offset;
        m_driveInverted = driveInverted;
        m_angleInverted = angleInverted;
        m_canCoderInverted = cancoderInverted;
        
        m_angleMotor = new TalonFX(angleMotor);
        m_driveMotor = new TalonFX(driveMotor);
        m_canCoder = new CANCoder(canCoder);

        configAngleMotor();
        configDriveMotor();
        configCanCoder();

        m_lastAngle = getState().angle.getDegrees();
        
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {

        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if(openLoop){
            m_driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond / SwerveDrivetrainConstants.MAX_SPEED);
        } else {
            m_driveMotor.set(ControlMode.Velocity, 
                             Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO), 
                             DemandType.ArbitraryFeedForward, 
                             feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveDrivetrainConstants.MAX_SPEED * 0.01)) ? m_lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
       
        m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));

        m_lastAngle = angle;
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    private void configCanCoder() {
        m_canCoder.configFactoryDefault();
        m_canCoder.configAllSettings(Robot.ctreConfigs.canCoderConfig);
        m_canCoder.configSensorDirection(m_canCoderInverted);
    }

    private void configAngleMotor() {
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(Robot.ctreConfigs.angleMotorConfig);
        m_angleMotor.setInverted(m_angleInverted);
        m_angleMotor.setNeutralMode(SwerveDrivetrainConstants.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(Robot.ctreConfigs.driveMotorConfig);
        m_driveMotor.setInverted(m_driveInverted);
        m_driveMotor.setNeutralMode(SwerveDrivetrainConstants.DRIVE_NEUTRAL_MODE);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    private void resetToAbsolute() {
        m_angleMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_offset, 
                                        SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition());
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
    }

    public double getDriveEncoder() {
        return m_driveMotor.getSelectedSensorPosition();
    }

    public double getDriveEncoderDegrees() {
        return m_canCoder.getAbsolutePosition();
    }

    public double getDriveVelocity() {
        return m_driveMotor.getSelectedSensorVelocity();
    }

    


    public void zeroModule() {
        m_angleMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(m_offset, SwerveDrivetrainConstants.ANGLE_GEAR_RATIO));
    }

    public double getDriveTemperature(){
        return m_driveMotor.getTemperature();
    }

    public double getAngleTemperature() {
        return m_angleMotor.getTemperature();
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }
}
