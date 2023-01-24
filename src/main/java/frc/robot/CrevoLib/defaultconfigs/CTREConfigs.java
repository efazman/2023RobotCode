package frc.robot.CrevoLib.defaultconfigs;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.SwerveDrivetrainConstants;

public class CTREConfigs {
    public TalonFXConfiguration driveMotorConfig, angleMotorConfig;
    public CANCoderConfiguration canCoderConfig;
    // public Pigeon2Configuration pigeon2Configuration;
    public CTREConfigs(){
        driveMotorConfig = new TalonFXConfiguration();
        angleMotorConfig = new TalonFXConfiguration();
        canCoderConfig = new CANCoderConfiguration();
        
        //Angle Motor Default Configs
        SupplyCurrentLimitConfiguration angleMotorSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveDrivetrainConstants.ANGLE_ENABLE_CURRENT_LIMIT,
            SwerveDrivetrainConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT,
            SwerveDrivetrainConstants.ANGLE_PEAK_CURRENT_LIMIT,
            SwerveDrivetrainConstants.ANGLE_PEAK_CURRENT_DURATION
        );

        angleMotorConfig.slot0.kP = SwerveDrivetrainConstants.ANGLE_P;
        angleMotorConfig.slot0.kI = SwerveDrivetrainConstants.ANGLE_I;
        angleMotorConfig.slot0.kD = SwerveDrivetrainConstants.ANGLE_D;
        angleMotorConfig.slot0.kF = SwerveDrivetrainConstants.ANGLE_F;
        angleMotorConfig.supplyCurrLimit = angleMotorSupplyLimit;
        angleMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        angleMotorConfig.slot0.allowableClosedloopError = SwerveDrivetrainConstants.ANGLE_ALLOWABLE_CL_ERROR;

        //Drive Motor Default Configs
        SupplyCurrentLimitConfiguration driveMotorSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveDrivetrainConstants.DRIVE_ENABLE_CURRENT_LIMIT,
            SwerveDrivetrainConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT,
            SwerveDrivetrainConstants.DRIVE_PEAK_CURRENT_LIMIT,
            SwerveDrivetrainConstants.DRIVE_PEAK_CURRENT_DURATION
        );

        driveMotorConfig.slot0.kP = SwerveDrivetrainConstants.DRIVE_P;
        driveMotorConfig.slot0.kI = SwerveDrivetrainConstants.DRIVE_I;
        driveMotorConfig.slot0.kD = SwerveDrivetrainConstants.DRIVE_D;
        driveMotorConfig.slot0.kF = SwerveDrivetrainConstants.DRIVE_F;
        driveMotorConfig.supplyCurrLimit = driveMotorSupplyLimit;
        driveMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        driveMotorConfig.openloopRamp = SwerveDrivetrainConstants.OPEN_LOOP_RAMP;
        driveMotorConfig.closedloopRamp = SwerveDrivetrainConstants.CLOSED_LOOP_RAMP;

        //CANCoder Configuration
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

   
        
    }   
    public TalonFXConfiguration getDriveMotorConfig() {
        return driveMotorConfig;
    }
    public TalonFXConfiguration getAngleMotorConfig() {
        return angleMotorConfig;
    }
    public CANCoderConfiguration getCANCoderConfig() {
        return canCoderConfig;
    }
}
