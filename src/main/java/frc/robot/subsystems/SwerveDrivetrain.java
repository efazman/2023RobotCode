package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveModes;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.CrevoLib.math.Conversions;


public class SwerveDrivetrain extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveModules;
    public Pigeon2 m_pigeonGyro;
    public DriveModes currDriveMode;

    public SwerveModuleState[] setpointState = {
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0))};

    
    public SwerveDrivetrain(){
        m_pigeonGyro = new Pigeon2(SwerveDrivetrainConstants.PIGEON_ID);
        // m_pigeonGyro.setYaw(0);

    
    
        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, 
                            SwerveDrivetrainConstants.FRONT_LEFT_OFFSET,
                            SwerveDrivetrainConstants.FRONT_LEFT_ANGLE_ID,
                            SwerveDrivetrainConstants.FRONT_LEFT_DRIVE_ID,
                            SwerveDrivetrainConstants.FRONT_LEFT_CANCODER_ID,
                            SwerveDrivetrainConstants.FRONT_LEFT_ANGLE_INVERT,
                            SwerveDrivetrainConstants.FRONT_LEFT_DRIVE_INVERT,
                            SwerveDrivetrainConstants.FRONT_LEFT_CANCODER_INVERT
            ),
            new SwerveModule(1, 
                            SwerveDrivetrainConstants.FRONT_RIGHT_OFFSET,
                            SwerveDrivetrainConstants.FRONT_RIGHT_ANGLE_ID,
                            SwerveDrivetrainConstants.FRONT_RIGHT_DRIVE_ID,
                            SwerveDrivetrainConstants.FRONT_RIGHT_CANCODER_ID,
                            SwerveDrivetrainConstants.FRONT_RIGHT_ANGLE_INVERT,
                            SwerveDrivetrainConstants.FRONT_RIGHT_DRIVE_INVERT,
                            SwerveDrivetrainConstants.FRONT_RIGHT_CANCODER_INVERT
            ),
            new SwerveModule(2, 
                            SwerveDrivetrainConstants.BACK_LEFT_OFFSET,
                            SwerveDrivetrainConstants.BACK_LEFT_ANGLE_ID,
                            SwerveDrivetrainConstants.BACK_LEFT_DRIVE_ID,
                            SwerveDrivetrainConstants.BACK_LEFT_CANCODER_ID,
                            SwerveDrivetrainConstants.BACK_LEFT_ANGLE_INVERT,
                            SwerveDrivetrainConstants.BACK_LEFT_DRIVE_INVERT,
                            SwerveDrivetrainConstants.BACK_LEFT_CANCODER_INVERT
            ),
            new SwerveModule(3, 
                            SwerveDrivetrainConstants.BACK_RIGHT_OFFSET,
                            SwerveDrivetrainConstants.BACK_RIGHT_ANGLE_ID,
                            SwerveDrivetrainConstants.BACK_RIGHT_DRIVE_ID,
                            SwerveDrivetrainConstants.BACK_RIGHT_CANCODER_ID,
                            SwerveDrivetrainConstants.BACK_RIGHT_ANGLE_INVERT,
                            SwerveDrivetrainConstants.BACK_RIGHT_DRIVE_INVERT,
                            SwerveDrivetrainConstants.BACK_RIGHT_CANCODER_INVERT
            ),
        };

        currDriveMode = DriveModes.NORMAL;
        m_swerveOdometry = new SwerveDriveOdometry(SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        final SwerveModuleState[] swerveModuleStates;
        
        if (fieldRelative) {
            swerveModuleStates = SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
            );
        } else {
            swerveModuleStates = SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                    new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDrivetrainConstants.MAX_SPEED);

        for(var module : m_swerveModules){
            module.setDesiredState(swerveModuleStates[module.m_moduleNumber], isOpenLoop);
        }
        setpointState = swerveModuleStates;
    }

    public void driveCustomCenterOfRotation(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        final SwerveModuleState[] swerveModuleStates;
        
        if (fieldRelative) {
            swerveModuleStates = SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()), 
                    new Translation2d(SwerveDrivetrainConstants.DRIVETRAIN_ACTUAL_LENGTH/2.0, 0)
            );
        } else {
            swerveModuleStates = SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                    new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                    new Translation2d(SwerveDrivetrainConstants.DRIVETRAIN_ACTUAL_LENGTH/2.0, 0)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDrivetrainConstants.MAX_SPEED);

        for(var module : m_swerveModules){
            module.setDesiredState(swerveModuleStates[module.m_moduleNumber], isOpenLoop);
        }
        setpointState = swerveModuleStates;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDrivetrainConstants.MAX_SPEED);
        
        for (var mod : m_swerveModules) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds));
    }

    public void stopSwerve() {
        drive(new Translation2d(0 ,0), 0, true, true);
    }

    public ChassisSpeeds getChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, Rotation2d robotAngle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotAngle);
    }

    public boolean getIsNormalDriveMode() { 
        if(currDriveMode == DriveModes.NORMAL) {
            return true;
        }
        else {
            return false;
        }
    }

    public void changeDriveMode() {
       if(currDriveMode == DriveModes.NORMAL) {
        currDriveMode = DriveModes.CUSTOM_ROTATE;
       }
       else {
        currDriveMode = DriveModes.NORMAL;
       }
    }

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public double getAngle() {
        return m_pigeonGyro.getYaw();
    }

    public double getNonContinuousGyro() {
        return getAngle() % 360;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (var mod : m_swerveModules) {
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public void goForward() {
        for(var mod : m_swerveModules) {
            mod.setDesiredState(new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)), true);
        }
    }

    public void resetGyro() {
        m_pigeonGyro.setYaw(0);
    }

    public void zeroGyro(double reset) {
        m_pigeonGyro.setYaw(reset);
    }

    public void zeroModules() {
        for(SwerveModule mod: m_swerveModules) {
            mod.zeroModule();
        }
    }

    public SwerveModule[] getModules() {
        return m_swerveModules;
    }

    public Rotation2d getYaw() {
        if (SwerveDrivetrainConstants.PIGEON_INVERT) {
            return Rotation2d.fromDegrees(360 - m_pigeonGyro.getYaw());
        } else {
            return Rotation2d.fromDegrees(m_pigeonGyro.getYaw());
        }
    }

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveModules){
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    @Override
    public void periodic(){
        //UPDATE ODOMETRY
        m_swerveOdometry.update(getYaw(), getModulePositions());


        //UPDATE LOGGER (PERIODIC) -> AdvantageScope Configuration
        Logger.getInstance().recordOutput("Odometry/RobotPose2d", m_swerveOdometry.getPoseMeters());
        Logger.getInstance().recordOutput("Odometry/RobotPose3d", new Pose3d(m_swerveOdometry.getPoseMeters()));
        Logger.getInstance().recordOutput("Yaw/Robot", getAngle());
        
        
        Logger.getInstance().recordOutput("Drive/FLDrivePosition", m_swerveModules[0].getPosition().distanceMeters);
        Logger.getInstance().recordOutput("Drive/FLDriveVelocity", Conversions.falconToMPS(m_swerveModules[0].getDriveVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO));
        Logger.getInstance().recordOutput("Drive/FLDriveTemperature", m_swerveModules[0].getDriveTemperature());
        Logger.getInstance().recordOutput("Drive/FLAngleTemperature", m_swerveModules[0].getAngleTemperature());
        
        Logger.getInstance().recordOutput("Drive/FRDrivePosition", m_swerveModules[1].getPosition().distanceMeters);
        Logger.getInstance().recordOutput("Drive/FRDriveVelocity", Conversions.falconToMPS(m_swerveModules[1].getDriveVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO));
        Logger.getInstance().recordOutput("Drive/FRDriveTemperature", m_swerveModules[1].getDriveTemperature());
        Logger.getInstance().recordOutput("Drive/FRAngleTemperature", m_swerveModules[1].getAngleTemperature());
        
        Logger.getInstance().recordOutput("Drive/BLDrivePosition", m_swerveModules[2].getPosition().distanceMeters);
        Logger.getInstance().recordOutput("Drive/BLDriveVelocity", Conversions.falconToMPS(m_swerveModules[2].getDriveVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO));
        Logger.getInstance().recordOutput("Drive/BLDriveTemperature", m_swerveModules[2].getDriveTemperature());
        Logger.getInstance().recordOutput("Drive/BLAngleTemperature", m_swerveModules[2].getAngleTemperature());
        
        Logger.getInstance().recordOutput("Drive/BRDrivePosition", m_swerveModules[3].getPosition().distanceMeters);
        Logger.getInstance().recordOutput("Drive/BRDriveVelocity", Conversions.falconToMPS(m_swerveModules[3].getDriveVelocity(), SwerveDrivetrainConstants.WHEEL_CIRCUMFERENCE, SwerveDrivetrainConstants.DRIVE_GEAR_RATIO));
        Logger.getInstance().recordOutput("Drive/BRDriveTemperature", m_swerveModules[3].getDriveTemperature());
        Logger.getInstance().recordOutput("Drive/BRAngleTemperature", m_swerveModules[3].getAngleTemperature());
       
        Logger.getInstance().recordOutput("Drive/RealStates", getStates());
        Logger.getInstance().recordOutput("Drive/SetpointStates", setpointState);
        SmartDashboard.putNumber("Yaw", getAngle());

    }
}
