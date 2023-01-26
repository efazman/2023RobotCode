package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PoseEstimationConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimator extends SubsystemBase {
    //Declares the PhotonCamera we use (Limelight 2+)
    private final PhotonCamera photonCamera;
    //Declares the SwerveDrivetrain Subsystem to use
    private final SwerveDrivetrain m_drivetrain;
    //Declares the AprilTagLayout on the Charged Up Field
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  //FOLLOWING CODE TAKEN DIRECTLY FROM WPILIB EXAMPLE CODE -> KALMAN FILTER CONFIGURATIONS
  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  //Creates a WPILIB SwerveDrivePoseEstimator object
  private final SwerveDrivePoseEstimator poseEstimator;

  //Creates a 2d Field to animate our robot's movement
  private final Field2d field2d = new Field2d();
  
  //Stores the timestamp from the previous vision capture of an AprilTag object
  private double previousPipelineTimestamp = 0;

  //Constructor for PoseEstimator Subsystem
  public PoseEstimator(PhotonCamera photonCamera, SwerveDrivetrain drivetrainSubsystem) {
    //instantiates the photoncamera and swerve drivetrain
    this.photonCamera = photonCamera;
    this.m_drivetrain = drivetrainSubsystem;
    //try catch logic to access the AprilTag WPI resource file and current alliance
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = PoseEstimationConstants.CURRENT_ALLIANCE;
      layout.setOrigin(alliance == Alliance.Red ?
          OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;

    //Creates a new Shuffleboard tab to visualize the vision animations
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    //instantiates the SwerveDrivePoseEstimator WPI object
    poseEstimator =  new SwerveDrivePoseEstimator(
        SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS,
        drivetrainSubsystem.getYaw(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    //Adds the Pose (x,y) to Shuffleboard to see the changes in robot pose
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    //Adds field animation to the Shuffleboard Field2d Widget
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }
  
  @Override
  public void periodic() {
    // Update pose estimator with the best visible target
    var pipelineResult = photonCamera.getLatestResult();
    //gets the timestamp of when the result was recorded - to be compared with the previous timestamp variable
    var resultTimestamp = pipelineResult.getTimestampSeconds();

    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
    // we have found results in the pipeline so now the new timestamp = result timestamp
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      //the ID of the AprilTag detected
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        //find the transform 3d from camera to target
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
        //Tranform the cam pose to the pose of the camera on the robot, physical location of the camera
        var visionMeasurement = camPose.transformBy(VisionConstants.CAM_TO_ROBOT);
        //adds another vision measurement to our pose estimator
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      m_drivetrain.getYaw(),
      m_drivetrain.getModulePositions());

    //sets the field animation with the current pose detected
    field2d.setRobotPose(getCurrentPose());
    //AdvantageScope logging for 2d pose visualization (not necessary, but looks cool)
    Logger.getInstance().recordOutput("Estimated Pose", getCurrentPose());
  }

  //returns a string version of the pose in a nice to visualize way to put in Shuffleboard
  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  //returns current estimated pose
  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      m_drivetrain.getYaw(),
      m_drivetrain.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }
 
}

