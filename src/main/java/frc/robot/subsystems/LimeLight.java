package frc.robot.subsystems;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {

  // ---------------------------------------------------------------------------
  // Hub geometry constants (2026 REBUILT)
  // ---------------------------------------------------------------------------

  // The hub is an octagon. Each face has two AprilTags: one centered on the face
  // and one laterally offset. Tags face outward; the hub center is "behind" them.
  //
  // HUB_FACE_DEPTH: distance from the tag face to the hub center (meters).
  //   Approximate from field drawings: hub outer radius ≈ 0.61 m.
  private static final double HUB_FACE_DEPTH = 0.61; // meters

  // HUB_TAG_LATERAL_OFFSET: lateral distance of the *offset* tag from the face midpoint.
  //   From field drawings: ~0.28 m. Centered tags have 0 lateral offset.
  private static final double HUB_TAG_LATERAL_OFFSET = 0.28; // meters

  // Hub AprilTag IDs (from game manual section 5.11).
  // All four hub faces per alliance get 2 tags each: one centered + one offset.
  // Blue alliance hub tags: 2, 3, 4, 5, 8, 9, 10, 11
  // Red  alliance hub tags: 18, 19, 20, 21, 24, 25, 26, 27
  //
  // Per-tag lateral offset in hub-face frame (positive = left as seen from outside).
  // Derived from the field layout JSON (WPILib 2026-rebuilt-welded.json).
  // Tags with offset 0 are the centered tags on each face.
  // Tags with nonzero offset are the laterally shifted tags.
  //
  // Map: tag ID -> lateral offset of hub center relative to tag center (meters).
  // Positive means hub center is to the LEFT of the tag (from outside the hub).
  private static final java.util.Map<Integer, Double> HUB_TAG_LATERAL = new java.util.HashMap<>();
  static {
    // Blue alliance hub tags
    // Face 1 (tags 2=offset, 3=centered), Face 2 (tags 4=centered, 5=offset),
    // Face 3 (tags 8=offset, 9=centered), Face 4 (tags 10=centered, 11=offset)
    // Signs are determined by which side the offset tag is on each face.
    // If unsure of sign, measure on the actual field and flip as needed.
    HUB_TAG_LATERAL.put(2,  +HUB_TAG_LATERAL_OFFSET); // offset tag, blue hub
    HUB_TAG_LATERAL.put(3,   0.0);                     // centered tag, blue hub
    HUB_TAG_LATERAL.put(4,   0.0);                     // centered tag, blue hub
    HUB_TAG_LATERAL.put(5,  -HUB_TAG_LATERAL_OFFSET); // offset tag, blue hub
    HUB_TAG_LATERAL.put(8,  +HUB_TAG_LATERAL_OFFSET); // offset tag, blue hub
    HUB_TAG_LATERAL.put(9,   0.0);                     // centered tag, blue hub
    HUB_TAG_LATERAL.put(10,  0.0);                     // centered tag, blue hub
    HUB_TAG_LATERAL.put(11, -HUB_TAG_LATERAL_OFFSET); // offset tag, blue hub
    // Red alliance hub tags (mirrored)
    HUB_TAG_LATERAL.put(18, -HUB_TAG_LATERAL_OFFSET); // offset tag, red hub
    HUB_TAG_LATERAL.put(19,  0.0);                     // centered tag, red hub
    HUB_TAG_LATERAL.put(20,  0.0);                     // centered tag, red hub
    HUB_TAG_LATERAL.put(21, +HUB_TAG_LATERAL_OFFSET); // offset tag, red hub
    HUB_TAG_LATERAL.put(24, -HUB_TAG_LATERAL_OFFSET); // offset tag, red hub
    HUB_TAG_LATERAL.put(25,  0.0);                     // centered tag, red hub
    HUB_TAG_LATERAL.put(26,  0.0);                     // centered tag, red hub
    HUB_TAG_LATERAL.put(27, +HUB_TAG_LATERAL_OFFSET); // offset tag, red hub
  }

  // ---------------------------------------------------------------------------

  //create NetworkTable objects
  NetworkTableInstance nInstance;
  NetworkTable table;
 
  //limelight values
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry ty;
  private NetworkTableEntry tx;
  private NetworkTableEntry tid;
  public  double[] tagpose, botpose;
  private int pipeline;
  public  double distance;

  //trajectory fields
  private Trajectory trajectory;
  SwerveSubsystem m_SwerveSubsystem;

  HashMap<Double, Pose2d> poses;

  public LimeLight(SwerveSubsystem m_SwerveSubsystem)
  {
      this.m_SwerveSubsystem = m_SwerveSubsystem;
      System.out.println("Limelight object initialized");

      nInstance = NetworkTableInstance.getDefault();
      table = nInstance.getTable("limelight");
      ta = table.getEntry("ta");
      tv = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      tid = table.getEntry("tid");
      tagpose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
      botpose = table.getEntry("robotpose_targetspace").getDoubleArray(new double[6]);
    poses = new HashMap<Double, Pose2d>();

 
  }

  public Pose2d getTagPose() {
      return new Pose2d(getLLData().getY(), getLLData().getX(), Rotation2d.fromDegrees(0));
  }

  public Pose2d getLLData(){
    return LimelightHelpers.getBotPose2d("limelight");
  }
  public void printTargetPoses() {
      System.out.println(Arrays.asList(poses));
  }

  public double getTa() {
      return ta.getDouble(0);
  }

  public double getTv() {
      return tv.getDouble(0);
  }

  public double getTx() {
      return tx.getDouble(0);
  }
  public double getDesiredAngle() {
      return tx.getDouble(0)+m_SwerveSubsystem.getYawDegrees();
  }

  public double getTy() {
      return ty.getDouble(0);
  }

  public double getTid() {
      return tid.getDouble(0);
  }
    public static double getDistance(){
        // Returns planar (ground) distance in meters from the robot origin to the detected target.
        // Uses the Limelight 'targetpose_robotspace' which is in meters.
        if (!LimelightHelpers.getTV("limelight")) {
            return Double.NaN;
        }
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
        Translation3d t = targetPose.getTranslation();
        // planar distance on the ground (ignore vertical Z)
        double planar = Math.hypot(t.getX(), t.getY());
        SmartDashboard.putNumber("LL Target Distance (m)", planar);
        return planar;
    }


  public String getPipeline() {
      return "Currently using pipeline " + pipeline;
  }

  public void setPipeline(int pipeline) {
      this.pipeline = pipeline;
      table.getEntry("pipeline").setNumber(pipeline);
      System.out.println("Pipeline changed to " + pipeline);
  }
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("x value", getTagPose().getX());
    SmartDashboard.putNumber("y value", getTagPose().getY());

      super.periodic();

  }
  public void mapOriginPairs() {
      //initialize dictionary
      if (this.getTv() == 1) {
          poses.put(this.getTid(), this.getTagPose());
          System.out.println("Successfully mapped");
      }

      else {
          System.out.println("No mapping to be done");
      }

  }

  public Trajectory generateTargetTrajectory(TrajectoryConfig config) {
      System.out.println("Trajectory generated successfully");
      //set tag pose as the current origin
      //origin = this.getTagPose();
      if (this.getTv() == 1) {
          m_SwerveSubsystem.resetOdometry(m_SwerveSubsystem.getPose()); //sets origin to tag pose
 
          trajectory = TrajectoryGenerator.generateTrajectory(
              m_SwerveSubsystem.getPose(),
              List.of(),
              this.getTagPose(),
              config);

      }
     
      return trajectory;
 
  }

    public void faceTag(){

        if(getTv() == 1){
            // Limelight tx is in degrees (horizontal offset)
            double txDegrees = getTx();

            // convert to radians
            double errorRad = Math.toRadians(txDegrees);

            // simple P controller to convert angle error to angular velocity
            double omega = m_SwerveSubsystem.turnController.calculate(errorRad, 0); // setpoint is 0 radians (facing the tag)

            // clamp
            omega = Math.max(-Constants.SwerveConstants.maxAngularVelocity, Math.min(Constants.SwerveConstants.maxAngularVelocity, omega));
            omega = MathUtil.applyDeadband(omega, Constants.SwerveConstants.inputDeadband);

            // drive: translation=0, rotation=omega (rad/s), fieldRelative=true, isOpenLoop=true
            m_SwerveSubsystem.drive(new Translation2d(0,0), omega, true, true);
        }else{
            // if no target, stop rotating
            m_SwerveSubsystem.drive(new Translation2d(0,0), 0, true, true);
        }
    }

    /**
     * Turns the robot to aim at the HUB CENTER rather than the face of the detected tag.
     *
     * For hub tags on an octagonal structure, facing the tag directly often means
     * facing the TAG FACE rather than the center of the hub opening. This method
     * uses the 3D pose of the tag (in robot-relative space) to compute the angle
     * to the hub center and drives toward that instead.
     *
     * Call this instead of faceTag() when trying to score.
     */
    public void faceHub() {
        if (getTv() != 1) {
            m_SwerveSubsystem.drive(new Translation2d(0, 0), 0, true, true);
            return;
        }

        int tagId = (int) getTid();
        Double hubLateral = HUB_TAG_LATERAL.get(tagId);

        if (hubLateral == null) {
            // Not a hub tag — fall back to regular tag facing
            faceTag();
            return;
        }

        // robotpose_targetspace gives the robot's position in the TAG's coordinate frame:
        //   [0] = x  (rightward from tag's perspective)
        //   [1] = y  (upward)
        //   [2] = z  (toward robot / outward from hub face; positive = away from hub)
        //   [3..5] = rotation (not needed here)
        double[] robotInTagFrame = table.getEntry("robotpose_targetspace")
                                        .getDoubleArray(new double[6]);

        double robot_x = robotInTagFrame[0]; // robot's lateral position in tag frame
        double robot_z = robotInTagFrame[2]; // robot's forward distance from tag (positive = toward robot)

        // Hub center position in tag frame:
        //   laterally:  hubLateral (left/right from face midpoint)
        //   depth:      -HUB_FACE_DEPTH (behind the tag face, into the hub)
        //
        // Vector from robot to hub center, in tag frame:
        //   dx = hubLateral - robot_x   (lateral)
        //   dz = -HUB_FACE_DEPTH - (-robot_z) = robot_z - HUB_FACE_DEPTH  (forward)
        //   Note: robot_z is positive (robot is in front of tag), so dz is the net
        //   depth from the robot to the hub center.
        double dx = hubLateral - robot_x;
        double dz = robot_z - HUB_FACE_DEPTH; // positive means hub center is still in front

        // The angle to the hub center relative to the tag-face normal (in the horizontal plane)
        // is atan2(dx, dz). This is the angle ERROR the robot needs to correct.
        // Positive dx => hub center is to the right of the tag => robot must turn right (negative tx correction).
        // The Limelight tx convention: positive = target is to the robot's right.
        // So the corrected horizontal aim angle (in radians) is:
        //   setpoint = current_tx + atan2(dx, dz)
        // But we want to DRIVE TO this corrected angle, so the error is:
        //   error = tx_rad - atan2(dx, dz)
        // We set the PID setpoint to atan2(dx, dz) (the desired tx in radians = hub direction).
        double targetTxRad = Math.atan2(dx, dz);

        // Current tx in radians
        double currentTxRad = Math.toRadians(getTx());

        // Drive tx toward targetTxRad (not toward 0 as in faceTag)
        double omega = m_SwerveSubsystem.turnController.calculate(currentTxRad, targetTxRad);

        omega = Math.max(-Constants.SwerveConstants.maxAngularVelocity,
                         Math.min(Constants.SwerveConstants.maxAngularVelocity, omega));
        omega = MathUtil.applyDeadband(omega, Constants.SwerveConstants.inputDeadband);

        SmartDashboard.putNumber("Hub Target TX (deg)", Math.toDegrees(targetTxRad));
        SmartDashboard.putNumber("Hub TX Error (deg)", Math.toDegrees(currentTxRad - targetTxRad));

        m_SwerveSubsystem.drive(new Translation2d(0, 0), omega, true, true);
    }
}