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
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
   
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
  public double tagOmega;

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
            double errorRad = Math.toRadians(-txDegrees);

            // simple P controller to convert angle error to angular velocity
            double omega = m_SwerveSubsystem.turnController.calculate(errorRad, 0); // setpoint is 0 radians (facing the tag)

            // clamp
            omega = Math.max(-Constants.SwerveConstants.maxAngularVelocity, Math.min(Constants.SwerveConstants.maxAngularVelocity, omega));
            // omega = MathUtil.applyDeadband(omega, Constants.SwerveConstants.inputDeadband);

            SmartDashboard.putNumber("omega", omega);
            // drive: translation=0, rotation=omega (rad/s), fieldRelative=true, isOpenLoop=true
            m_SwerveSubsystem.drive(new Translation2d(0,0), omega*Constants.SwerveConstants.maxAngularVelocity, true, true);
        }
        // else{
        //     // if no target, stop rotating
        //     m_SwerveSubsystem.drive(new Translation2d(0,0), 0, true, true);
        // }
    }
}
