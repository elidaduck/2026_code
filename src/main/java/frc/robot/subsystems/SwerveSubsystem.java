// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.*;
import com.pathplanner.lib.util.*;
import com.pathplanner.lib.auto.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PPconstants;


public class SwerveSubsystem extends SubsystemBase {
  private final Pigeon2 gyro;
  private Pose2d previousPose;
  private long lastUpdateTime;


  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //instantiates new pigeon gyro, wipes it, and zeros it
    gyro = new Pigeon2(1);
    zeroGyro();
    
    //Creates all four swerve modules into a swerve drive
    mSwerveMods =
    new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveConstants.Mod1.constants),
      new SwerveModule(1, Constants.SwerveConstants.Mod0.constants),
      new SwerveModule(2, Constants.SwerveConstants.Mod3.constants),
      new SwerveModule(3, Constants.SwerveConstants.Mod2.constants)
    };
    
    //creates new swerve odometry (odometry is where the robot is on the field)
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

    //puts out the field
    field = new Field2d();
    SmartDashboard.putData("Field", field);

     // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelitive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(0.01, 0.0, 0.0) // Rotation PID constants
        ),
        PPconstants.PPConfig,
        () ->  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,            
        this // Reference to this subsystem to set requirements
    );
  }

public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // get the current pose of the robot
    Pose2d currentPose = swerveOdometry.getPoseMeters();

    // get the current yaw of the robot
    double currentYaw = currentPose.getRotation().getDegrees();
  

    // calculate the desired chassis speed based on the desired pose
    ChassisSpeeds desiredChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            -translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(-currentYaw))
        : new ChassisSpeeds(-translation.getX(), translation.getY(), rotation);

    // calculate the desired swerve module states based on the desired chassis speed
    SmartDashboard.putNumber("rotation", rotation);
    SwerveModuleState[] desiredSwerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

    // update the swerve module states for each module
    for (int i = 0; i < 4; i++) {
        SwerveModule mod = mSwerveMods[i];
         SwerveModuleState desiredState = desiredSwerveModuleStates[i];

        // Add this line to ensure the module's angle is set correctly
        mod.setAngle(desiredState);

        mod.setDesiredState(desiredState, isOpenLoop);
    }

    // update the swerve odometry with the current pose and the desired swerve module states
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[mSwerveMods.length];
    for (int i = 0; i < mSwerveMods.length; i++) {
        modulePositions[i] = mSwerveMods[i].getPosition();
    }
    Rotation2d currentRotation = getYaw();

    swerveOdometry.update(currentRotation, modulePositions);
}

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void driveRobotRelitive(ChassisSpeeds targetSpeed,DriveFeedforwards feedforwards)
  // takes the coordinate on field wants to go to, the rotation of it, whether or
  // not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeed,
        new Translation2d());
    // sets to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    // set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);// MIGHT NEED TO CHANGE
    }
  }



  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}


  public void zeroGyro() {
    gyro.reset();;
  }

  public Rotation2d getYaw() {
    //fancy if else loop again
    return (Constants.SwerveConstants.invertNavx)
        ? Rotation2d.fromDegrees((360 - gyro.getYaw().getValueAsDouble()))
        : Rotation2d.fromDegrees((gyro.getYaw().getValueAsDouble() + Constants.SwerveConstants.degreesOffSet));
  }
  public double getYawDegrees() {
    return gyro.getYaw().getValueAsDouble();
  }
  

 

  public boolean AutoBalance(){
    double roll_error = gyro.getPitch().getValueAsDouble();//the angle of the robot
    double balance_kp = -.005;//Variable muliplied by roll_error
    double position_adjust = 0.0;
    double min_command = 0.0;//adds a minimum input to the motors to overcome friction if the position adjust isn't enough
    if (roll_error > 6.0)
    {
      position_adjust = balance_kp * roll_error + min_command;//equation that figures out how fast it should go to adjust
      //position_adjust = Math.max(Math.min(position_adjust,.15), -.15);  this gets the same thing done in one line
      if (position_adjust > .1){position_adjust = .1;}
      if (position_adjust < -.1){position_adjust = -.1;}
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      
      return false;
    }
    else if (roll_error < -6.0)
    {
      position_adjust = balance_kp * roll_error - min_command;
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      if (position_adjust > .3){position_adjust = .3;}
      if (position_adjust < -.3){position_adjust = -.3;}
      return false;
    }
    else{
      drive(new Translation2d(0, 0), 0.0, true, false);
      return true;}
    
  }
  public double getAngle(SwerveModule mod){
    if(mod.getState().angle.getDegrees() > 360){
      return mod.getState().angle.getDegrees() -360;
  }
  else if(mod.getState().angle.getDegrees() < 0){
       return mod.getState().angle.getDegrees() + 360;
  }
    return mod.getState().angle.getDegrees();
  }



  @Override
  public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("gyro yaw",  getYaw().getDegrees());
    SmartDashboard.putNumber("robot pose x", getPose().getX());
    SmartDashboard.putNumber("robot pose y", getPose().getY());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated Angle", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
  }
}


}
