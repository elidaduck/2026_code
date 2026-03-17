// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  private final SparkMax outtakeMotor;
  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    outtakeMotor = new SparkMax(14, MotorType.kBrushless);
  }
  public void setOuttakeSpeed(double speed){
    //speed = getDistanceSpeed();
    outtakeMotor.set(speed);
  }
  public void stopOuttake(){
    outtakeMotor.set(0);
  }
  public double getDistanceSpeed(){
    double distance = LimeLight.getDistance(); //get distance from limelight
    //calculate speed based on distance
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
