// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperToOuttakeSubsystem extends SubsystemBase {
  private final SparkMax hopperToOuttakeMotor;
  /** Creates a new HopperToOuttake. */
  public HopperToOuttakeSubsystem() {
    hopperToOuttakeMotor = new SparkMax(13, MotorType.kBrushless);
  }
  public void setHopperToOuttakeSpeed(double speed){
    hopperToOuttakeMotor.set(speed);
  }
  public void stopHopperToOuttake(){
    hopperToOuttakeMotor.set(0);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
