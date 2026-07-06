// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  private final SparkMax topHopperMotor;
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    topHopperMotor = new SparkMax(11, MotorType.kBrushless);
    SparkMaxConfig hopperConfig = new SparkMaxConfig();
    hopperConfig.smartCurrentLimit(40);
    topHopperMotor.configure(hopperConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }
  public void setHopperSpeed(double speed){
    topHopperMotor.set(speed);
  }
  public void stopHopper(){
    topHopperMotor.set(0);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
