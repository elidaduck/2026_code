// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  private final TalonFX outtakeMotor;
  private final LimeLight m_limeLight;
  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem(LimeLight m_limeLight) {
    outtakeMotor = new TalonFX(14);
    this.m_limeLight = m_limeLight;
  }
  public void setOuttakeSpeed(double speed){
    //speed = getDistanceSpeed();
    outtakeMotor.set(speed);
  }
  public void stopOuttake(){
    outtakeMotor.set(0);
  }
  public double getDistanceSpeed(){
    double distance = m_limeLight.getDistance(); //get distance from limelight
    //calculate speed based on distance
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
