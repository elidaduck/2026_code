// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeExtentionSubsystem extends SubsystemBase {
  private final SparkMax intakeExtentionMotor;
  public double m_intakeKP;
  public double m_intakeKI;
  public double m_intakeKD;
  private SparkClosedLoopController intakeextentionPIDController;
  private SparkMaxConfig IntakeExtentionConfig;
  private RelativeEncoder extentionEncoder;


  


  /** Creates a new IntakeSubsystem. */
  public IntakeExtentionSubsystem() {
    intakeExtentionMotor = new SparkMax(9,MotorType.kBrushless); 
    this.m_intakeKP = Constants.intakeKP;
    this.m_intakeKI = Constants.intakeKI;
    this.m_intakeKD = Constants.intakeKD;
    IntakeExtentionConfig = new SparkMaxConfig();
    IntakeExtentionConfig.closedLoop.pid(m_intakeKP, m_intakeKI, m_intakeKD);
    IntakeExtentionConfig.inverted(false).idleMode(IdleMode.kBrake);
    IntakeExtentionConfig.encoder.positionConversionFactor(Constants.intakeGearRatio);
    intakeextentionPIDController = intakeExtentionMotor.getClosedLoopController();
    extentionEncoder = intakeExtentionMotor.getEncoder();
    IntakeExtentionConfig.smartCurrentLimit(40);
    intakeExtentionMotor.configure(IntakeExtentionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    



  }
  public void setIntakeExtentionSpeed(double speed){
    intakeExtentionMotor.set(speed);
  }
  public void stopIntake(){
    intakeExtentionMotor.set(0);
  }
  public void setIntakePosition(double position){
    intakeextentionPIDController.setSetpoint(position, ControlType.kPosition);
    extentionEncoder.setPosition(position);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
