// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.HopperToOuttakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeCommand extends Command {
  private HopperSubsystem m_HopperSubsystem;
  private HopperToOuttakeSubsystem m_HopperToOuttakeSubsystem;
  private OuttakeSubsystem m_OuttakeSubsystem;
  
  /** Creates a new outtakeCommand. */
  public OuttakeCommand(HopperSubsystem m_HopperSubsystem,
                         HopperToOuttakeSubsystem m_HopperToOuttakeSubsystem, OuttakeSubsystem m_OuttakeSubsystem) {
    this.m_HopperSubsystem = m_HopperSubsystem;
    this.m_HopperToOuttakeSubsystem = m_HopperToOuttakeSubsystem;
    this.m_OuttakeSubsystem = m_OuttakeSubsystem;
    addRequirements(m_HopperSubsystem, m_HopperToOuttakeSubsystem, m_OuttakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_HopperSubsystem.setHopperSpeed(0.3);
    m_HopperToOuttakeSubsystem.setHopperToOuttakeSpeed(-0.8);
    m_OuttakeSubsystem.setOuttakeSpeed(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_HopperSubsystem.stopHopper();
    m_HopperToOuttakeSubsystem.stopHopperToOuttake();
    m_OuttakeSubsystem.stopOuttake(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
