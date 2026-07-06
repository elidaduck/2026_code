// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.OuttakeCommand;
// import frc.robot.commands.ClimberUp;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.HopperToOuttakeSubsystem;
import frc.robot.subsystems.IntakeExtentionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.OuttakeSubsystem;

// import frc.robot.subsystems.Climber;6
// import frc.robot.subsystems.Climber;

import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.Newton;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.*;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;





public class RobotContainer {

 

 
  // private SendableChooser<Command> chooser;
  private final PS5Controller m_PS5Controller = new PS5Controller(0);
  private final GenericHID m_subsystemsController = new GenericHID(1);

  // private final GenericHID joystick = new GenericHID(1);

  /* Drive Controls */
  private final int translationAxis = PS5Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS5Controller.Axis.kRightX.value;
  
  private final Trigger robotCentric = new JoystickButton(m_PS5Controller,PS5Controller.Button.kCross.value);
  private final Trigger oButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kCircle.value);
  private final Trigger sqrButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kSquare.value);
  private final Trigger triButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kTriangle.value);
  private final Trigger l1Button = new JoystickButton(m_PS5Controller, PS5Controller.Button.kL1.value);
  private final Trigger r1Button = new JoystickButton(m_PS5Controller, PS5Controller.Button.kR1.value);
  private final Trigger l2Button = new JoystickButton(m_PS5Controller, PS5Controller.Button.kL2.value);
  private final Trigger r2Button = new JoystickButton(m_PS5Controller, PS5Controller.Button.kR2.value);
  private final Trigger middleButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kTouchpad.value);
  private final Trigger psButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kPS.value);
  private final Trigger button1 = new JoystickButton(m_subsystemsController, 1);
  private final Trigger button2 = new JoystickButton(m_subsystemsController, 2);
  private final Trigger button3 = new JoystickButton(m_subsystemsController, 3);
  private final Trigger button4 = new JoystickButton(m_subsystemsController, 4);
  private final Trigger button5 = new JoystickButton(m_subsystemsController, 5);
  private final Trigger button6 = new JoystickButton(m_subsystemsController, 6);
  private final Trigger button7 = new JoystickButton(m_subsystemsController, 7);
  private final Trigger button8 = new JoystickButton(m_subsystemsController, 8);
  private final Trigger button9 = new JoystickButton(m_subsystemsController, 9);
  private final Trigger button11 = new JoystickButton(m_subsystemsController, 11);











  public final SwerveSubsystem m_SwerveSubsystem;


  private final IntakeExtentionSubsystem m_intakeExtentionSubsystem;
  private final OuttakeSubsystem m_outtakeSubsystem;
  private final HopperToOuttakeSubsystem m_hopperToOuttakeSubsystem;
  private final IntakeSubsystem m_intake;
  private final HopperSubsystem m_hopper;
  private final LimeLight vision;
  private final TeleopSwerve controllerSwerve;
  private final OuttakeCommand m_OuttakeCommand;

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
   
       /* Subsystems */
  m_SwerveSubsystem = new SwerveSubsystem();
  m_intakeExtentionSubsystem = new IntakeExtentionSubsystem();
  m_hopperToOuttakeSubsystem = new HopperToOuttakeSubsystem();
  m_intake = new IntakeSubsystem();
  m_hopper = new HopperSubsystem();
  vision = new LimeLight(m_SwerveSubsystem);
  m_outtakeSubsystem = new OuttakeSubsystem();
  m_OuttakeCommand = new OuttakeCommand(m_hopper, m_hopperToOuttakeSubsystem, m_outtakeSubsystem);
  controllerSwerve =  new TeleopSwerve(
          m_SwerveSubsystem,
          () -> -m_PS5Controller.getRawAxis(translationAxis),
          () -> -m_PS5Controller.getRawAxis(strafeAxis),
          () -> m_PS5Controller.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean());



   
    m_SwerveSubsystem.setDefaultCommand(controllerSwerve);
    

    configureBindings();
  }
  public Command nullAuto(){
    return null;
  }

 
  private void configureBindings() {
    oButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    button5.onTrue(new InstantCommand(() -> m_intakeExtentionSubsystem.setIntakeExtentionSpeed(0.2)));
    button5.onFalse(new InstantCommand(() -> m_intakeExtentionSubsystem.stopIntake()));
    button3.onTrue(new InstantCommand(() -> m_intakeExtentionSubsystem.setIntakeExtentionSpeed(-0.2)));
    button3.onFalse(new InstantCommand(() -> m_intakeExtentionSubsystem.stopIntake()));
    button1.onTrue(m_OuttakeCommand.onlyWhile(button1));
    button2.onTrue(new InstantCommand(() -> m_intake.setIntakeSpeed(0.4)));
    button2.onFalse(new InstantCommand(() -> m_intake.stopIntake()));
    button4.onTrue(new InstantCommand(() -> m_intake.setIntakeSpeed(1)));
    button4.onFalse(new InstantCommand(() -> m_intake.stopIntake()));
    button6.onTrue(new SequentialCommandGroup(new InstantCommand(() ->m_hopper.setHopperSpeed(0.3)),new InstantCommand(() -> m_hopperToOuttakeSubsystem.setHopperToOuttakeSpeed(-0.7)), new InstantCommand(() -> m_outtakeSubsystem.setOuttakeSpeed(1))));
    button6.onFalse(new SequentialCommandGroup(new InstantCommand(() -> m_hopper.stopHopper()),new InstantCommand(() -> m_hopperToOuttakeSubsystem.stopHopperToOuttake()), new InstantCommand(() -> m_outtakeSubsystem.stopOuttake())));
    triButton.onTrue(new RunCommand(() -> vision.faceTag()).onlyWhile(triButton));
    button11.onTrue(new InstantCommand(() -> m_outtakeSubsystem.setOuttakeSpeed(0.58)));
    // l1Button.onTrue(new InstantCommand(() -> m_intake.setIntakeSpeed(0.2)));
    // l1Button.onFalse(new InstantCommand(() -> m_intake.stopIntake()));
    // r1Button.onTrue(new InstantCommand(() -> m_hopper.setHopperSpeed(0.3)));
    // r1Button.onFalse(new InstantCommand(() -> m_hopper.stopHopper()));
    // l2Button.onTrue(new InstantCommand(() -> m_outtakeSubsystem.setOuttakeSpeed(0.6)));
    // l2Button.onFalse(new InstantCommand(() -> m_outtakeSubsystem.stopOuttake()));
    // r2Button.onTrue(new InstantCommand(() -> m_hopperToOuttakeSubsystem.setHopperToOuttakeSpeed(-0.8)));
    // r2Button.onFalse(new InstantCommand(() -> m_hopperToOuttakeSubsystem.stopHopperToOuttake()));
    // middleButton.onTrue(new RunCommand(() -> vision.faceTag()).onlyWhile(r2Button));
    // middleButton.onFalse((controllerSwerve));
    // psButton.onTrue(new InstantCommand(() -> m_hopper.setHopperSpeed(-0.3)));
    // psButton.onFalse(new InstantCommand(() -> m_hopper.stopHopper()));


  



   
  }

  public Command driveAndShoot () {
    return new SequentialCommandGroup(new RunCommand(() -> m_SwerveSubsystem.drive(new Translation2d(1, 0), 0, true, true)).withTimeout(2).andThen(m_OuttakeCommand));
  }

  public Command shootCommand() {
    return new InstantCommand(() -> m_outtakeSubsystem.setOuttakeSpeed(0.60)).alongWith(new InstantCommand(() -> m_hopperToOuttakeSubsystem.setHopperToOuttakeSpeed(-0.8)).alongWith(new InstantCommand(()-> m_hopper.setHopperSpeed(-0.5))));
  }



 

  public Command getAutonomousCommand() {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Back Left 45");
    // m_SwerveSubsystem.resetOdometry(path.getPreviewStartingHolonomicPose());
    // return AutoBuilder.followPath(path);


    return shootCommand();
  }
}
