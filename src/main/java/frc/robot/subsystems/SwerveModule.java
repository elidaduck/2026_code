// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.CANCoderUtil;
import frc.lib.CANSparkMaxUtil;
import frc.lib.OnboardModuleState;
import frc.lib.SwerveModuleConstants;
import frc.lib.CANCoderUtil.CCUsage;
import frc.lib.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    public double m_angleKP;
    public double m_angleKI;
    public double m_angleKD;
    public double m_angleKFF;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SparkMax angleMotor;
    private SparkMax driveMotor;

    private SparkMaxConfig angleConfig;
    private SparkMaxConfig driveConfig;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;

    private CANcoder angleEncoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController angleController;

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);
    private final SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(
        Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);
    // creates a feedforward for the swerve drive. feedforward does 90% of the work,
    // estimating stuff
    // PID fixes the error

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.m_angleKP = moduleConstants.angleKP;
        this.m_angleKI = moduleConstants.angleKI;
        this.m_angleKD = moduleConstants.angleKD;
        this.m_angleKFF = moduleConstants.angleKFF;
        angleOffset = moduleConstants.angleOffset;
        angleConfig = new SparkMaxConfig();
        driveConfig = new SparkMaxConfig();
        angleConfig.inverted(Constants.SwerveConstants.angleInvert).idleMode(Constants.SwerveConstants.angleNeutralMode);
        driveConfig.inverted(Constants.SwerveConstants.driveInvert).idleMode(Constants.SwerveConstants.driveNeutralMode);
        angleConfig.encoder.positionConversionFactor(Constants.SwerveConstants.angleConversionFactor); //maybe here is the problem
        driveConfig.encoder.positionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor) 
        .velocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
        angleConfig.closedLoop.pidf(m_angleKP, m_angleKI, m_angleKD, m_angleKFF); // sets PIDF values
        driveConfig.closedLoop.pidf(Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI,
         Constants.SwerveConstants.driveKD, Constants.SwerveConstants.driveKFF);
        angleConfig.smartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit)
        .voltageCompensation(Constants.SwerveConstants.voltageComp);
        driveConfig.smartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit)
        .voltageCompensation(Constants.SwerveConstants.voltageComp);
        


        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        
        configAngleEncoder();

        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getClosedLoopController();
    // Do NOT reassign angleConfig here; it was configured above with PID and encoder
    // conversion factors. Reassigning would clear those settings.

        configAngleMotor();

        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
    // Do NOT reassign driveConfig here; it was configured above with PID and encoder
    // conversion factors. Reassigning would clear those settings.
        configDriveMotor();

        lastAngle = getState().angle;

        resetToAbsolute();



    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }
    public boolean isOptimizable(SwerveModuleState desiredState) {
        Rotation2d setPointDirection = desiredState.angle;
        Rotation2d currentDirection = getState().angle;
        double deltaDirection = Math.cos(setPointDirection.minus(currentDirection).getRadians());

        // If the dot product is negative, reversing the wheel direction may be beneficial
        return deltaDirection < 0;
    }
    /** Normalize angle (degrees) to range (-180, 180] */
    private double wrapDegTo180(double angle) {
        double a = ((angle + 180.0) % 360.0);
        if (a < 0) a += 360.0;
        return a - 180.0;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous
        // controller which
        // REV supports this now so dont have to worry with rev, but need some funky
        // configs i dont want to do
        // have to be sad with falcons but thats what you get for giving money to Tony
        desiredState.optimize(getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }
    

   private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
        // when not taking feedback
        double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
        driveMotor.set(percentOutput);
    } else {
        driveController.setSetpoint(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }
}
    public void setAngle(SwerveModuleState desiredState) {
        // read continuous encoder position (degrees)
        double currentDeg = integratedAngleEncoder.getPosition();

        // desired angle from WPILib SwerveModuleState (degrees)
        double desiredDeg = desiredState.angle.getDegrees(); // ensure this is in -180..180 or 0..360

        // compute minimal delta and continuous setpoint
        double delta = wrapDegTo180(desiredDeg - currentDeg);
        double setpointDeg = currentDeg + delta;

        // optional deadband to avoid jitter for tiny deltas
        final double deadbandDeg = 1.0; // tune between 0.5..3.0
        if (Math.abs(delta) < deadbandDeg) {
            setpointDeg = currentDeg;
        }

        // command the controller with the continuous setpoint
        angleController.setSetpoint(setpointDeg, ControlType.kPosition, ClosedLoopSlot.kSlot0);



        lastAngle = getState().angle;
    }

    public void resetToAbsolute() {
        double absoluteDeg = getCanCoder().getDegrees() - angleOffset.getDegrees(); // maybe 0..360
        // normalize CANcoder value into (-180,180] to match our convenience
        absoluteDeg = wrapDegTo180(absoluteDeg);

        double currentDeg = integratedAngleEncoder.getPosition();
        // pick the equivalent of absoluteDeg that is closest to currentDeg
        double adjusted = currentDeg + wrapDegTo180(absoluteDeg - currentDeg);
        integratedAngleEncoder.setPosition(adjusted);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private void configAngleEncoder() {
        // angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {

        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);

        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Timer.delay(1);

        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // factory resets the spark max

        // full utilisation on the can loop hell yea
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        // sets current limit

        // burns to spark max
;
        // resets encoder position to 0
        driveEncoder.setPosition(0.0);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        
    }

    

}
