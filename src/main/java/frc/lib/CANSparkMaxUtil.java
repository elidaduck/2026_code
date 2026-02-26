package frc.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;

/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
  public enum Usage {

    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setCANSparkMaxBusUsage(
      SparkMax motor, Usage usage, boolean enableFollowing) {
    SparkMaxConfig configLowPeriod = new SparkMaxConfig();
    configLowPeriod.signals.primaryEncoderPositionPeriodMs(10);
    SparkMaxConfig configMediumPeriod = new SparkMaxConfig();
    configMediumPeriod.signals.primaryEncoderPositionPeriodMs(20);
    SparkMaxConfig configHighPeriod = new SparkMaxConfig();
    configHighPeriod.signals.primaryEncoderPositionPeriodMs(500);
    SparkMaxConfig config20 = new SparkMaxConfig();
    config20.signals.primaryEncoderPositionPeriodMs(20);

    SparkMaxConfig config500 = new SparkMaxConfig();
    config500.signals.primaryEncoderPositionPeriodMs(500);

    SparkMaxConfig config1000 = new SparkMaxConfig();
    config1000.signals.primaryEncoderPositionPeriodMs(1000);

    if (enableFollowing) {
      motor.configure(configLowPeriod, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else {
      motor.configure(configHighPeriod, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    if (usage == Usage.kAll) {
      //sets ussage to send all the frames of data yay
      motor.configure(config20, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.configure(config20, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.configure(config500, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else if (usage == Usage.kPositionOnly) {
      //only sends the position frames every 20 ms, saves on velocity and other status
      motor.configure(config1000, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.configure(config20, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.configure(config1000, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else if (usage == Usage.kVelocityOnly) {
      //only sends the velocity every 20 ms
      motor.configure(config20, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.configure(config1000, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.configure(config1000, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else if (usage == Usage.kMinimal) {
      //sends as little data as possible to save canbus ussage
      motor.configure(config500, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.configure(config500, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.configure(config500, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   */
  public static void setCANSparkMaxBusUsage(SparkMax motor, Usage usage) {
    setCANSparkMaxBusUsage(motor, usage, false);
  }
}