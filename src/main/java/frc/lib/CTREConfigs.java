package frc.lib;


import com.ctre.phoenix6.configs.CANcoderConfiguration;



public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    // swerveCanCoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    // swerveCanCoderConfig.sensorDirection = Constants.SwerveConstants.canCoderInvert;
    // swerveCanCoderConfig.initializationStrategy =
    //     SensorInitializationStrategy.BootToAbsolutePosition;
    // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}