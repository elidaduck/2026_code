
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

  // -------------------------------------------------------------------------
  // Physical constants
  // -------------------------------------------------------------------------

  /** Launch angle above horizontal, in radians. (70 degrees) */
  private static final double LAUNCH_ANGLE_RAD = Math.toRadians(70.0);

  /** Height of the shooter release point above the carpet, in meters. (60 cm) */
  private static final double LAUNCH_HEIGHT_M = 0.60;

  /**
   * Height of the hub opening above the carpet, in meters.
   * From the 2026 REBUILT field manual: front edge of hub opening = 72 in = 1.8288 m.
   */
  private static final double HUB_OPENING_HEIGHT_M = 1.8288;

  /**
   * Vertical distance the ball must climb from shooter to hub opening.
   * delta_h = hub_height - shooter_height
   */
  private static final double DELTA_H_M = HUB_OPENING_HEIGHT_M - LAUNCH_HEIGHT_M; // 1.2288 m

  /** Standard gravity, m/s^2. */
  private static final double G = 9.81;

  // -------------------------------------------------------------------------
  // Motor / wheel constants
  // -------------------------------------------------------------------------

  /**
   * Kraken (TalonFX) free-spin speed in RPM.
   * The SparkMax on ID 14 drives a Kraken (TalonFX). Free speed 6000 RPM.
   */
  private static final double KRAKEN_FREE_RPM = 6000.0;

  /**
   * Shooter wheel diameter in meters.
   * Estimated from photos: orange + green flywheel wheels look ~4 inches.
   * ADJUST this if your actual wheel diameter is different.
   */
  private static final double WHEEL_DIAMETER_M = 0.1016; // 4 inches

  /** Wheel radius in meters. */
  private static final double WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0;

  /**
   * Ball exit speed efficiency factor.
   * For a dual-wheel compression shooter with a foam ball, ball speed is roughly
   * the orange wheel surface speed. Derived from geometry: the green wheels (2.25")
   * spin at the same RPM as the orange (4") wheels, so average contact speed is
   * (r_orange + r_green)/2 / r_orange = 78.1% of orange surface speed, minus
   * ~7% slip loss and ~2.5% bearing loss = 0.71. Tune up/down based on
   * whether shots are consistently long or short.
   */
  private static final double EFFICIENCY = 0.71;

  /**
   * Maximum achievable ball speed (m/s) at 100% motor output.
   *   v_max = Kraken_free_rpm * 2π * r / 60 * efficiency
   */
  private static final double MAX_BALL_SPEED_MS =
      (KRAKEN_FREE_RPM * 2.0 * Math.PI * WHEEL_RADIUS_M / 60.0) * EFFICIENCY; // ≈ 22.6 m/s

  /**
   * Distance from the hub AprilTag face to the hub scoring opening (hub center),
   * in meters. The Limelight reports distance to the TAG FACE; the actual target
   * (hub opening) is this much farther in. Derived from hub outer radius ≈ 0.61 m.
   */
  private static final double HUB_FACE_DEPTH_M = 0.61;

  /**
   * Minimum and maximum valid shooting distances, in meters (shooter to hub opening).
   * Used to clamp the physics calculation to sensible field positions.
   */
  private static final double MIN_SHOOT_DIST_M = 0.8;
  private static final double MAX_SHOOT_DIST_M = 6.0;

  // -------------------------------------------------------------------------

  private final SparkMax outtakeMotor;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    outtakeMotor = new SparkMax(14, MotorType.kBrushless);
  }

  /**
   * Sets the outtake wheel to a fixed percent output (0.0 to 1.0).
   * Used for manual control; call {@link #setDistanceSpeed()} for auto-calculated speed.
   */
  public void setOuttakeSpeed(double speed) {
    outtakeMotor.set(speed);
  }

  /**
   * Reads the Limelight distance and calculates the required motor speed from
   * projectile physics, then applies it to the outtake motor.
   *
   * Call this repeatedly from a command's execute() while the Limelight has a
   * valid hub tag target. The motor will spin at exactly the speed needed to
   * arc the ball into the hub from the current distance.
   *
   * @return the motor percent output that was applied (0.0 = no valid target)
   */
  public double setDistanceSpeed() {
    double speed = getDistanceSpeed();
    if (!Double.isNaN(speed)) {
      outtakeMotor.set(speed);
    }
    return speed;
  }

  /**
   * Calculates the required motor percent output to reach the hub from the
   * current Limelight-measured distance, using projectile physics.
   *
   * <h3>Physics derivation</h3>
   * Projectile equations (ignoring air resistance, which is acceptable for a
   * dense foam ball at these short distances):
   * <pre>
   *   x(t) = v0 * cos(θ) * t
   *   y(t) = v0 * sin(θ) * t − ½·g·t²
   * </pre>
   * At time t_hit, x = d (horizontal distance) and y = Δh (height gain needed):
   * <pre>
   *   t_hit = d / (v0 · cos θ)
   *   Δh = d·tan θ − (g·d²) / (2·v0²·cos²θ)
   * </pre>
   * Solving for v0:
   * <pre>
   *   v0 = d · sqrt(g / 2) / (cos θ · sqrt(d·tan θ − Δh))
   * </pre>
   * Motor percent output is then:
   * <pre>
   *   percent = v0 / (Kraken_free_rpm * 2π * r / 60 * efficiency)
   * </pre>
   *
   * @return required motor percent (0.0–1.0), or {@code Double.NaN} if no valid
   *         target is detected or the distance is out of range.
   */
  public double getDistanceSpeed() {
    // Get planar distance from Limelight to the hub AprilTag face (meters).
    double distToTag = LimeLight.getDistance();

    if (Double.isNaN(distToTag)) {
      // No valid Limelight target — don't fire.
      SmartDashboard.putString("Shooter State", "No LL target");
      return Double.NaN;
    }

    // The tag sits on the outer face of the hub. The scoring opening is
    // HUB_FACE_DEPTH_M further inward. Use that as the actual shooting distance.
    double d = distToTag - HUB_FACE_DEPTH_M;

    if (d < MIN_SHOOT_DIST_M || d > MAX_SHOOT_DIST_M) {
      SmartDashboard.putString("Shooter State", "Out of range: " + String.format("%.2f", d) + " m");
      SmartDashboard.putNumber("Shooter Distance (m)", d);
      return Double.NaN;
    }

    // ---------- Physics: solve for required launch speed ----------
    double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);
    double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);

    // The term under the square root: (d·tanθ − Δh) must be positive.
    // If it's ≤ 0, the angle is too shallow or the target is below the launcher.
    double underRoot = d * tanTheta - DELTA_H_M;
    if (underRoot <= 0) {
      SmartDashboard.putString("Shooter State", "Angle insufficient at this distance");
      return Double.NaN;
    }

    // Required ball exit speed (m/s)
    double v0 = d * Math.sqrt(G / 2.0) / (cosTheta * Math.sqrt(underRoot));

    // Convert ball speed to motor percent output
    //   actual_rpm ≈ free_rpm * percent  (linear approximation)
    //   ball_speed  = actual_rpm * 2π*r/60 * efficiency
    //   => percent = ball_speed / MAX_BALL_SPEED_MS
    double percent = v0 / MAX_BALL_SPEED_MS;

    // Clamp to [0, 1]
    percent = Math.min(1.0, Math.max(0.0, percent));

    // Publish telemetry
    SmartDashboard.putNumber("Shooter Distance (m)", d);
    SmartDashboard.putNumber("Shooter Required v0 (m/s)", v0);
    SmartDashboard.putNumber("Shooter Motor %", percent);
    SmartDashboard.putString("Shooter State", "Active");

    return percent;
  }

  public void stopOuttake() {
    outtakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // Publish live distance every loop for SmartDashboard monitoring
    double rawDist = LimeLight.getDistance();
    if (!Double.isNaN(rawDist)) {
      SmartDashboard.putNumber("LL Raw Distance to Tag (m)", rawDist);
      SmartDashboard.putNumber("Shooter Distance to Hub (m)", rawDist - HUB_FACE_DEPTH_M);
    }
  }
}
