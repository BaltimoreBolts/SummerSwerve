package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.SwerveModuleConstants;

/**
 * This class contains values that remain constant while the robot is running.
 * 
 * It's split into categories using subclasses, preventing too many members from
 * being defined on one class.
 */
public class Constants {
  /** All joystick, button, and axis IDs. */
  public static class kControls {
    public static final double AXIS_DEADZONE = 0.1;

    public static final int DRIVE_JOYSTICK_ID = 0;

    public static final int TRANSLATION_X_AXIS = XboxController.Axis.kLeftX.value;
    public static final int TRANSLATION_Y_AXIS = XboxController.Axis.kLeftY.value;
    public static final int ROTATION_AXIS = XboxController.Axis.kRightX.value;

    public static final int GYRO_RESET_BUTTON = XboxController.Button.kY.value;

    // Prevent from acclerating/decclerating to quick
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(4);
  }

  /** All swerve constants. */
  public static class kSwerve {
    /** Constants that apply to the whole drive train. */                          //THEY HAD IT AT 19.5 SO SWERVE PID VALUES ARE PROBABLY OFF
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.75); // Width of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(22.75); // Length of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double k_turnGearRatio = 7.0/150.0;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // 0 - back right
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // 1 - back left
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // 2 - front left
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0) // 3 - front right
    );

    public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = (150/7) / 1.0; // THEY HAD IT AT 12.8:1 -- not sure why
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int ANGLE_CURRENT_LIMIT = 25;

    /** Drive motor characterization. */
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 1.68E-4;
    public static final double DRIVE_KA = 0.0;

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.00025;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = DRIVE_KV;

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 0.25;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.0;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 5.0;

    /** Inversions. */

    public static final boolean DRIVE_MOTOR_INVERSION = false;
    public static final boolean ANGLE_MOTOR_INVERSION = true;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

    /** //NOTE FROM PERVIOUS WRITERS WHO USED CAN CODERS
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */

    //mod 0 = back right
    public static final SwerveModuleConstants MOD_0_Constants = new SwerveModuleConstants(

      3,
      2,
      0,
      Rotation2d.fromDegrees(260.9)
    );
    //mod 1 = back left
    public static final SwerveModuleConstants MOD_1_Constants = new SwerveModuleConstants(
      5,
      4,
      1,
      Rotation2d.fromDegrees(295.6)
    );

    //mod 2 = front left
    public static final SwerveModuleConstants MOD_2_Constants = new SwerveModuleConstants(
      7,
      6,
      2,
      Rotation2d.fromDegrees(293.4)
    );

    //mod 3 = front right
    public static final SwerveModuleConstants MOD_3_Constants = new SwerveModuleConstants(
      9,
      8,
      3,
      Rotation2d.fromDegrees(314.2)
    );
  }

  public static class kAuto {
    /** PID Values. */
    public static final double X_CONTROLLER_KP = 1.0;
    public static final double Y_CONTROLLER_KP = 1.0;
    public static final double THETA_CONTROLLER_KP = 1.0;
    
    /** Constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2.0;
    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 5.0;
  }
}
