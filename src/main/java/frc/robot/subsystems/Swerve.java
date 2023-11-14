package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules;

  private final SwerveDriveOdometry swerveOdometry;

  private final AHRS gyro;

  public Swerve() {
    gyro = new AHRS();

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.kSwerve.KINEMATICS, getYaw(), getPositions());
    zeroGyro();
  }

  /** 
   * This is called a command factory method, and these methods help reduce the
   * number of files in the command folder, increasing readability and reducing
   * boilerplate. 
   * 
   * Double suppliers are just any function that returns a double.
   */
  public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isFieldRelative) {
    return new RunCommand(() -> {
      // Grabbing input from suppliers.
      double forwardBack = forwardBackAxis.getAsDouble();
      double leftRight = leftRightAxis.getAsDouble();
      double rotation = rotationAxis.getAsDouble();

      // Adding deadzone.
      forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
      leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
      rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;

      // Converting to m/s
      forwardBack *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND; 
      leftRight *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      rotation *= Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;

      // Get desired module states.
      ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
        : new ChassisSpeeds(forwardBack, leftRight, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states);
    }, this).withName("SwerveDriveCommand");
  }

  /** To be used by auto. Use the drive method during teleop. */
  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, false);
  }

  private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    // Makes sure the module states don't exceed the max speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[modules[i].moduleNumber]);
    }
  }

  // TODO implement
  // public SwerveModuleState[] getStates() {
  //   SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
  //   for (int i = 0; i < modules.length; i++) {
  //     currentStates[i] = modules[i].getState();
  //   }

  //   return currentStates;
  // }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getPosition();
    }

    return currentStates;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }


  public Command zeroGyroCommand() {
    return new InstantCommand(this::zeroGyro).withName("ZeroGyroCommand");
  }

  private void zeroGyro() {
    gyro.zeroYaw();
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());

    for (SwerveModule module : modules) {
      SmartDashboard.putNumber(String.format("Thrifty angle %d", module.moduleNumber), module.getThriftyAngle().getDegrees());
      SmartDashboard.putNumber(String.format("Max angle %d", module.moduleNumber), module.getSteerAngle().getDegrees());
      SmartDashboard.putNumber(String.format("Distance %d", module.moduleNumber), module.getDisance());
      SmartDashboard.putNumber("navX", gyro.getAngle());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    for (SwerveModule module : modules) {
      // TODO implement
      // builder.addStringProperty(
      //   String.format("Module %d", module.moduleNumber),
      //   () -> {
      //     SwerveModuleState state = module.getState();
      //     return String.format("%6.2fm/s %6.3fdeg", state.speedMetersPerSecond, state.angle.getDegrees());
      //   },
      //   null);

      // builder.addDoubleProperty(
      //   String.format("Module angle %d", module.moduleNumber),
      //   () -> module.getSteerAngle().getDegrees(),
      //   null);

        
      // builder.addDoubleProperty(
      //   String.format("Angle %d", module.moduleNumber),
      //   () -> module.getThriftyAngle().getDegrees(),
      //   null);

      SmartDashboard.putNumber(String.format("Thrifty angle %d", module.moduleNumber), module.getThriftyAngle().getDegrees());
      SmartDashboard.putNumber(String.format("Max angle %d", module.moduleNumber), module.getSteerAngle().getDegrees());
    }
  }
}
