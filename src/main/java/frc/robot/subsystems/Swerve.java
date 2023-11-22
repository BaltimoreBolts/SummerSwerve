package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules;

  private final AHRS gyro;

  private final SwerveDriveOdometry m_odometry;

  private final Field2d m_dashboardField = new Field2d();

  private final List<DoubleLogEntry> m_moduleRawVelocityEntries = new ArrayList<DoubleLogEntry>();
  private final List<DoubleLogEntry> m_modulepercentPowerEntries = new ArrayList<DoubleLogEntry>();


  public Swerve() {
    gyro = new AHRS();

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };

    DataLog log = DataLogManager.getLog();
    for (int i = 0; i < 4; i++) {
      m_moduleRawVelocityEntries.add(new DoubleLogEntry(log, "/swerve/moduleRawVelocity[" + i + "]"));
      m_modulepercentPowerEntries.add(new DoubleLogEntry(log, "/swerve/moduleOutPower[" + i + "]"));
    }

    zeroGyro();

    m_odometry = new SwerveDriveOdometry(Constants.kSwerve.KINEMATICS, getYaw(), getStates());

    SmartDashboard.putData("Field", m_dashboardField);
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
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, m_odometry.getPoseMeters().getRotation())
        : new ChassisSpeeds(forwardBack, leftRight, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states);
    }, this).withName("SwerveDriveCommand");
  }

  public Command driveTestVelocity(DoubleSupplier velocity) {
    return new RunCommand(() -> {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      for (int i = 0; i < modules.length; i++) {
        states[i].speedMetersPerSecond = velocity.getAsDouble();
        modules[i].setStateTestVelocity(states[i]);
      }
    });
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

  public SwerveModulePosition[] getStates() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getState();
    }

    return currentStates;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }

  public void resetOdometry() {
    m_odometry.resetPosition(getYaw(), getStates(), new Pose2d());
  }

  public Command zeroGyroCommand() {
    return new InstantCommand(this::zeroGyro).withName("ZeroGyroCommand");
  }

  private void zeroGyro() {
    gyro.zeroYaw();
  }


  @Override
  public void periodic() {
    m_odometry.update(getYaw(), getStates());
    m_dashboardField.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("x_val odom", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y_val odom", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("angle odom", m_odometry.getPoseMeters().getRotation().getDegrees());

    SmartDashboard.putNumber("navX", gyro.getAngle());

    int i = 0;

    for (SwerveModule module : modules) {
      SmartDashboard.putNumber(String.format("Thrifty angle %d", module.moduleNumber), module.getThriftyAngle().getDegrees());
      SmartDashboard.putNumber(String.format("Max angle %d", module.moduleNumber), module.getSteerAngle().getDegrees());
      SmartDashboard.putNumber(String.format("Distance %d", module.moduleNumber), module.getDisance());
      SmartDashboard.putNumber(String.format("Rot %d", module.moduleNumber), module.getDriveRot());

      m_moduleRawVelocityEntries.get(i).append(module.getDriveRawVelocity());
      m_modulepercentPowerEntries.get(i).append(module.getDriveOutputPower());
      i++;
    }

    SmartDashboard.putNumber("percentPower", modules[0].getDriveOutputPower());
    SmartDashboard.putNumber("rawVelocity", modules[0].getDriveRawVelocity());
  }
  
}
