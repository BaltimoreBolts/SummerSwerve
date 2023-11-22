package frc.robot.utils;

//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.SensorInitializationStrategy;
//import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.AnalogPotentiometer; // FOR ANALOG ENCODER (Maybe)
import edu.wpi.first.wpilibj.AnalogEncoder; // FOR ANALOG ENCODER
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;


public class SwerveModule {
  public final int moduleNumber;

  private final CANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkMaxPIDController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;

  private final CANSparkMax angleMotor;
  private final RelativeEncoder angleEncoder;
  private final SparkMaxPIDController anglePID;
  
  private final AnalogEncoder m_turningEncoder; // FOR ANALOG ENCODER
  private final Rotation2d m_thriftyOffsetDegrees;

  private Rotation2d m_startupOffset;

  private Rotation2d lastAngle;

  public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
    this.moduleNumber = moduleNumber;
    
    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getPIDController();
    driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    angleMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();
    anglePID = angleMotor.getPIDController();

    m_turningEncoder = new AnalogEncoder(constants.thriftyEncoderID);
    m_thriftyOffsetDegrees = constants.thriftyOffsetDegrees;

    configureDevices();
    lastAngle = getSteerAngle();
  }

  public void setState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerAngle());

    double speed = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
    setDrivePower(speed);

    Rotation2d angle = Math.abs(state.speedMetersPerSecond) <= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
      ? lastAngle
      : state.angle;

    setSteerAngle(angle);
    lastAngle = angle;
  }

  public void setStateTestVelocity(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerAngle());

    setDriveVelocity(state.speedMetersPerSecond);

    Rotation2d angle = Math.abs(state.speedMetersPerSecond) <= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
      ? lastAngle
      : state.angle;

    setSteerAngle(angle);
    lastAngle = angle;
  }

// TODO - Implement
//  public SwerveModuleState getState() {
//    double velocity = driveEncoder.getVelocity();
//    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
//    return new SwerveModuleState(velocity, rot);
//  }

  public Rotation2d getSteerAngle() {
    return Rotation2d.fromRotations(angleMotor.getEncoder().getPosition() * Constants.kSwerve.k_turnGearRatio).plus(m_startupOffset);
  }

  public Rotation2d getSteerRawAngle() {
    return Rotation2d.fromRotations(angleMotor.getEncoder().getPosition()).times(Constants.kSwerve.k_turnGearRatio);
  }

  public Rotation2d getThriftyAngle() {
    return Rotation2d.fromRotations(m_turningEncoder.getAbsolutePosition());
  }

  public double getDriveRot() {
    return driveEncoder.getPosition() / Constants.kSwerve.DRIVE_GEAR_RATIO;
  }

  public double getDisance() {
    return driveEncoder.getPosition() / Constants.kSwerve.DRIVE_GEAR_RATIO * Constants.kSwerve.WHEEL_CIRCUMFERENCE;
  }

  public double getDriveRawVelocity() {
    return driveEncoder.getVelocity();
  };

  public double getDriveOutputPower() {
    return driveMotor.getAppliedOutput();
  }

  public SwerveModulePosition getState() {
    return new SwerveModulePosition(getDisance(), getSteerAngle());
  }

  private void configureDevices() {
    // Drive motor configuration.
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(Constants.kSwerve.DRIVE_MOTOR_INVERSION);
    driveMotor.setIdleMode(Constants.kSwerve.DRIVE_IDLE_MODE);
    driveMotor.setOpenLoopRampRate(Constants.kSwerve.OPEN_LOOP_RAMP);
    driveMotor.setClosedLoopRampRate(Constants.kSwerve.CLOSED_LOOP_RAMP);
    driveMotor.setSmartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);
 
    drivePID.setP(Constants.kSwerve.DRIVE_KP);
    drivePID.setI(Constants.kSwerve.DRIVE_KI);
    drivePID.setD(Constants.kSwerve.DRIVE_KD);
    drivePID.setFF(Constants.kSwerve.DRIVE_KF);
 
    driveEncoder.setPositionConversionFactor(1);
    driveEncoder.setVelocityConversionFactor(1);
    driveEncoder.setPosition(0);

    // Angle motor configuration.
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(Constants.kSwerve.ANGLE_MOTOR_INVERSION);
    angleMotor.setIdleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
    angleMotor.setSmartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

    anglePID.setP(Constants.kSwerve.ANGLE_KP);
    anglePID.setI(Constants.kSwerve.ANGLE_KI);
    anglePID.setD(Constants.kSwerve.ANGLE_KD);
    anglePID.setFF(Constants.kSwerve.ANGLE_KF);

    anglePID.setPositionPIDWrappingEnabled(true);
    anglePID.setPositionPIDWrappingMaxInput(1.0 / Constants.kSwerve.k_turnGearRatio);
    anglePID.setPositionPIDWrappingMinInput(0.0);

    angleEncoder.setPositionConversionFactor(1.0);
    angleEncoder.setVelocityConversionFactor(1.0);
    configureEncoders();
  }

  private void configureEncoders() {
    m_startupOffset = (getThriftyAngle().minus(m_thriftyOffsetDegrees)).minus(getSteerRawAngle());
  }

  private void setSteerAngle(Rotation2d angle_rad) {
    double position = (angle_rad.minus(m_startupOffset).getRotations() / Constants.kSwerve.k_turnGearRatio);
    anglePID.setReference(position, ControlType.kPosition);
  }

  private void setDrivePower(double pctSpeed){
    drivePID.setReference(pctSpeed, CANSparkMax.ControlType.kDutyCycle);
  }

  private void setDriveVelocity(double velocity) {
    drivePID.setReference(velocity, ControlType.kVelocity);
  }

  public void stop() {
    angleMotor.set(0);
    driveMotor.set(0);
  }
}
