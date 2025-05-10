// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

// WPI Math Libraries
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// Phoenix6 Libraries
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// 4265 Constants
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
  // Define drive and azimuth motors
  private final TalonFX m_driveMotor;
  private final TalonFX m_azimuthMotor;

  // TODO: Do we want to use the constants file to store these values?
  private final PIDController m_drivePIDController =
      new PIDController(0.07386364, 0.4166666, 0.0);

  // TODO: Do we want to use the constants file to store these values?
  private final ProfiledPIDController m_azimuthPIDController =
      new ProfiledPIDController(
        0.13204545,
        0,
        0,
        new TrapezoidProfile.Constraints(
          SwerveModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          SwerveModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared
        )
      );

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorId The channel of the drive motor.
   * @param azimuthMotorId The channel of the azimuth motor.
   * @param driveMotorConfiguration Drive motor configuration for the specific swerve module.
   * @param azimuthMotorConfiguration Azimuth motor configuration for the specific swerve module.
   */
  public SwerveModule(
      int driveMotorId,
      int azimuthMotorId,
      TalonFXConfiguration driveMotorConfiguration,
      TalonFXConfiguration azimuthMotorConfiguration)
  {
    // Instantiate the drive and azimuth motors for the swerve module
    m_driveMotor = new TalonFX(driveMotorId);
    m_azimuthMotor = new TalonFX(azimuthMotorId);

    // Reset all the encoders
    resetEncoders();

    // Setup the drive motor configuration
    m_driveMotor.getConfigurator().apply(driveMotorConfiguration);

    // Setup the azimuth motor configuration
    m_azimuthMotor.getConfigurator().apply(azimuthMotorConfiguration);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_azimuthPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    /* SwerveModuleState
     * expects the following:
     * Double speedMetersPerSecond - The speed of the wheel of the module.
     * Rotation2d angle - The angle of the module.
     */

    /* Calculating speedMetersPerSecond
     * getRotorVelocity - returns RPS (rotations per second)
     * driveGearRatio - is the gear ratio for the drive motor to the wheel
     * kWheelDiameterMeters - radius of the wheel in meters
     * 1. Convert motor shaft rps to output shaft (wheel) rps = m_driveMotor.getRotorVelocity().getValueAsDouble() * driveGearRatio
     * 2. Calculate wheel distance per rotation = Math.PI * kWheelDiameterMeters
     * 3. multiply results from step 1 and step 2
     */

     /* Calculating angle
     * Rotation2d is expecting a value of radians (from the wheel perspective)
     * To achieve this, we would need the following:
     * getRotorPosition - returns the Position of the motor rotor (rotations).
     * m_azimuthRatio - is the gear ratio for the azimuth motor to the wheel
     * 1. Convert motor shaft angle to output shaft (wheel) angle = m_azimuthMotor.getRotorPosition().getValueAsDouble() * azimuthGearRatio
     * 2. Convert this value to radians (multiply result from step 1 by 2 * PI)
     */
    return new SwerveModuleState(
      (m_driveMotor.getRotorVelocity().getValueAsDouble() * SwerveModuleConstants.driveGearRatio) * (Math.PI * SwerveModuleConstants.kWheelDiameterMeters),
      new Rotation2d(m_azimuthMotor.getRotorPosition().getValueAsDouble() * SwerveModuleConstants.azimuthGearRatio * 2 * Math.PI)
    );
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
     /* SwerveModulePosition
     * expects the following:
     * Double distanceMeters - The distance measured by the wheel of the module.
     * Rotation2d angle - The angle of the module.
     */

     /* Calculating distanceMeters
     * getRotorPosition - returns rotations of motor rotor
     * driveGearRatio - is the gear ratio of the drive motor to the wheel
     * kWheelRadius - radius of the wheel in meters
     * 1. Convert motor shaft rotations to output shaft (wheel) rotations = m_driveMotor.getRotorPosition().getValueAsDouble() * driveGearRatio
     * 2. Calculate wheel distance per rotation = Math.PI * kWheelDiameterMeters
     * 3. multiply results from step 1 and step 2
     */

     /* Calculating angle
     * Rotation2d is expecting a value of radians (from the wheel perspective)
     * To achieve this, we would need the following:
     * getRotorPosition - returns the Position of the motor rotor (rotations).
     * azimuthGearRatio - is the gear ratio of the azimuth motor to the wheel
     * 1. Convert motor shaft angle to output shaft (wheel) angle = m_azimuthMotor.getRotorPosition().getValueAsDouble() * azimuthGearRatio
     * 2. Convert this value to radians (multiply result from step 1 by 2 * PI)
     */
    return new SwerveModulePosition(
      (m_driveMotor.getRotorPosition().getValueAsDouble() * SwerveModuleConstants.driveGearRatio) * (Math.PI * SwerveModuleConstants.kWheelDiameterMeters),
      new Rotation2d(m_azimuthMotor.getRotorPosition().getValueAsDouble() * SwerveModuleConstants.azimuthGearRatio * 2 * Math.PI)
    );
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Determine the rotation of the swerve
    var encoderRotation = new Rotation2d(m_azimuthMotor.getRotorPosition().getValueAsDouble() * SwerveModuleConstants.azimuthGearRatio * 2 * Math.PI);

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(
          (m_driveMotor.getRotorVelocity().getValueAsDouble() * SwerveModuleConstants.driveGearRatio) * (Math.PI * SwerveModuleConstants.kWheelDiameterMeters),
          state.speedMetersPerSecond);

    // Calculate the azimuth motor output from the azimuth PID controller.
    final double azimuthOutput =
        m_azimuthPIDController.calculate(
          m_azimuthMotor.getRotorPosition().getValueAsDouble() * SwerveModuleConstants.azimuthGearRatio * 2 * Math.PI,
          state.angle.getRadians());

    // Set the drive and azimuth motor output from the PID controllers.
    m_driveMotor.set(driveOutput);
    m_azimuthMotor.set(azimuthOutput);
  }

  /**
   * Zeroes all the SwerveModule encoders.
  */
  public void resetEncoders() {
    m_driveMotor.setPosition(0.0);
    m_azimuthMotor.setPosition(0.0);
  }

  /**
   * setDriveVoltage is used for the sysId running.
   * 
   * @param voltage
   */
  public void setDriveVoltage(double voltage) {
    m_driveMotor.setVoltage(voltage);
    m_azimuthMotor.setVoltage(0.0);
  }

  /**
   * Returns the drive motor voltage.
   */
  public StatusSignal<Double> getDriveVoltage() {
    return m_driveMotor.getMotorVoltage();
  }

  /**
   * Returns the drive motor position.
   */
  public double getDrivePosition() {
    return (m_driveMotor.getRotorPosition().getValueAsDouble() * SwerveModuleConstants.driveGearRatio) * (Math.PI * SwerveModuleConstants.kWheelDiameterMeters);
  }

  /**
   * Returns the swerve drive motor velocity.
   */
  public double getDriveVelocity() {
    return (m_driveMotor.getRotorVelocity().getValueAsDouble() * SwerveModuleConstants.driveGearRatio) * (Math.PI * SwerveModuleConstants.kWheelDiameterMeters);
  }
}
