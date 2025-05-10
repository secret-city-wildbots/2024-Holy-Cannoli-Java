// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

// Phoenix6 Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;

// Path Planner Libraries
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

// WPI Math Libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

// WPI Unit Libraries
// import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.*;

// WPI Driver Station and Smart Dashboard Libraries
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// WPI Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// 4265 Constants
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModule0DriveConfiguration;
import frc.robot.Constants.SwerveModule1DriveConfiguration;
import frc.robot.Constants.SwerveModule2DriveConfiguration;
import frc.robot.Constants.SwerveModule3DriveConfiguration;
import frc.robot.Constants.SwerveModule0AzimuthConfiguration;
import frc.robot.Constants.SwerveModule1AzimuthConfiguration;
import frc.robot.Constants.SwerveModule2AzimuthConfiguration;
import frc.robot.Constants.SwerveModule3AzimuthConfiguration;

public class SwerveDrive extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightAzimuthMotorPort,
          getSwerveDriveMotorConfiguration(0),
          getSwerveAzimuthMotorConfiguration(0));

  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftAzimuthMotorPort,
          getSwerveDriveMotorConfiguration(1),
          getSwerveAzimuthMotorConfiguration(1));

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftAzimuthMotorPort,
          getSwerveDriveMotorConfiguration(2),
          getSwerveAzimuthMotorConfiguration(2));

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightAzimuthMotorPort,
          getSwerveDriveMotorConfiguration(3),
          getSwerveAzimuthMotorConfiguration(3));

  // The gyro sensor
  private final Pigeon2 m_pigeon = new Pigeon2(DriveConstants.kPigeonPort);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_pigeon.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  // Sysid initialization
  private final SysIdRoutine sysId;

  // Mutable measures for logging data
  private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.mutable(MetersPerSecond.of(0));
  // private final MutableMeasure<Angle> angularDistance = MutableMeasure.mutable(Rotations.of(0));
  // private final MutableMeasure<Velocity<Angle>> angularVelocity = MutableMeasure.mutable(RotationsPerSecond.of(0));

  /** Creates a new DriveSubsystem. */
  public SwerveDrive() {
    // Configure the drive motor SysId routine
    sysId = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        // Drive mechanism - voltage setter
        (Measure<Voltage> volts) -> {
            m_frontRight.setDriveVoltage(volts.in(Volts));
            m_frontLeft.setDriveVoltage(volts.in(Volts));
            m_rearLeft.setDriveVoltage(volts.in(Volts));
            m_rearRight.setDriveVoltage(volts.in(Volts));

        },
        // Log drive data
        log -> {
            log.motor("m_frontRight")
                .voltage(appliedVoltage.mut_replace(m_frontRight.getDriveVoltage().getValueAsDouble(), Volts))
                .linearPosition(distance.mut_replace(m_frontRight.getDrivePosition(), Meters))
                .linearVelocity(velocity.mut_replace(m_frontRight.getDriveVelocity(), MetersPerSecond));
            log.motor("m_frontLeft")
                .voltage(appliedVoltage.mut_replace(m_frontLeft.getDriveVoltage().getValueAsDouble(), Volts))
                .linearPosition(distance.mut_replace(m_frontLeft.getDrivePosition(), Meters))
                .linearVelocity(velocity.mut_replace(m_frontLeft.getDriveVelocity(), MetersPerSecond));
            log.motor("m_rearLeft")
                .voltage(appliedVoltage.mut_replace(m_rearLeft.getDriveVoltage().getValueAsDouble(), Volts))
                .linearPosition(distance.mut_replace(m_rearLeft.getDrivePosition(), Meters))
                .linearVelocity(velocity.mut_replace(m_rearLeft.getDriveVelocity(), MetersPerSecond));
            log.motor("m_rearRight")
                .voltage(appliedVoltage.mut_replace(m_rearRight.getDriveVoltage().getValueAsDouble(), Volts))
                .linearPosition(distance.mut_replace(m_rearRight.getDrivePosition(), Meters))
                .linearVelocity(velocity.mut_replace(m_rearRight.getDriveVelocity(), MetersPerSecond));
        },
        // Drive subsystem
        this
      )
    );

    // Configure Path Planner Code
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID
        new PIDConstants(5.0, 0.0, 0.0), // Angle PID
        5, // Max speed in m/s
        Units.inchesToMeters(33.375 / 2.0), // <--CHANGE TO YOUR DRIVE BASE RADIUS (distance from center of robot to furthest module in meters)
        new ReplanningConfig()
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Returns the current robot-relative ChassisSpeeds.
   *
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                DriveConstants.kDrivePeriod));
    
    // Send swerve module states the dashboard
    sendModuleStatesToDashboard(swerveModuleStates);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);

    m_frontLeft.setDesiredState(targetStates[0]);
    m_frontRight.setDesiredState(targetStates[1]);
    m_rearLeft.setDesiredState(targetStates[2]);
    m_rearRight.setDesiredState(targetStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();

    return states;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_pigeon.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_pigeon.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_pigeon.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // Convenience methods to get the SysId commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return sysId.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return sysId.dynamic(direction);
  }
  
  public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
      return sysId.quasistatic(direction);
  }
  
  public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
      return sysId.dynamic(direction);
  }

  /**
   * Takes in a swerve module position and reutrns the module configuration for that specific swerve drive motor.
   * <p> The following integers correspond to the position of the swerve module: 0 - FR, 1 - FL, 2 - BL, 3 - BR
   * @param position An integer representing the position of the swerve module drive motor.
   * 
   * @return a TalonFXConfiguration for the drive motor of the swerve module.
   */
  private TalonFXConfiguration getSwerveDriveMotorConfiguration(int position) {
    // Instantiate a new TalonFXConfiguration
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Check the module position
    switch (position) {
      case 0: // FR
        // Obtain the desired configuration from the Constants.SwerveModule0Configuration
        config.HardwareLimitSwitch.ForwardLimitEnable = SwerveModule0DriveConfiguration.Hardware_ForwardLimitEnable;
        config.HardwareLimitSwitch.ReverseLimitEnable = SwerveModule0DriveConfiguration.Hardware_ReverseLimitEnable;
        config.MotorOutput.Inverted = SwerveModule0DriveConfiguration.MotorOutput_Inverted;
        config.MotorOutput.NeutralMode = SwerveModule0DriveConfiguration.MotorOutput_NeutralMode;
        break;
      case 1: // FL
        // Obtain the desired configuration from the Constants.SwerveModule1Configuration
        config.HardwareLimitSwitch.ForwardLimitEnable = SwerveModule1DriveConfiguration.Hardware_ForwardLimitEnable;
        config.HardwareLimitSwitch.ReverseLimitEnable = SwerveModule1DriveConfiguration.Hardware_ReverseLimitEnable;
        config.MotorOutput.Inverted = SwerveModule1DriveConfiguration.MotorOutput_Inverted;
        config.MotorOutput.NeutralMode = SwerveModule1DriveConfiguration.MotorOutput_NeutralMode;
        break;
      case 2: // BL
        // Obtain the desired configuration from the Constants.SwerveModule2Configuration
        config.HardwareLimitSwitch.ForwardLimitEnable = SwerveModule2DriveConfiguration.Hardware_ForwardLimitEnable;
        config.HardwareLimitSwitch.ReverseLimitEnable = SwerveModule2DriveConfiguration.Hardware_ReverseLimitEnable;
        config.MotorOutput.Inverted = SwerveModule2DriveConfiguration.MotorOutput_Inverted;
        config.MotorOutput.NeutralMode = SwerveModule2DriveConfiguration.MotorOutput_NeutralMode;
        break;

      case 3: //BR
        // Obtain the desired configuration from the Constants.SwerveModule3Configuration
        config.HardwareLimitSwitch.ForwardLimitEnable = SwerveModule3DriveConfiguration.Hardware_ForwardLimitEnable;
        config.HardwareLimitSwitch.ReverseLimitEnable = SwerveModule3DriveConfiguration.Hardware_ReverseLimitEnable;
        config.MotorOutput.Inverted = SwerveModule3DriveConfiguration.MotorOutput_Inverted;
        config.MotorOutput.NeutralMode = SwerveModule3DriveConfiguration.MotorOutput_NeutralMode;
        break;
      default:
        break;
    }

    // Return the TalonFXConfiguration
    return config;
  }

  /**
   * Takes in a swerve module position and reutrns the module configuration for that specific swerve azimuth motor.
   * <p> The following integers correspond to the position of the swerve module: 0 - FR, 1 - FL, 2 - BL, 3 - BR
   * @param position An integer representing the position of the swerve module drive motor.
   * 
   * @return a TalonFXConfiguration for the azimuth motor of the swerve module.
   */
  private TalonFXConfiguration getSwerveAzimuthMotorConfiguration(int position) {
    // Instantiate a new TalonFXConfiguration
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Check the module position
    switch (position) {
      case 0: // FR
        // Obtain the desired configuration from the Constants.SwerveModule0Configuration
        config.HardwareLimitSwitch.ForwardLimitEnable = SwerveModule0AzimuthConfiguration.Hardware_ForwardLimitEnable;
        config.HardwareLimitSwitch.ReverseLimitEnable = SwerveModule0AzimuthConfiguration.Hardware_ReverseLimitEnable;
        config.MotorOutput.Inverted = SwerveModule0AzimuthConfiguration.MotorOutput_Inverted;
        config.MotorOutput.NeutralMode = SwerveModule0AzimuthConfiguration.MotorOutput_NeutralMode;
        break;
      case 1: // FL
        // Obtain the desired configuration from the Constants.SwerveModule1Configuration
        config.HardwareLimitSwitch.ForwardLimitEnable = SwerveModule1AzimuthConfiguration.Hardware_ForwardLimitEnable;
        config.HardwareLimitSwitch.ReverseLimitEnable = SwerveModule1AzimuthConfiguration.Hardware_ReverseLimitEnable;
        config.MotorOutput.Inverted = SwerveModule1AzimuthConfiguration.MotorOutput_Inverted;
        config.MotorOutput.NeutralMode = SwerveModule1AzimuthConfiguration.MotorOutput_NeutralMode;
        break;
      case 2: // BL
        // Obtain the desired configuration from the Constants.SwerveModule2Configuration
        config.HardwareLimitSwitch.ForwardLimitEnable = SwerveModule2AzimuthConfiguration.Hardware_ForwardLimitEnable;
        config.HardwareLimitSwitch.ReverseLimitEnable = SwerveModule2AzimuthConfiguration.Hardware_ReverseLimitEnable;
        config.MotorOutput.Inverted = SwerveModule2AzimuthConfiguration.MotorOutput_Inverted;
        config.MotorOutput.NeutralMode = SwerveModule2AzimuthConfiguration.MotorOutput_NeutralMode;
        break;

      case 3: //BR
        // Obtain the desired configuration from the Constants.SwerveModule3Configuration
        config.HardwareLimitSwitch.ForwardLimitEnable = SwerveModule3AzimuthConfiguration.Hardware_ForwardLimitEnable;
        config.HardwareLimitSwitch.ReverseLimitEnable = SwerveModule3AzimuthConfiguration.Hardware_ReverseLimitEnable;
        config.MotorOutput.Inverted = SwerveModule3AzimuthConfiguration.MotorOutput_Inverted;
        config.MotorOutput.NeutralMode = SwerveModule3AzimuthConfiguration.MotorOutput_NeutralMode;
        break;
      default:
        break;
    }

    // Return the TalonFXConfiguration
    return config;
  }

  private void sendModuleStatesToDashboard (SwerveModuleState[] allStates) {
    // Build desiredState so advantageScope can see the values using the desired module states
    double desiredState[] = {
      allStates[0].angle.getDegrees(),   // FL
      allStates[0].speedMetersPerSecond, // FL
      allStates[1].angle.getDegrees(),   // FR
      allStates[1].speedMetersPerSecond, // FR
      allStates[2].angle.getDegrees(),   // RL
      allStates[2].speedMetersPerSecond, // RL
      allStates[3].angle.getDegrees(),   // RR
      allStates[3].speedMetersPerSecond, // RR
    };

    // Build actualState so advantageScope can see the values using the desired module states
    double actualState[] = {
      m_frontLeft.getState().angle.getDegrees(),
      m_frontLeft.getState().speedMetersPerSecond,
      m_frontRight.getState().angle.getDegrees(),
      m_frontRight.getState().speedMetersPerSecond,
      m_rearLeft.getState().angle.getDegrees(),
      m_rearLeft.getState().speedMetersPerSecond,
      m_rearRight.getState().angle.getDegrees(),
      m_rearRight.getState().speedMetersPerSecond,
    };

    // Send desiredState and actualState to the SmartDashboard to be seen in advantageScope
    SmartDashboard.putNumberArray("SwerveModuleDesiredStates", desiredState);
    SmartDashboard.putNumberArray("SwerveModuleActualStates", actualState);
  }
}
