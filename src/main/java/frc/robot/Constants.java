// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPI TimedRobot
import edu.wpi.first.wpilibj.TimedRobot;

// WPI Math Libraries
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// Phoenix6 Signal Libraries
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ControllerConstants {
    // Slew rate limiters to make joystick inputs more gentle
    public static final SlewRateLimiter kDriveXSpeedLimiter = new SlewRateLimiter(6);
    public static final SlewRateLimiter kDriveYSpeedLimiter = new SlewRateLimiter(6);
    public static final SlewRateLimiter kRotateLimiter = new SlewRateLimiter(3);
  }

  public static final class DriveConstants {
    // CAN IDs for the Swerve drive motors
    public static final int kFrontRightDriveMotorPort = 10;
    public static final int kFrontLeftDriveMotorPort = 11;
    public static final int kRearLeftDriveMotorPort = 12;
    public static final int kRearRightDriveMotorPort = 13;

    // CAN IDs for the Swerve drive motors
    public static final int kFrontRightAzimuthMotorPort = 20;
    public static final int kFrontLeftAzimuthMotorPort = 21;
    public static final int kRearLeftAzimuthMotorPort = 22;
    public static final int kRearRightAzimuthMotorPort = 23;

    // CAN ID for the pigeon (gyro)
    public static final int kPigeonPort = 6;

    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(23.5);

    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.0 + 5.0 / 8.0);

    // Swerve drive kinematics
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    // Gyro reversed boolean
    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    // TODO: run SysId to get these values
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    // Max speed m / s
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(16.5);
  }

  public static final class SwerveModuleConstants {
    /*
     * Gear Ratios for the swerve drive and azimuth COTS can be found at the following link:
     * https://www.swervedrivespecialties.com/products/mk4i-swerve-module
     */
    public static final double driveGearRatio = 1 / 6.12;
    public static final double azimuthGearRatio = 1 / (150.0 / 7.0);

    // Wheel diameter in meters
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // We convert from inches to meters

    // Robot max angular speed and acceleration
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    // kP values for Drive and Azimuth Motor PID Controllers
    // TODO: need to run SysId to get these values (kP kI and kD)
    public static final double kPModuleAzimuthController = 0;
    public static final double kPModuleDriveController = 0;
  }

  public static final class SwerveModule0DriveConfiguration {
    /*
     * Swerve Module Drive Motor Configs (Front Right)
     * 
     * Note: We need to set the motor output to be CW (clockwise), becuase we have the bevel
     * gears facing inwards.
     */
    public static final Boolean Hardware_ForwardLimitEnable = false;
    public static final Boolean Hardware_ReverseLimitEnable = false;
    public static final InvertedValue MotorOutput_Inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue MotorOutput_NeutralMode = NeutralModeValue.Coast;
  
  }

  public static final class SwerveModule1DriveConfiguration {
    /*
     * Swerve Module Drive Motor Configs (Front Left)
     * 
     * Note: We need to set the motor output to be CCW (counter clockwise), because we have the bevel gears facing
     * inwards.
     */
    public static final Boolean Hardware_ForwardLimitEnable = false;
    public static final Boolean Hardware_ReverseLimitEnable = false;
    public static final InvertedValue MotorOutput_Inverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue MotorOutput_NeutralMode = NeutralModeValue.Coast;
  
  }
 
  public static final class SwerveModule2DriveConfiguration {
    /*
     * Swerve Module Drive Motor Configs (Back Left)
     * 
     * Note: We need to set the motor output to be CCW (counter clockwise), because we have the bevel gears facing
     * inwards.
     */
    public static final Boolean Hardware_ForwardLimitEnable = false;
    public static final Boolean Hardware_ReverseLimitEnable = false;
    public static final InvertedValue MotorOutput_Inverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue MotorOutput_NeutralMode = NeutralModeValue.Coast;
  
  }

  public static final class SwerveModule3DriveConfiguration {
    /*
     * Swerve Module Drive Motor Configs (Back Right)
     * 
     * Note: We need to set the motor output to be CW (clockwise), becuase we have the bevel
     * gears facing inwards.
     */
    public static final Boolean Hardware_ForwardLimitEnable = false;
    public static final Boolean Hardware_ReverseLimitEnable = false;
    public static final InvertedValue MotorOutput_Inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue MotorOutput_NeutralMode = NeutralModeValue.Coast;
  
  }

  public static final class SwerveModule0AzimuthConfiguration {
    /*
     * Swerve Module Drive Motor Configs (Front Right)
     * 
     * Note: We need to set the motor output to be CW (clockwise), becuase we have the bevel
     * gears facing inwards.
     */
    public static final Boolean Hardware_ForwardLimitEnable = false;
    public static final Boolean Hardware_ReverseLimitEnable = false;
    public static final InvertedValue MotorOutput_Inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue MotorOutput_NeutralMode = NeutralModeValue.Coast;
  
  }

  public static final class SwerveModule1AzimuthConfiguration {
    /*
     * Swerve Module Drive Motor Configs (Front Left)
     * 
     * Note: We need to set the motor output to be CW (clockwise), because we have the bevel gears facing
     * inwards.
     */
    public static final Boolean Hardware_ForwardLimitEnable = false;
    public static final Boolean Hardware_ReverseLimitEnable = false;
    public static final InvertedValue MotorOutput_Inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue MotorOutput_NeutralMode = NeutralModeValue.Coast;
  
  }
 
  public static final class SwerveModule2AzimuthConfiguration {
    /*
     * Swerve Module Drive Motor Configs (Back Left)
     * 
     * Note: We need to set the motor output to be CW (clockwise), because we have the bevel gears facing
     * inwards.
     */
    public static final Boolean Hardware_ForwardLimitEnable = false;
    public static final Boolean Hardware_ReverseLimitEnable = false;
    public static final InvertedValue MotorOutput_Inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue MotorOutput_NeutralMode = NeutralModeValue.Coast;
  
  }

  public static final class SwerveModule3AzimuthConfiguration {
    /*
     * Swerve Module Drive Motor Configs (Back Right)
     * 
     * Note: We need to set the motor output to be CW (clockwise), becuase we have the bevel
     * gears facing inwards.
     */
    public static final Boolean Hardware_ForwardLimitEnable = false;
    public static final Boolean Hardware_ReverseLimitEnable = false;
    public static final InvertedValue MotorOutput_Inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue MotorOutput_NeutralMode = NeutralModeValue.Coast;
  
  }

  public static final class OIConstants {
    // Driver Controller port number
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    // Auto configuration constants for the robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // PID Values
    // TODO: need to run SysId and test to dial in these values
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
