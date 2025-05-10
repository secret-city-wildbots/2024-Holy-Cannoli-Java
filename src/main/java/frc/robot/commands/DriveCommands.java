package frc.robot.commands;

// Java Util Libraries
import java.util.function.DoubleSupplier;

// WPI Math Libraries
import edu.wpi.first.math.MathUtil;

// WPI Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 4265 Constants
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

// 4265 Subsystems
import frc.robot.subsystems.drive.SwerveDrive;

public class DriveCommands {

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      SwerveDrive driveSystem,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rotSpeed) {
    return Commands.run(
        () -> {
            driveSystem.drive(
                // Multiply by max speed to map the joystick unitless inputs to actual units.
                // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                // converting them to actual units.
                ControllerConstants.kDriveYSpeedLimiter.calculate(
                    MathUtil.applyDeadband(xSpeed.getAsDouble(), 0.08)) * DriveConstants.kMaxSpeedMetersPerSecond,
                ControllerConstants.kDriveXSpeedLimiter.calculate(
                    MathUtil.applyDeadband(ySpeed.getAsDouble(), 0.08)) * DriveConstants.kMaxSpeedMetersPerSecond,
                ControllerConstants.kRotateLimiter.calculate(
                    MathUtil.applyDeadband(rotSpeed.getAsDouble(), 0.08)) * SwerveModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                false
            );
        },
        driveSystem);
  }
}