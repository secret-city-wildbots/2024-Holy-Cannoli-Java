// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Path Planner Libraries
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

// WPI Xbox Controller Library
import edu.wpi.first.wpilibj.XboxController;

// WPI Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// 4265 Constants
import frc.robot.Constants.OIConstants;

// 4265 Commands
import frc.robot.commands.*;

// 4265 Subsystems
import frc.robot.subsystems.drive.SwerveDrive;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveDrive m_robotDrive = new SwerveDrive();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the default commands
    configureDefaultCommands();

    // register named commands
    registerNamedCommands();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your default commands for the subsystems.
   */
  private void configureDefaultCommands() {
    // DriveSubsystem Default Command
    m_robotDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_robotDrive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()
        )
    );
  }

  /**
   * Use this method to register named commands for path planner.
   */
  private void registerNamedCommands() {
    // Register HelloWorld
     NamedCommands.registerCommand("HelloWorld", HelloWorldCommand.helloWorldCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // While the A button is pressed, schedule the command
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    // While the B button is pressed, schedule the command
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
    .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // While the Y button is pressed, schedule the command
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
  
    // While the X button is pressed, schedule the command
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Path planner autonomous command
    return new PathPlannerAuto("first_auto");
  }
}
