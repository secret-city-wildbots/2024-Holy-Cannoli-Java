package frc.robot.commands;

// WPI Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class HelloWorldCommand {
    
    private HelloWorldCommand(){}
    
    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command helloWorldCommand() {
        // Create a new functional command to test commands inbetween multiple auto paths
        return new FunctionalCommand(
            // Initialize: This code runs once when the command starts.
            () -> {
                System.out.println("Command started");
                // Initialize any required state or resources here
            },
            // Execute: This code runs repeatedly while the command is active.
            () -> {
                System.out.println("Executing command...");
                // Perform the main task of the command here
            },
            // End: This code runs once when the command ends.
            interrupted -> {
                if (interrupted) {
                    System.out.println("Command was interrupted");
                } else {
                    System.out.println("Command finished successfully");
                }
                // Clean up resources or perform any final actions here
            },
            // Is Finished: Returns true when the command should end.
            () -> {
                return true; 
            }  // Command will run indefinitely unless interrupted
        );
    }

}
