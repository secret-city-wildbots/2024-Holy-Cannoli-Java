package frc.robot.characterization;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.*;

public class SwerveModuleSysId extends SubsystemBase {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    
    // Constants for turn motor encoder conversion
    private static final double GEAR_RATIO = 12.8; // Example gear ratio
    private static final double COUNTS_PER_ROTATION = 2048.0;
    private static final double DEGREES_PER_COUNT = 360.0 / COUNTS_PER_ROTATION / GEAR_RATIO;
    
    // Mutable measures for logging data
    private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.mutable(MetersPerSecond.of(0));
    private final MutableMeasure<Angle> angularDistance = MutableMeasure.mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> angularVelocity = MutableMeasure.mutable(RotationsPerSecond.of(0));
    
    // Create SysId routines for drive and turn
    private final SysIdRoutine driveSysId;
    private final SysIdRoutine turnSysId;
    
    public SwerveModuleSysId(int driveMotorId, int turnMotorId) {
        driveMotor = new TalonFX(driveMotorId);
        turnMotor = new TalonFX(turnMotorId);
        
        // Configure motors
        configureMotors();
        
        // Configure the drive motor SysId routine
        driveSysId = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Drive mechanism - voltage setter
                (Measure<Voltage> volts) -> {
                    driveMotor.setVoltage(volts.in(Volts)); 
                    // Lock turn motor at 0 degrees during drive characterization
                    turnMotor.setVoltage(calculateTurnPID(0.0));
                },
                // Log drive data
                log -> {
                    log.motor("drive_" + driveMotorId)
                        .voltage(appliedVoltage.mut_replace(driveMotor.get() * 12.0, Volts))
                        .linearPosition(distance.mut_replace(getDrivePosition(), Meters))
                        .linearVelocity(velocity.mut_replace(getDriveVelocity(), MetersPerSecond));
                },
                // Drive subsystem
                this));
                
        // Configure the turn motor SysId routine
        turnSysId = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Turn mechanism - voltage setter
                (Measure<Voltage> volts) -> {
                    turnMotor.setVoltage(volts.in(Volts));
                    driveMotor.setVoltage(0.0);
                },
                // Log turn data
                log -> {
                    log.motor("turn_" + turnMotorId)
                        .voltage(appliedVoltage.mut_replace(turnMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .angularPosition(angularDistance.mut_replace(getTurnPosition(), Rotations))
                        .angularVelocity(angularVelocity.mut_replace(getTurnVelocity(), RotationsPerSecond));
                },
                // Turn subsystem
                this));
    }
    
    private void configureMotors() {
        // Create configurations
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        
        // Configure drive motor
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfig);
        
        // Configure turn motor
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotor.getConfigurator().apply(turnConfig);
        
        // Reset encoders to zero position
        driveMotor.setPosition(0.0);
        turnMotor.setPosition(0.0);
    }
    
    private double calculateTurnPID(double targetAngle) {
        double currentAngle = getTurnPosition();
        double error = targetAngle - currentAngle;
        
        // Normalize error to -180 to 180 degrees
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        // Simple P controller - adjust gains as needed
        return error * 0.1;
    }
    
    private double getDriveVelocity() {
        double velocity = (driveMotor.getRotorPosition().getValueAsDouble() / 6.12) * (2 * Math.PI * 0.0508);
        return velocity;
    }
    
    private double getDrivePosition() {
        // Convert from sensor counts to meters
        double position = driveMotor.getRotorPosition().getValueAsDouble() / 2048 * (2 * Math.PI * 0.0508) / 6.12;
        return position;
    }
    
    private double getTurnVelocity() {
        return (turnMotor.getVelocity().getValue() * 10.0 / 2048.0) / 21.4285;
    }
    
    private double getTurnPosition() {
        return (turnMotor.getPosition().getValue() / 2048.0) / 21.4285;
    }
    
    // Convenience methods to get the SysId commands
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return driveSysId.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return driveSysId.dynamic(direction);
    }
    
    public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return turnSysId.quasistatic(direction);
    }
    
    public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
        return turnSysId.dynamic(direction);
    }
}