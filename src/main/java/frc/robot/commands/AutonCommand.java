package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.drive.DriveSubsystemInterface;

public class AutonCommand extends SequentialCommandGroup {
    private final DriveSubsystemInterface driveSubsystemInterface;

    public AutonCommand(DriveSubsystemInterface driveSubsystemInterface) {
        this.driveSubsystemInterface = driveSubsystemInterface;
        addRequirements(driveSubsystemInterface);

        addCommands(
            // Reset encoders
            new InstantCommand(driveSubsystemInterface::resetEncoders),
            new RunCommand(() -> {
                // Commmands go here
            }));
    }

}
