package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class AutoCommmands {
    
    private final DriveSubsystem robotDrive;
    private final Elevator elevator;
    private final Intake intake;
    private final Shooter shooter;
    private final Transfer transfer;
    private final ComboCommands combo;

    public AutoCommmands(DriveSubsystem robotDrive, Intake intake, Elevator elevator, Transfer transfer, Shooter shooter, ComboCommands combo) {
        this.robotDrive = robotDrive;
        this.elevator = elevator;
        this.intake = intake;
        this.transfer = transfer;
        this.shooter = shooter;
        this.combo = combo;
    }

    public Command shootOnStart() {
        return new SequentialCommandGroup(
            shooter.shooterAutoLineRevUpCommand(),
            new StepCommand(transfer.shootCommand(), shooter::bothAtSetpointRPM, transfer)
        );
    }
    
    // public Command autoIntake() {
    //     return new SequentialCommandGroup(
    //         combo.startShooterIntakeCommand()
    //         .until(transfer::shooterFull)
    //         .andThen(combo.stowCommandGroup())
    //     );
    // }
}
