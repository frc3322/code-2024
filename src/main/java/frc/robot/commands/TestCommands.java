package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class TestCommands{
    
    private final DriveSubsystem robotDrive;
    private final Elevator elevator;
    private final Intake intake;
    private final Shooter shooter;
    private final Transfer transfer;

    public TestCommands(DriveSubsystem robotDrive, Elevator elevator, Intake intake, Transfer transfer, Shooter shooter){
        this.robotDrive = robotDrive;
        this.intake = intake;
        this.elevator = elevator;
        this.transfer = transfer;
        this.shooter = shooter;
      }

    public Command driveTestCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    robotDrive.drive(.25, 0, 0, false, true);
                },
                robotDrive
            ),
            new WaitCommand(3),
            new InstantCommand(
                () -> {
                    robotDrive.drive(0, .25, 0, false, true);
                },
                robotDrive
            ),
            new WaitCommand(3),
            new InstantCommand(
                () -> {
                    robotDrive.drive(0, 0, .25, false, true);
                },
                robotDrive
            ),
            new WaitCommand(3),
            new InstantCommand(
                () -> {
                    robotDrive.drive(.25, .25, .25, false, true);
                },
                robotDrive
            ),
            new WaitCommand(3),
            new InstantCommand(
                () -> {
                    robotDrive.drive(0, 0, 0, false, true);
                },
                robotDrive
            )
        );
    }

    public Command elevatorTestCommand() {
        return new SequentialCommandGroup(
            elevator.goToBottomCommand(),
            new WaitCommand(2),
            elevator.goToAmpCommand(),
            new WaitCommand(2),
            elevator.goToTopCommand(),
            new WaitCommand(2),
            elevator.goToBottomCommand(),
            new WaitCommand(2),
            elevator.goToTopCommand(),
            new WaitCommand(2),
            elevator.goToBottomCommand()
        );
    }

    // public Command intakeTestCommand() {
    //     return new SequentialCommandGroup(
            
    //     )
    // }

    public Command shooterTestCommand() {
        return new SequentialCommandGroup(
            shooter.shooterRevUpCommand(1000),
            new WaitCommand(3),
            shooter.shooterAutoLineRevUpCommand(),
            new WaitCommand(3),
            shooter.stopShooterCommand()
        );
    }

    // public Command transferTestCommand() {

    // }

}