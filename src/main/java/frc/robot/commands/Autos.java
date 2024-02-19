package frc.robot.commands;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class Autos {
    
    DriveSubsystem robotDrive; //cannot be final, otherwise it gives an error that it may not have been initialized. private is fine.
    Elevator elevator;
    Intake intake;
    Shooter shooter;
    Transfer transfer;

    

    public Autos(DriveSubsystem robotDrive, Intake intake, Elevator elevator, Transfer transfer, Shooter shooter) {
        this.robotDrive = robotDrive;
        this.elevator = elevator;
        this.intake = intake;
        this.transfer = transfer;
        this.shooter = shooter;
        
    }

    public SequentialCommandGroup TwoPieceTopAuto() {
        return new SequentialCommandGroup(
            shooter.shooterAutoLineRevUpCommand(),
            intake.flipToGroundCommand(),
            transfer.intakeToShooterCommand(),
            new ParallelCommandGroup(
                robotDrive.followAutonPath(PathPlannerPath.fromPathFile("TwoPieceTop")),
                new ParallelDeadlineGroup(
                    new StepCommand(transfer.intakeToShooterCommand(), ()->robotDrive.stepCommandBooleanSupplier(1.29, 5.48, 180), transfer),
                    new StepCommand(intake.intakeToMiddle(), ()->robotDrive.stepCommandBooleanSupplier(2.01, 6.85, -160), intake)
                )
            )
        );
    }


    /**
     * 
     * @return
     */
    public SequentialCommandGroup threePlusOneTopAuto() {
        return new SequentialCommandGroup(
        //start shooter--need more time to rev up.    
        shooter.shooterAutoLineRevUpCommand(),
            //new ParallelCommandGroup(
                //shoot from starting position--may not be viable strategy
                intake.flipToGroundCommand(),
                transfer.intakeToShooterCommand()/* )*/,    
     
        new ParallelCommandGroup(
            //start following path. The auto will end when it is completed
            robotDrive.followAutonPath(PathPlannerPath.fromPathFile("CloseTopFixed")),
            //sequence of stepCommands to run along with path. Add position stuff later.
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new StepCommand(transfer.intakeToShooterCommand(), ()->robotDrive.stepCommandBooleanSupplier(1.50, 5.86, Math.toRadians(180)), transfer),
                    new StepCommand(intake.intakeToMiddle(), ()->robotDrive.stepCommandBooleanSupplier(1.86, 7.03, Math.toRadians(-175)), intake)),
                new ParallelCommandGroup(
                    new InstantCommand(()->transfer.stopShooterTransfer()),
                    new InstantCommand(()->transfer.stopTransfer())),
                new ParallelDeadlineGroup(
                    new StepCommand(transfer.intakeToShooterCommand(), ()->robotDrive.stepCommandBooleanSupplier(1.50, 5.68, Math.toRadians(180)), transfer),
                    new StepCommand(intake.intakeToMiddle(), ()->robotDrive.stepCommandBooleanSupplier(2.09, 5.42, Math.toRadians(180)), intake)),
                new ParallelCommandGroup(
                    new InstantCommand(()->transfer.stopShooterTransfer()),
                    new InstantCommand(()->transfer.stopTransfer())),
                new ParallelDeadlineGroup(
                    new StepCommand(transfer.intakeToShooterCommand(), ()->robotDrive.stepCommandBooleanSupplier(1.50, 5.48, Math.toRadians(180)), transfer),
                    new StepCommand(intake.intakeToMiddle(), ()->robotDrive.stepCommandBooleanSupplier(1.50, 5.48, Math.toRadians(180)), intake)),
                new ParallelCommandGroup(
                    new InstantCommand(()->transfer.stopShooterTransfer()),
                    new InstantCommand(()->transfer.stopTransfer()))
            )
        ));
    }
    


    
}
