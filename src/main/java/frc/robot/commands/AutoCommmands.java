package frc.robot.commands;


import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
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
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new WaitUntilConditionCommand(shooter::bothAtSetpointRPM),
                    transfer.shootCommand()
                ),
                shooter.shooterAutoLineRevUpCommand()
            )
            //shooter.stopShooterCommand()
        );
    }
    
    public Command autoIntakeToShooter() {
        return new SequentialCommandGroup(
            combo.startShooterIntakeCommand()
            .until(transfer::shooterFull)
            .andThen(combo.stowCommand())
            .withTimeout(.5)
        );
    }
    public Command twoPieceTopAuto() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("TwoPieceTop");
        robotDrive.resetEstimatedPose(path.getPreviewStartingHolonomicPose());
        //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        return new SequentialCommandGroup(
            shootOnStart(),
            new ParallelCommandGroup(
                //shooter.shooterAutoLineRevUpCommand(),
                robotDrive.followAutonPath(path),
                new SequentialCommandGroup(
                    new SequentialCommandGroup(
                        new WaitUntilConditionCommand(()->robotDrive.atPose(AutoConstants.blueTopNotePose, 1.5, 90)),
                        autoIntakeToShooter()
                    ),
                    new SequentialCommandGroup(
                        new WaitUntilConditionCommand(()->robotDrive.atPose(AutoConstants.centerShootPose, 0.5, 10)),
                        transfer.shootCommand()
                    )
                )
                // new StepCommand(transfer.shootCommand(), ()->robotDrive.atPose(AutoConstants.centerShootPose, 0.3, 1, transfer)

                //new StepCommand(intake.flipToGroundAndRunPayloadCommand(intake.intakeToMiddle(), 0, 0), ()->true/*()->robotDrive.stepCommandBooleanSupplier(2.01, 6.85, -160)*/, intake)

            ));
            
            //new ParallelCommandGroup(
                
                
            //)
        //);
    }


    /**
     * 
     * @return
     */
//     public SequentialCommandGroup threePlusOneTopAuto() {
//         return new SequentialCommandGroup(
//         //start shooter--need more time to rev up.    
//         shootOnStart(),
//         new ParallelCommandGroup(
//             //start following path. The auto will end when it is completed
//             robotDrive.followAutonPath(PathPlannerPath.fromPathFile("CloseTopFixed")),
//             //sequence of stepCommands to run along with path. Add position stuff later.
//             new SequentialCommandGroup(
//                 new ParallelDeadlineGroup(
//                     new StepCommand(transfer.intakeToShooterCommand(), ()->robotDrive.atPose(new Pose2d(1.50, 5.86, new Rotation2d(Math.toRadians(180)))), transfer),
//                     new StepCommand(intake.intakeToMiddle(), ()->robotDrive.atPose(new Pose2d(1.86, 7.03, new Rotation2d(Math.toRadians(-175)))), intake)),
//                 new ParallelCommandGroup(
//                     new InstantCommand(()->transfer.stopShooterTransfer()),
//                     new InstantCommand(()->transfer.stopTransfer())),
//                 new ParallelDeadlineGroup(
//                     new StepCommand(transfer.intakeToShooterCommand(), ()->robotDrive.atPose(new Pose2d(1.50, 5.68, new Rotation2d(Math.toRadians(180)))), transfer),
//                     new StepCommand(intake.intakeToMiddle(), ()->robotDrive.atPose(new Pose2d(2.09, 5.42, new Rotation2d(Math.toRadians(180)))), intake)),
//                 new ParallelCommandGroup(
//                     new InstantCommand(()->transfer.stopShooterTransfer()),
//                     new InstantCommand(()->transfer.stopTransfer())),
//                 new ParallelDeadlineGroup(
//                     new StepCommand(transfer.intakeToShooterCommand(), ()->robotDrive.atPose(new Pose2d(1.50, 5.48, new Rotation2d(Math.toRadians(180)))), transfer),
//                     new StepCommand(intake.intakeToMiddle(), ()->robotDrive.atPose(new Pose2d(1.50, 5.48, new Rotation2d(Math.toRadians(180)))), intake)),
//                 new ParallelCommandGroup(
//                     new InstantCommand(()->transfer.stopShooterTransfer()),
//                     new InstantCommand(()->transfer.stopTransfer()))
//             )
//         ));
//     }

}
