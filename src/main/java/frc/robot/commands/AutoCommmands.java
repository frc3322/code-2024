package frc.robot.commands;


import java.lang.reflect.Field;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
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
            .andThen(
                combo.stowCommand()
                .until(intake::atSetpoint)
            )
            
        );
    }

    public Command intakeTopNote() {
        Pose2d notePose = robotDrive.isAllianceRed() ? FieldConstants.redTopNotePose : FieldConstants.blueTopNotePose;

        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(notePose, 1.5, 0)),
            autoIntakeToShooter()
        );

    }

    public Command intakeMiddleNote() {
        Pose2d notePose = robotDrive.isAllianceRed() ? FieldConstants.redMiddleNotePose : FieldConstants.blueMiddleNotePose;

        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(notePose, 1.5, 0)),
            autoIntakeToShooter()
        );

    }

    public Command intakeBottomNote() {
        Pose2d notePose = robotDrive.isAllianceRed() ? FieldConstants.redBottomNotePose : FieldConstants.blueBottomNotePose;

        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(notePose, 2, 0)),
            autoIntakeToShooter()
        );

    }

    public Command shoot(Pose2d shootPose) {
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(shootPose, 0.5, 10)),
            transfer.shootCommand()
        );
    }

    public Command twoPieceTopAuto() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.twoPieceTopString);
        Pose2d shootPose = path.getPreviewStartingHolonomicPose();
        robotDrive.resetEstimatedPose(shootPose);

        //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        return new SequentialCommandGroup(
            shootOnStart(),
            new ParallelCommandGroup(
                robotDrive.followAutonPath(path),
                new SequentialCommandGroup(
                    new SequentialCommandGroup(
                        new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.blueTopNotePose, 1.5, 90)),
                        autoIntakeToShooter()
                    ),
                    new SequentialCommandGroup(
                        new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.topShootPose, 0.5, 10)),
                        transfer.shootCommand()
                    )
                )
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
