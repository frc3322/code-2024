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

    Pose2d shootPose;
    PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.fourPieceMiddleString);

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
            
        );
    }

    public Command intakeTopNote(boolean red) {
        Pose2d notePose = robotDrive.flipPoseIfRed(FieldConstants.blueTopNotePose, red);

        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(notePose, 1.5, 0)),
            autoIntakeToShooter()
        );

    }

    public Command intakeMiddleNote(boolean red) {
        Pose2d notePose = robotDrive.flipPoseIfRed(FieldConstants.blueMiddleNotePose, red);

        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(notePose, 1.5, 0)),
            autoIntakeToShooter()
        );

    }

    public Command intakeBottomNote(boolean red) {
       Pose2d notePose = robotDrive.flipPoseIfRed(FieldConstants.blueBottomNotePose, red);

        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(notePose, 2, 0)),
            autoIntakeToShooter()
        );

    }

    public Command intakeCenterMiddleNote(){
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.centerMidPose, 1.5, 0)),
            autoIntakeToShooter()
            );
            
        }

    public Command intakeCenterTopNote(){
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.centerTopPose, 1.5, 0)),
            autoIntakeToShooter()
            );
            
        }

    public Command flipIntakeUp() {
        return combo.stowCommand().withTimeout(0.2);
    }
        
    public Command shoot(Pose2d shootPose) {
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(shootPose, 0.3, 10)),
            transfer.shootCommand()
        );
    }

    // public Command twoPieceTopAuto() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.twoPieceTopString);
    //     Pose2d shootPose = path.getPreviewStartingHolonomicPose();
    //     robotDrive.resetEstimatedPose(shootPose);

    //     //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
    //     return new SequentialCommandGroup(
    //         shootOnStart(),
    //         new ParallelCommandGroup(
    //             robotDrive.followAutonPath(path),
    //             new SequentialCommandGroup(
    //                 new SequentialCommandGroup(
    //                     new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.blueTopNotePose, 1.5, 90)),
    //                     autoIntakeToShooter()
    //                 ),
    //                 new SequentialCommandGroup(
    //                     new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.topShootPose, 0.5, 10)),
    //                     transfer.shootCommand()
    //                 )
    //             )
    //         ));
            
    //         //new ParallelCommandGroup(
                
                
    //         //)
    //     //);
    // }

    // public Command threePieceTopAuto() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.twoPieceTopString);
    //     Pose2d shootPose = path.getPreviewStartingHolonomicPose();
    //     robotDrive.resetEstimatedPose(shootPose);

    //     //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
    //     return new SequentialCommandGroup(
    //         shootOnStart(),
    //         new ParallelCommandGroup(
    //             robotDrive.followAutonPath(path),
    //             new SequentialCommandGroup(
    //                 intakeTopNote(),
    //                 shoot(shootPose),
    //                 intakeMiddleNote(),
    //                 shoot(shootPose)
    //             )
    //         ));
            
    // }

    // public Command fourPieceTopAuto() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.twoPieceTopString);
    //     Pose2d shootPose = path.getPreviewStartingHolonomicPose();
    //     robotDrive.resetEstimatedPose(shootPose);

    //     //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
    //     return new SequentialCommandGroup(
    //         shootOnStart(),
    //         new ParallelCommandGroup(
    //             robotDrive.followAutonPath(path),
    //             new SequentialCommandGroup(
    //                 intakeTopNote(),
    //                 shoot(shootPose),
    //                 intakeMiddleNote(),
    //                 shoot(shootPose),
    //                 intakeBottomNote(),
    //                 shoot(shootPose)
    //             )
    //         ));
            
    // }

    // public Command twoPieceMiddleAuto() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.twoPieceMiddleString);
    //     Pose2d shootPose = path.getPreviewStartingHolonomicPose();
    //     robotDrive.resetEstimatedPose(shootPose);

    //     //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
    //     return new SequentialCommandGroup(
    //         shootOnStart(),
    //         new ParallelCommandGroup(
    //             robotDrive.followAutonPath(path),
    //             new SequentialCommandGroup(
    //                 intakeMiddleNote(),
    //                 shoot(shootPose)
    //             )
    //         ));
            
    // }

    // public Command threePieceMiddleTopAuto() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.threePieceMiddleTopString);
    //     Pose2d shootPose = path.getPreviewStartingHolonomicPose();
    //     robotDrive.resetEstimatedPose(shootPose);

    //     //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
    //     return new SequentialCommandGroup(
    //         shootOnStart(),
    //         new ParallelCommandGroup(
    //             robotDrive.followAutonPath(path),
    //             new SequentialCommandGroup(
    //                 intakeTopNote(),
    //                 shoot(shootPose),
    //                 intakeCenterTopNote(),
    //                 shoot(shootPose)
    //             )
    //         ));
            
    // }

    // public Command topThreeCenterMiddleAuto() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.topThreeCenterMiddleString);
    //     Pose2d shootPose = path.getPreviewStartingHolonomicPose();
    //     robotDrive.resetEstimatedPose(shootPose);

    //     //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
    //     return new SequentialCommandGroup(
    //         shootOnStart(),
    //         new ParallelCommandGroup(
    //             robotDrive.followAutonPath(path),
    //             new SequentialCommandGroup(
    //                 intakeMiddleNote(),
    //                 shoot(shootPose),
    //                 intakeCenterMiddleNote(),
    //                 shoot(shootPose)
    //             )
    //         ));
            
    // }

    // public Command threePieceMiddleBottomAuto() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.threePieceMiddleBottomString);
    //     Pose2d shootPose = path.getPreviewStartingHolonomicPose();
    //     robotDrive.resetEstimatedPose(shootPose);

    //     //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
    //     return new SequentialCommandGroup(
    //         shootOnStart(),
    //         new ParallelCommandGroup(
    //             robotDrive.followAutonPath(path),
    //             new SequentialCommandGroup(
    //                 intakeMiddleNote(),
    //                 shoot(shootPose),
    //                 flipIntakeUp(),
    //                 intakeBottomNote(),
    //                 flipIntakeUp(),
    //                 shoot(shootPose)
    //             )
    //         ));
            
    // }

    // public Command middleThreeCenterMiddleAuto() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.middleThreeCenterMiddleString);
    //     Pose2d shootPose = path.getPreviewStartingHolonomicPose();
    //     robotDrive.resetEstimatedPose(shootPose);

    //     //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
    //     return new SequentialCommandGroup(
    //         shootOnStart(),
    //         new ParallelCommandGroup(
    //             robotDrive.followAutonPath(path),
    //             new SequentialCommandGroup(
    //                 intakeMiddleNote(),
    //                 shoot(shootPose),
    //                 intakeCenterMiddleNote(),
    //                 shoot(shootPose)
    //             )
    //         ));
            
    // }

    public Command fourPieceMiddleAutoBlue() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.fourPieceMiddleString);
        Pose2d shootPose = robotDrive.flipPoseIfRed(path.getPreviewStartingHolonomicPose(), false);
        robotDrive.resetEstimatedPose(shootPose);

        //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        return new SequentialCommandGroup(
            shootOnStart(),
            new ParallelCommandGroup(
                robotDrive.followAutonPath(path, false),
                new SequentialCommandGroup(
                    intakeMiddleNote(false),
                    shoot(shootPose),
                    intakeTopNote(false),
                    new SequentialCommandGroup(
                        new WaitUntilConditionCommand(()->robotDrive.atPose(shootPose, 0.2, 10)),
                        transfer.shootCommand()
                    ),
                    intakeBottomNote(false),
                    shoot(shootPose)
                )
            ));
            
    }

    public Command fourPieceMiddleAutoRed() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.fourPieceMiddleString);
        Pose2d shootPose = robotDrive.flipPoseIfRed(path.getPreviewStartingHolonomicPose(), true);
        robotDrive.resetEstimatedPose(shootPose);

        //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        return new SequentialCommandGroup(
            shootOnStart(),
            new ParallelCommandGroup(
                robotDrive.followAutonPath(path, true),
                new SequentialCommandGroup(
                    intakeMiddleNote(true),
                    shoot(shootPose),
                    intakeTopNote(true),
                    new SequentialCommandGroup(
                        new WaitUntilConditionCommand(()->robotDrive.atPose(shootPose, 0.2, 10)),
                        transfer.shootCommand()
                    ),
                    intakeBottomNote(true),
                    shoot(shootPose)
                )
            ));
            
    }

//     public Command twoPieceBottomAuto() {
//         PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.twoPieceBottomString);
//         Pose2d shootPose = path.getPreviewStartingHolonomicPose();
//         robotDrive.resetEstimatedPose(shootPose);

//         //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
//         return new SequentialCommandGroup(
//             shootOnStart(),
//             new ParallelCommandGroup(
//                 robotDrive.followAutonPath(path),
//                 new SequentialCommandGroup(
//                     intakeBottomNote(),
//                     shoot(shootPose)
//                 )
//             ));
            
//     }

//     public Command threePieceBottomAuto() {
//         PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.threePieceBottomString);
//         Pose2d shootPose = path.getPreviewStartingHolonomicPose();
//         robotDrive.resetEstimatedPose(shootPose);

//         //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
//         return new SequentialCommandGroup(
//             shootOnStart(),
//             new ParallelCommandGroup(
//                 robotDrive.followAutonPath(path),
//                 new SequentialCommandGroup(
//                     intakeBottomNote(),
//                     shoot(shootPose),
//                     intakeMiddleNote(),
//                     shoot(shootPose)
//                 )
//             ));
            
//     }

//     public Command bottomThreeCenterMiddleAuto() {
//         PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.bottomThreeCenterMiddleString);
//         Pose2d shootPose = path.getPreviewStartingHolonomicPose();
//         robotDrive.resetEstimatedPose(shootPose);

//         //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
//         return new SequentialCommandGroup(
//             shootOnStart(),
//             new ParallelCommandGroup(
//                 robotDrive.followAutonPath(path),
//                 new SequentialCommandGroup(
//                     intakeBottomNote(),
//                     shoot(shootPose),
//                     intakeCenterMiddleNote(),
//                     shoot(shootPose)
//                 )
//             ));
            
//     }

//     public Command fourPieceBottomAuto() {
//         PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.fourPieceBottomString);
//         Pose2d shootPose = path.getPreviewStartingHolonomicPose();
//         robotDrive.resetEstimatedPose(shootPose);

//         //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
//         return new SequentialCommandGroup(
//             shootOnStart(),
//             new ParallelCommandGroup(
//                 robotDrive.followAutonPath(path),
//                 new SequentialCommandGroup(
//                     intakeBottomNote(),
//                     shoot(shootPose),
//                     intakeMiddleNote(),
//                     shoot(shootPose),
//                     intakeTopNote(),
//                     shoot(shootPose)
//                 )
//             ));
            
//     }

//     public Command fivePieceMiddleAuto(){
//         PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.fivePieceMiddleString);
//         Pose2d shootPose = path.getPreviewStartingHolonomicPose();
//         robotDrive.resetEstimatedPose(shootPose);

//         //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
//         return new SequentialCommandGroup(
//             shootOnStart(),
//             new ParallelCommandGroup(
//                 robotDrive.followAutonPath(path),
//                 new SequentialCommandGroup(
//                     intakeMiddleNote(),
//                     shoot(shootPose),
//                     intakeCenterMiddleNote(),
//                     shoot(shootPose),
//                     intakeTopNote(),
//                     new SequentialCommandGroup(
//                         new WaitUntilConditionCommand(()->robotDrive.atPose(shootPose, 0.1, 10)),
//                         transfer.shootCommand()
//                     ),
//                     intakeBottomNote(),
//                     shoot(shootPose)
//                 )
//             )
//         );
//     }

}

