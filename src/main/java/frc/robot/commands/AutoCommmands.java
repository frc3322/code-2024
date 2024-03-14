package frc.robot.commands;


import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import io.github.oblarg.oblog.annotations.Log;

public class AutoCommmands {
    
    private final DriveSubsystem robotDrive;
    private final Shooter shooter;
    private final Transfer transfer;
    private final Intake intake;
    private final ComboCommands combo;

    public AutoCommmands(DriveSubsystem robotDrive, Intake intake, Elevator elevator, Transfer transfer, Shooter shooter, ComboCommands combo) {
        this.robotDrive = robotDrive;
        this.transfer = transfer;
        this.shooter = shooter;
        this.intake = intake;
        this.combo = combo;
    }

    public Command shootOnStart() {
        return new SequentialCommandGroup( 
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new WaitUntilConditionCommand(shooter::bothAtAutoRPM),
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

        public Command autoIntakeToMiddle() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                intake.intakeToMiddle(),
                intake.runPayload(intake.flipToGroundCommand()),
                transfer.runTransferCommand(true)
            ),
            new ParallelCommandGroup(
                transfer.intakeToShooterCommand(),
                intake.runPayload(intake.flipToStowCommand())
                .until(intake::atSetpoint)
                
            )
            
        );
    }

    public Command intakeTopNote() {
        Pose2d notePose = robotDrive.flipPoseIfRed(FieldConstants.blueTopNotePose);

        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(notePose, 1.5, 0)),
            autoIntakeToShooter()
        );

    }

    public Command intakeMiddleNote() {
        Pose2d notePose = robotDrive.flipPoseIfRed(FieldConstants.blueMiddleNotePose);

        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(notePose, 1.5, 0)),
            autoIntakeToShooter()
        );

    }

    public Command intakeBottomNote() {
        Pose2d notePose = robotDrive.flipPoseIfRed(FieldConstants.blueBottomNotePose);

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

    public Command intakeCenterMiddleTopNote() {
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.centerMidTopPose, 1.5, 0)),
            autoIntakeToShooter()
            );
    }

    public Command intakeCenterTopNote(){
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.centerTopPose, 1.5, 0)),
            autoIntakeToShooter()
            );
            
        }

    public Command intakeCenterMiddleBottomNote(){
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.centerMidBottomPose, 4, 0)),
            autoIntakeToShooter()
            );
            
        }

    public Command intakeCenterBottomNote(){
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(FieldConstants.centerBottomPose, 4, 0)),
            autoIntakeToShooter()
            );
            
        }

    public Command flipIntakeUp() {
        return combo.stowCommand().withTimeout(.6);
    }
        
    public Command shoot(Pose2d shootPose) {
        return new SequentialCommandGroup(
            new WaitUntilConditionCommand(()->robotDrive.atPose(shootPose, 0.3, 10)),
            transfer.shootCommand()
        );
    }

    //Full autos

    public Command threePieceMiddleTopAuto() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.threePieceMiddleTopString);
        Pose2d shootPose = path.getPreviewStartingHolonomicPose();
        robotDrive.resetEstimatedPose(shootPose);
        robotDrive.enableLimeLight(true);

        //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        return new SequentialCommandGroup(
            shootOnStart(),
            new ParallelCommandGroup(
                robotDrive.followAutonPath(path),
                new SequentialCommandGroup(
                    intakeMiddleNote(),
                    shoot(shootPose),
                    intakeTopNote(),
                    shoot(shootPose)
                )
            ));
            
    }

    public Command threePieceMiddleBottomAuto() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.threePieceMiddleBottomString);
        Pose2d shootPose = robotDrive.flipPoseIfRed(path.getPreviewStartingHolonomicPose());
        robotDrive.resetEstimatedPose(shootPose);
        robotDrive.enableLimeLight(false);


        //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        return new SequentialCommandGroup(
            shootOnStart(),
            new ParallelCommandGroup(
                robotDrive.followAutonPath(path),
                new SequentialCommandGroup(
                    intakeMiddleNote(),
                    shoot(shootPose),
                    intakeBottomNote(),
                    shoot(shootPose)
                )
            ));
            
    }

    public Command fourPieceMiddleAuto() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.fourPieceMiddleString);
        Pose2d shootPose = robotDrive.flipPoseIfRed(path.getPreviewStartingHolonomicPose());
        robotDrive.resetEstimatedPose(shootPose);
        robotDrive.enableLimeLight(true);

        //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        return new SequentialCommandGroup(
            shootOnStart(),
            new ParallelCommandGroup(
                robotDrive.followAutonPath(path),
                new SequentialCommandGroup(
                    intakeMiddleNote(),
                    shoot(shootPose),
                    intakeTopNote(),
                    new SequentialCommandGroup(
                        new WaitUntilConditionCommand(()->robotDrive.atPose(shootPose, 0.2, 10)),
                        transfer.shootCommand()
                    ),
                    intakeBottomNote(),
                    shoot(shootPose)
                )
            ));
            
    }


    public Command bottomTwoPlusOneCenterAuto() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.bottomTwoPlusOneCenterString);
        Pose2d shootPose = robotDrive.flipPoseIfRed(path.getPreviewStartingHolonomicPose());
        robotDrive.resetEstimatedPose(shootPose);
        robotDrive.enableLimeLight(true);

        //robotDrive.setYawToAngle(-path.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        return new SequentialCommandGroup(
            shootOnStart(),
            new ParallelCommandGroup(
                robotDrive.followAutonPath(path),
                new SequentialCommandGroup(
                    intakeCenterBottomNote(),
                    shoot(shootPose),
                    combo.startAmpIntakeCommand().until(intake::innerIntakeFull),
                    combo.stopIntakeCommand()
                )
            ));
            
    }

    public Command shootAndLeaveTopAuto() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.topLeaveString);
        Pose2d shootPose = robotDrive.flipPoseIfRed(path.getPreviewStartingHolonomicPose());
        robotDrive.resetEstimatedPose(shootPose);

        return new SequentialCommandGroup(
            shootOnStart(),
            robotDrive.followAutonPath(path)
        );
    }
    
    public Command shootAndLeaveBottomAuto() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.bottomLeaveString);
        Pose2d shootPose =robotDrive.flipPoseIfRed(path.getPreviewStartingHolonomicPose());
        robotDrive.resetEstimatedPose(shootPose);

        return new SequentialCommandGroup(
            shootOnStart(),
            robotDrive.followAutonPath(path)
        );
    }
}

