// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.*;


public class ComboCommands{

    Intake intake;
    Elevator elevator;
    Transfer transfer;
    Shooter shooter;

    boolean atAmp = false;

    public ComboCommands(Elevator elevator, Intake groundIntake, Transfer transfer, Shooter shooter){
        this.elevator = elevator;
        this.intake = groundIntake;
        this.transfer = transfer;
        this.shooter = shooter;

        atAmp = false;
    }


    public ParallelCommandGroup ejectTransferShooter(){
        return new ParallelCommandGroup(
            intake.startSpin(-.3),
            transfer.runTransferCommand(false)
        );
    }

    
//POSSIBLE BACKUP EJECT
    // public SequentialCommandGroup ejectDownTransferShooter(){
    //     return intake.flipToGroundCommand().andThen (new ParallelCommandGroup(
    //         intake.startSpin(-IntakeConstants.groundIntakeSpeed),
    //         transfer.runTransferCommand(false)
    //     ));
    // }

    public ParallelCommandGroup stopEjectTransferShooter(){
        return new ParallelCommandGroup(
            intake.stopSpin(),
            new InstantCommand(()-> transfer.stopTransfer())
        );
    }

    
    // public SequentialCommandGroup stopEjectDownTransferShooter(){
    //     return new ParallelCommandGroup(
    //         intake.stopSpin(),
    //         new InstantCommand(()-> transfer.stopTransfer())
    //     ).andThen(intake.flipToStowCommand());
    // }
    /**
     * @return Command group to flip, put elevator down, and start intake to first beam break
     */
    public ParallelCommandGroup startAmpIntakeCommand() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.intakeToBeamBreak(), 
                0, 
                0
                )
        );
    }

     /**
     * @return Command group to flip, put elevator down, and start intake to the shooter beam break
     */
    public ParallelCommandGroup startShooterIntakeCommand() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.startSpin(IntakeConstants.groundIntakeSpeed), 
                0, 
                0
            ),
            transfer.intakeToShooterCommand()
        );
    } 

     /**
     * @return Command group to flip, put elevator down, and start intake to middle
     */
    public ParallelCommandGroup startMiddleIntakeCommand() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.intakeToMiddle(), 
                0, 
                0
            )
        );
    }

     /**
     * @return Command group to stop rollers and flip to stow
     */
    public ParallelCommandGroup stopIntakeCommand() {
        return new ParallelCommandGroup(
            intake.flipToStowAndRunPayloadCommand(
                intake.stopSpin(), 
                0, 
                0
            )
        );
    }

    /**
     * @return Command group to stop rollers and flip to stow
     */
    public ParallelCommandGroup stopIntakeWithTransferRunningCommand() {
        return new ParallelCommandGroup(
            intake.flipToGroundAndRunPayloadCommand(
                intake.startSpin(IntakeConstants.groundIntakeSpeed), 
                0, 
                0
            ).until(intake::intakeEmpty)
            .andThen(
            intake.flipToStowAndRunPayloadCommand(
                intake.stopSpin(), 
                0, 
                0
            )),
            transfer.intakeToShooterCommand()
        );
    }

     /**
     * @return Command group to move elevator to amp position, flip to amp position, and eject note
     */
    public ParallelCommandGroup ampCommands() {
        return new ParallelCommandGroup(
            elevator.goToAmpCommand(),
            intake.runPayload(intake.flipToAmpCommand())
            .until(elevator::atSetpoint)
            .andThen(
                intake.flipToAmpAndRunPayloadCommand(
                    intake.startSpin(
                        -IntakeConstants.groundIntakeSpeed),
                    0,
                    0
                )
            )
        );
    }

    public ParallelCommandGroup topAmpCommands() {
        return new ParallelCommandGroup(
            elevator.goToTopAmpCommand(),
            intake.runPayload(intake.flipToStowCommand())
            .until(elevator::atSetpoint)
            .andThen(
                intake.flipToStowAndRunPayloadCommand(
                    intake.startSpin(
                        IntakeConstants.groundIntakeSpeed),
                    0,
                    0
                )
            )
        );
    }

    public Command goToTopAmp() {
        return new ParallelCommandGroup(
            elevator.goToTopAmpCommand(),
            intake.runPayload(intake.flipToAmpCommand())
        );
    }

    public Command scoreCommand() {
        return new ParallelCommandGroup(
            transfer.shootCommand(),
            intake.flipToAmpAndRunPayloadCommand(intake.startSpin(IntakeConstants.groundIntakeSpeed), 0, 0)
        );
    }

    public Command trapCommand() {
        return new SequentialCommandGroup(
   
            intake.partialEjectCommand(),
            new ParallelCommandGroup(
                elevator.elevatorTrapCommand(),
                intake.trapCommand(elevator::atTop)
            )

        );
    }
   

     /**
     * @return Command group to move elevator down, flip intake down, and transfer to shooter beam break
     */
    public SequentialCommandGroup noteTransferToShooter() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.startSpin(IntakeConstants.groundIntakeSpeed).until(intake::intakeEmpty), 
                1, 
                0
            ),
            transfer.intakeToShooterCommand()
        ).until(transfer::shooterFull)
        .andThen(this.stowCommand());
    }

     /**
     * @return Command group to move elevator down, flip intake down, and transfer to intake beam break
     */
    public Command noteTransferToIntake() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.reverseIntakeToBeamBreak(),
                0, 
                0
            ),
            new SequentialCommandGroup(
                new WaitCommand(TransferConstants.transferWaitTimeToIntake),
                transfer.shooterToIntakeCommand(intake::outerIntakeFull)
            )
            
       ).until(intake::outerIntakeFull)
        .andThen(this.stowCommand());
    }



    public Command stowCommand(){
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToStowAndRunPayloadCommand(
                intake.stopSpin(), 
                0, 
                0
            )
        );
    }
}

