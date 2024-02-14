// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ComboCommands{

  Intake intake;
  Elevator elevator;
  Transfer transfer;
  Shooter shooter;

  public ComboCommands(Elevator elevator, Intake groundIntake, Transfer transfer, Shooter shooter){
    this.elevator = elevator;
    this.intake = groundIntake;
    this.transfer = transfer;
    this.shooter = shooter;
  }

  public ParallelCommandGroup startAmpIntakeCommand() {
    return new ParallelCommandGroup(
           elevator.goToBottomCommand(),
            intake.flipToGround(),
            intake.intakeUntilBeamBreak()
    );
}

public ParallelCommandGroup startShooterIntakeCommand() {
    return new ParallelCommandGroup(
          //  elevator.goToBottomCommand(),
            intake.flipToGround(),
            intake.intakeCommand(),
            transfer.intakeToShooterCommand()
    );
}

public ParallelCommandGroup startMiddleIntakeCommand() {
    return new ParallelCommandGroup(
        //    elevator.goToBottomCommand(),
            intake.flipToGround(),
            intake.intakeCommand(),
            intake.intakeToMiddleCommand()
    );
}

public ParallelCommandGroup stopIntakeCommand() {
    return new ParallelCommandGroup(
      intake.stopRollersCommand(),
      intake.flipToStow()
    );
}

public SequentialCommandGroup ampCommands() {
    return new ParallelCommandGroup(
            elevator.goToAmpCommand(),
            intake.flipToAmp()
    ).andThen(intake.ejectCommand());
}

public SequentialCommandGroup noteTransferToShooter() {
    return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGround()
    ).andThen(intake.intakeCommand().alongWith(transfer.intakeToShooterCommand()));
}

public SequentialCommandGroup noteTransferToIntake() {
    return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGround()
    ).andThen(transfer.shooterToIntakeCommand(() -> intake.outerIntakeFull()));
}




 
}
