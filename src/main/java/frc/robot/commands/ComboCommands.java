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

  GroundIntake groundIntake;
  Elevator elevator;
  Transfer transfer;
  Shooter shooter;

  public ComboCommands(GroundIntake groundIntake, Elevator elevator, Transfer transfer, Shooter shooter){
    this.groundIntake = groundIntake;
    this.elevator = elevator;
    this.transfer = transfer;
    this.shooter = shooter;
  }

  public ParallelCommandGroup startAmpIntakeCommand = new ParallelCommandGroup(
    elevator.goToBottomCommand(),
    groundIntake.flipToGround(),
    groundIntake.intakeUntilBeamBreak()
  );

  public ParallelCommandGroup startShooterIntakeCommand = new ParallelCommandGroup(
    elevator.goToBottomCommand(),
    groundIntake.flipToGround(),
    groundIntake.intakeCommand(),
    transfer.intakeToShooterCommand()
  );

    public ParallelCommandGroup startMiddleIntakeCommand = new ParallelCommandGroup(
    elevator.goToBottomCommand(),
    groundIntake.flipToGround(),
    groundIntake.intakeCommand(),
    groundIntake.intakeToMiddleCommand()
    
  );

  public ParallelCommandGroup stopIntakeCommand = new ParallelCommandGroup(
   // groundIntake.stopRollersCommand(),
   // groundIntake.flipUp()
  );

  public SequentialCommandGroup ampCommands =  new ParallelCommandGroup(
    elevator.goToAmpCommand(),
    groundIntake.flipToAmp()
  ).andThen(groundIntake.ejectCommand());

   public SequentialCommandGroup noteTransferToShooter =  new ParallelCommandGroup(
    elevator.goToBottomCommand(),
    groundIntake.flipToGround()
  ).andThen(groundIntake.intakeCommand()
  .alongWith(transfer.intakeToShooterCommand()));

   public SequentialCommandGroup noteTransferToIntake =  new ParallelCommandGroup(
    elevator.goToBottomCommand(),
    groundIntake.flipToGround()
  ).andThen(transfer.shooterToIntakeCommand(()->groundIntake.outerIntakeFull()));


 
}
