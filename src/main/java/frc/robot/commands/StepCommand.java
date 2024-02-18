// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class StepCommand extends Command {
  
  private Command toRun;
  private BooleanSupplier condition;

  private boolean hasRun = false;
  
  /**
   * A command compisition that waits to run until the condition is true.
   * @param toRun The Runnable to run.
   * @param condition The condition to wait for.
   * @param requirements The subsystems required by this command
   */
  public StepCommand(Command toRun, BooleanSupplier condition, Subsystem... requirements) {
    this.toRun = toRun;
    
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasRun = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (condition.getAsBoolean()){
      toRun.schedule();
      hasRun = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasRun;
  }
}
