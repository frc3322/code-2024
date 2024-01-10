// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DynamicSwerveControllerCommand extends SwerveControllerCommand {
  /** Creates a new DynamicTrajectoryCommand. */
  public DynamicSwerveControllerCommand(
    Supplier<Trajectory> trajectorySupplier, 
    Supplier<Pose2d> poseSupplier,
    SwerveDriveKinematics driveKinematics,
    PIDController xController,
    PIDController yController,
    ProfiledPIDController thetaController,
    Consumer<SwerveModuleState[]> outputModuleStates,
    DriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
          trajectorySupplier.get(),
          poseSupplier,
          driveKinematics,
          xController,
          yController,
          thetaController,
          outputModuleStates,
          robotDrive
        );
      addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
