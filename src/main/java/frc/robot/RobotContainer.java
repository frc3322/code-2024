// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DynamicSwerveControllerCommand;
import frc.robot.Constants.ElevatorConstants;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.IO.GyroIO;
import frc.robot.subsystems.drive.IO.GyroIONavX;
import frc.robot.subsystems.drive.IO.ModuleIO;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOSparkMax;
//import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive;
   
  private final Elevator elevator = new Elevator();

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // Secondary controller
  CommandXboxController secondaryController = new CommandXboxController(OIConstants.kSecondaryControllerPort);

  // Auton selector for dashboard
  LoggedDashboardChooser<SequentialCommandGroup> autoSelector = new LoggedDashboardChooser<>("Auto chooser");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    /*◇─◇──◇─◇
     Subsystems
    ◇─◇──◇─◇*/

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        robotDrive = new DriveSubsystem(
          new ModuleIOSparkMax(CANIds.kFrontLeftDrivingCanId, CANIds.kFrontLeftTurningCanId),
          new ModuleIOSparkMax(CANIds.kFrontRightDrivingCanId, CANIds.kFrontRightTurningCanId),
          new ModuleIOSparkMax(CANIds.kRearLeftDrivingCanId, CANIds.kRearLeftTurningCanId),
          new ModuleIOSparkMax(CANIds.kRearRightDrivingCanId, CANIds.kRearRightTurningCanId),
          new GyroIONavX(Constants.DriveConstants.kGyroReversed)
        );
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        robotDrive = new DriveSubsystem(
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new GyroIO() {}
        );
        break;

      default:
        // Replayed robot, disable IO implementations
        robotDrive = new DriveSubsystem(
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new GyroIO() {}
        );
        break;
    }

    // Configure the button bindings
    configureButtonBindings();

    // Auton selector config
    autoSelector.addDefaultOption("Slow forward 5 meters", new SlowForward5Meters());

    // Configure default commands

    /*◇─◇──◇─◇
     Drivetrain
    ◇─◇──◇─◇*/

    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY() /2, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX() /2, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX() /2, OIConstants.kDriveDeadband),
                true, true),
            robotDrive));

    /*◇─◇──◇─◇
      Elevator
    ◇─◇──◇─◇*/

    elevator.setDefaultCommand(
      // Right stick on secondary controller is used for manual elevator. There are limits so it does not hit the top
      new RunCommand(
        () -> { 

          double powerIn = -MathUtil.applyDeadband(secondaryController.getRightY() / 4, .1);
          // Booleans to limit elevator movment
          boolean goingDown = -secondaryController.getRightY() < 0;
          boolean goingUp = -secondaryController.getRightY() > 0;
          boolean aboveLimit = elevator.getElevatorLeftEncoder() > ElevatorConstants.elevatorTopLimit;
          
          double limitedPower = (elevator.atBottom() && goingDown) || (aboveLimit && goingUp) ? 0 : powerIn;
          
          elevator.setMotor(limitedPower); 
        }, elevator)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /*◇─◇──◇─◇
     Drivetrain
    ◇─◇──◇─◇*/

    driverController.x()
        .whileTrue(new RunCommand(
            () -> robotDrive.setX(),
            robotDrive));

    driverController.start().onTrue(new InstantCommand(()->robotDrive.zeroHeading()));

    driverController.rightBumper().whileTrue(
      new DynamicSwerveControllerCommand(
          robotDrive::ZeroZeroDynamicTrajectory,
          robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          Trajectories.getDefaultThetaController(),
          robotDrive::setModuleStates,
          robotDrive)
    );
            
  
  }
  
  // public void updateLogger() {
  //   Logger.updateEntries();
  // }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.get();
  }

  /*◇─◇──◇─◇
      Autos
  ◇─◇──◇─◇*/
  
  private class SlowForward5Meters extends SequentialCommandGroup {
    
    public SlowForward5Meters(){
      addCommands(
        new InstantCommand( () -> {
          robotDrive.resetOdometry(Trajectories.slowForward4Meters().getInitialPose());
        }),

        new SwerveControllerCommand(
          Trajectories.slowForward4Meters(),
          robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          Trajectories.getDefaultThetaController(),
          robotDrive::setModuleStates,
          robotDrive)
      );
    }
  }
}
