// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ComboCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLightVision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.Constants.*;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final Elevator elevator = new Elevator();
  private final LimeLightVision vision = new LimeLightVision();
  private final Transfer transfer = new Transfer();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // Secondary controller
  CommandXboxController secondaryController = new CommandXboxController(OIConstants.kSecondaryControllerPort);

  //combination commands class
  ComboCommands comboCommands = new ComboCommands(elevator, intake, transfer, shooter);

  // Auton selector for dashboard
  SendableChooser<SequentialCommandGroup> autoSelector = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    // Configure Oblog logger
    Logger.configureLoggingAndConfig(this, true);

    // Auton selector config
   //  autoSelector.setDefaultOption("Test", new Test4And1Auto());

    autoSelector.addOption("No auto", null);

    // Configure default commands

    /*◇─◇──◇─◇
     Drivetrain
    ◇─◇──◇─◇*/
    robotDrive.setVisionSystem(vision);

    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX() / 1.5, OIConstants.kDriveDeadband),
                true, true),
            robotDrive));

    /*◇─◇──◇─◇
      Elevator
    ◇─◇──◇─◇*/

    elevator.setDefaultCommand(
      // Right stick on secondary controller is used for manual elevator. There are limits so it does not hit the top
      new RunCommand(
        () -> { 

          double powerIn = MathUtil.applyDeadband(secondaryController.getRightY(), .1);
          // Booleans to limit elevator movment
          boolean goingDown = -secondaryController.getRightY() < 0;
          boolean goingUp = -secondaryController.getRightY() > 0;
          
          double limitedPower = (elevator.atBottom() && goingDown) || (elevator.atTop() && goingUp) ? 0 : powerIn;
          
          elevator.setElevatorPower(powerIn); 
        }, elevator)
    );



    intake.setDefaultCommand(
      new RunCommand((
        () -> intake.setArmSpeed(secondaryController.getLeftY())
      ),
      intake)
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

    secondaryController.leftBumper().onTrue(
      shooter.stopShooterCommand()
    );

    secondaryController.rightBumper().onTrue(
      shooter.shooterAutoLineRevUpCommand()
    );

    secondaryController.povDown().onTrue(
      intake.flipToGround()
    );

    secondaryController.povUp().onTrue(
      intake.flipToStow()
    );

    secondaryController.start().onTrue(new InstantCommand(()-> intake.resetArmEncoder(), intake));
    secondaryController.a().whileTrue(intake.intakeUntilBeamBreak());
  
   
  
            
    //  driverController.b().whileTrue(
    //   robotDrive.AmpLineupDynamicTrajectory()
    // );

    // /*◇─◇──◇─◇
    //  Transfer
    // ◇─◇──◇─◇*/

    // driverController.povLeft().onTrue(comboCommands.noteTransferToShooter());
    // driverController.povRight().onTrue(comboCommands.noteTransferToIntake());

    // secondaryController.leftBumper().whileTrue(new StartEndCommand(() -> {
    //   transfer.setTransferSpeeds(TransferConstants.transferSpeed);
    //   transfer.setShooterTransferSpeeds(TransferConstants.transferSpeed);
    // }, () -> {
    //   transfer.stopTransfer();
    //   transfer.stopShooterTransfer();
    // }, transfer));

    // secondaryController.rightBumper().whileTrue(new StartEndCommand(() -> {
    //   transfer.setTransferSpeeds(-TransferConstants.transferSpeed);
    //   transfer.setShooterTransferSpeeds(-TransferConstants.transferSpeed);
    // }, () -> {
    //   transfer.stopTransfer();
    //   transfer.stopShooterTransfer();
    // }, transfer));


    // /*◇─◇──◇─◇
    //  Intake
    // ◇─◇──◇─◇*/

    // driverController.leftBumper()
    // .onTrue(comboCommands.startShooterIntakeCommand())
    // .onFalse(comboCommands.stopIntakeCommand());

    // driverController.rightBumper()
    // .onTrue(comboCommands.startAmpIntakeCommand())
    // .onFalse(comboCommands.stopIntakeCommand());

    // driverController.a()
    // .onTrue(comboCommands.startMiddleIntakeCommand())
    // .onFalse(comboCommands.stopIntakeCommand());

    // driverController.y()
    // .onTrue(comboCommands.startAmpIntakeCommand())
    // .onFalse(comboCommands.stopIntakeCommand());
    
    //driverController.b().whileTrue(intake.ejectCommand());

    // intake.setDefaultCommand(new RunCommand(() -> {
    //   intake.setArmSpeed(-secondaryController.getLeftY());    
    // }, intake));

    // secondaryController.leftStick().onTrue(intake.stopIntakeArmCommand());

    //auto amp controls go heeeeeerreeeeeeeeee (on right trigger)
    //button box levels


  }
  public void updateLogger() {
    Logger.updateEntries();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    /*
     * !! ATTENTION !!
     * WE USE PATHPLANNER FOR OUR AUTO PATHS
     * https://github.com/mjansen4857/pathplanner
     */

     return new PathPlannerAuto("SegmentedTopFour");
  }

  /*◇─◇──◇─◇
      Autos\
  ◇─◇──◇─◇*/
  
  private static class Test4And1Auto extends SequentialCommandGroup{

    public Test4And1Auto(){
      addCommands(
        new PathPlannerAuto("Test4+1Auto")
      );
    }
  }
}
