// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.Callable;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommmands;
import frc.robot.commands.ComboCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLightVision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Forks;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final Forks forks = new Forks();
  
  

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // Secondary controller
  CommandXboxController secondaryController = new CommandXboxController(OIConstants.kSecondaryControllerPort);

  //combination commands class
  ComboCommands comboCommands = new ComboCommands(elevator, intake, transfer, shooter);

  AutoCommmands autoCommmands = new AutoCommmands(robotDrive, intake, elevator, transfer, shooter, comboCommands);


  // Auton selector for dashboard
  SendableChooser<Callable<Command>> autoSelector = new SendableChooser<Callable<Command>>();

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
    autoSelector.addOption("Shoot only", () -> autoCommmands.shootOnStart());
    autoSelector.addOption("TopShootAndLeave", () -> autoCommmands.shootAndLeaveTopAuto());
    autoSelector.addOption("BottomShootAndLeave", () -> autoCommmands.shootAndLeaveBottomAuto());
    autoSelector.addOption("MiddleThreePieceAmpSide", () -> autoCommmands.threePieceMiddleTopAuto());
    autoSelector.addOption("MiddleThreePieceStageSide", () -> autoCommmands.threePieceMiddleBottomAuto());
    autoSelector.addOption("MiddleFourPiece", ()-> autoCommmands.fourPieceMiddleAuto());
    autoSelector.addOption("TopTwoPiece", () -> autoCommmands.twoPieceTopAuto());
    autoSelector.addOption("BottomTwoPiece", () -> autoCommmands.twoPieceBottomAuto());
    autoSelector.addOption("MidlineThreePieceSource", ()-> autoCommmands.threePieceMidlineSourceAuto());
    autoSelector.addOption("MidlineThreePieceAmp", ()-> autoCommmands.threePieceMidlineAmpAuto());

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

          double powerIn = MathUtil.applyDeadband(-secondaryController.getRightY(), .1);
          // Booleans to limit elevator movment
          boolean goingDown = -secondaryController.getRightY() < 0;
          boolean goingUp = -secondaryController.getRightY() > 0;
          
          double limitedPower = (elevator.atBottom() && goingDown) || (elevator.atTop() && goingUp) ? 0 : powerIn;
          
          elevator.setElevatorPower(limitedPower); 
        }, elevator)
    );



    intake.setDefaultCommand(
      intake.intakeDefaultCommand(secondaryController::getLeftY, (elevator::elevatorLimitIntake))
    );

    shooter.setDefaultCommand(
      shooter.autoRevUp(
        () -> transfer.shooterFull() && robotDrive.atShootPose()
      )
    );


    SmartDashboard.putData(autoSelector);
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

    driverController.leftStick().whileTrue(
      new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                robotDrive::getOutputToAngle,
                true, true),
            robotDrive)
    );

    driverController.leftBumper()
    .onTrue(comboCommands.ampCommands())
    .onFalse(comboCommands.stowCommand());


    secondaryController.leftBumper()
    .onTrue(comboCommands.topAmpCommands())
    .onFalse(comboCommands.stowCommand());

    driverController.rightBumper()
    .onTrue(transfer.shootCommand());

    

    secondaryController.start().onTrue(new InstantCommand(()-> intake.resetArmEncoder(), intake));
  
   
  
            // hehehhheheehehhe  emmmie was here mwah ahhahahaha ()
    //  driverController.b().whileTrue(
    //   robotDrive.AmpLineupDynamicTrajectory()
    // );

    secondaryController.rightStick().onTrue(
      elevator.stopElevatorCommand()
    .andThen(new InstantCommand(() -> intake.stopArm(), intake)));

    /*◇─◇──◇─◇
      Transfer
    ◇─◇──◇─◇*/

    driverController.povRight().onTrue(comboCommands.noteTransferToShooter());
    driverController.povLeft().onTrue(comboCommands.noteTransferToIntake());

    secondaryController.povLeft().whileTrue(transfer.runTransferCommand(true));
    secondaryController.povRight().whileTrue(transfer.runTransferCommand(false));


    /*◇─◇──◇─◇
       Intake
    ◇─◇──◇─◇*/

    driverController.rightTrigger(0.1)
    .onTrue(comboCommands.startShooterIntakeCommand())
    .onFalse(comboCommands.stopIntakeWithTransferRunningCommand());

    driverController.leftTrigger(0.1)
    .onTrue(comboCommands.startAmpIntakeCommand())
    .onFalse(comboCommands.stopIntakeCommand());


    driverController.a()
    .onTrue(comboCommands.startMiddleIntakeCommand())
    .onFalse(comboCommands.stopIntakeCommand());


    secondaryController.leftStick().onTrue(new InstantCommand(()->intake.stopArm(), intake));

    secondaryController.y().whileTrue(
      new ParallelCommandGroup(
        shooter.shooterIntakeCommand(),
        transfer.runTransferCommand(false)
      )
    );

    secondaryController.rightTrigger(0.1)
    .onTrue(comboCommands.trapCommand());
    

    // driverController.y()
    // manuals shoot
    
    driverController.b()
    .onTrue(comboCommands.ejectTransferShooter())
    .onFalse(comboCommands.stopEjectTransferShooter());

    secondaryController.b()
    .onTrue(comboCommands.ejectTransferShooter())
    .onFalse(comboCommands.stopEjectTransferShooter());

    secondaryController.x()
    .whileTrue(intake.runPayload(intake.startSpin(1)));

    driverController.povUp()
    .onTrue(new ParallelCommandGroup(
      elevator.goToTopCommand(),
      intake.flipToClimbCommand()));

    // intake.setDefaultCommand(new RunCommand(() -> {
    //   intake.setArmSpeed(-secondaryController.getLeftY());    
    // }, intake));

    secondaryController.a()
    .onTrue(
      forks.spinServosCommand()).onFalse(forks.stopServosCommand());


    secondaryController.povUp().onTrue(shooter.shooterAutoLineRevUpCommand());
    secondaryController.povDown().onTrue(shooter.stopShooterCommand());

    // secondaryController.leftStick().onTrue(intake.stopIntakeArmCommand());

    //auto amp controls go heeeeeerreeeeeeeeee (on right trigger)
    //button box levels

    // Feedback
    driverController.leftTrigger(0.1).whileTrue(
      new RunCommand(
        () -> {
          if (intake.innerIntakeFull() || transfer.shooterFull()){
            driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
            //leds.set(true);
          }
        }
    ))
    .whileFalse(new InstantCommand(
      () -> {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
        //leds.set(false);
      }
    ));

    driverController.rightTrigger(0.1).whileTrue(
      new RunCommand(
        () -> {
          if (intake.innerIntakeFull() || transfer.shooterFull()){
            driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
            //leds.set(true);
          }
        }
      ))
      .whileFalse(new InstantCommand(
        () -> {
          driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
          //leds.set(false);
        }
      ));
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

    Command command = new InstantCommand();

     try{
      command = autoSelector.getSelected().call();
     }
     catch(Exception e){
      DriverStation.reportError("Auto be weird", e.getStackTrace());
     }
     

     return command;
  }

  /*◇─◇──◇─◇
      Autos\
  ◇─◇──◇─◇*/
  
  private static class Test4And1Auto extends SequentialCommandGroup{

    public Test4And1Auto(){
      addCommands(
        //new PathPlannerAuto("Test4+1Auto")
      );
    }
  }
}
