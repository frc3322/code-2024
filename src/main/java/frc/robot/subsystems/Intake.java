// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NeoMotorConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable {
  /** Creates a new GroundIntake. */

  // Creates motor objects
  private final CANSparkMax wheelsMotor = new CANSparkMax(Constants.CANIds.kIntakeWheelsCanId, MotorType.kBrushless);
  private final CANSparkMax intakeArmLeft = new CANSparkMax(Constants.CANIds.kIntakePivotLeftCanId, MotorType.kBrushless);
  private final CANSparkMax intakeArmRight = new CANSparkMax(Constants.CANIds.kIntakePivotRightCanId, MotorType.kBrushless);


  // Creates encoder for intakeArm
  private final RelativeEncoder armLeftEncoder = intakeArmLeft.getEncoder();
  private final RelativeEncoder armRightEncoder = intakeArmRight.getEncoder();


  // Creates the intake beam sensor objects
  private final DigitalInput intakeOuterBeamBreak = new DigitalInput(1);
  private final DigitalInput intakeInnerBeamBreak = new DigitalInput(2);


  // Creates a new Profiled PID Controller
  public final ProfiledPIDController intakePIDController = new ProfiledPIDController(
    IntakeConstants.intakeP, 
    IntakeConstants.intakeI, 
    IntakeConstants.intakeD, 
    new Constraints(
      IntakeConstants.velocityConstraint, 
      IntakeConstants.accelerationConstraint
      )
    );

  public final ProfiledPIDController slowIntakePIDController = new ProfiledPIDController(
    IntakeConstants.intakeP, 
    IntakeConstants.intakeI, 
    IntakeConstants.intakeD, 
    new Constraints(
      IntakeConstants.velocityConstraint, 
      IntakeConstants.slowAccelerationConstraint
      )
    );

  // GroundIntake Constructor
  public Intake() {
    intakeArmLeft.restoreFactoryDefaults();
    intakeArmRight.restoreFactoryDefaults();

    wheelsMotor.setIdleMode(IdleMode.kCoast);
    intakeArmLeft.setIdleMode(IdleMode.kBrake);
    intakeArmRight.setIdleMode(IdleMode.kBrake);

    armLeftEncoder.setPositionConversionFactor(IntakeConstants.kIntakeArmGearRatio);
    armLeftEncoder.setVelocityConversionFactor(IntakeConstants.kIntakeArmGearRatio);

    
    armRightEncoder.setPositionConversionFactor(IntakeConstants.kIntakeArmGearRatio);
    armRightEncoder.setVelocityConversionFactor(IntakeConstants.kIntakeArmGearRatio);

    intakeArmLeft.setInverted(true);
    intakeArmRight.follow(intakeArmLeft, true);

    wheelsMotor.setInverted(true);

    intakeArmLeft.setSmartCurrentLimit(NeoMotorConstants.currentLimit);
    intakeArmRight.setSmartCurrentLimit(NeoMotorConstants.currentLimit);
    wheelsMotor.setSmartCurrentLimit(NeoMotorConstants.neo550CurrentLimitAmps);
    wheelsMotor.burnFlash();
    
    intakeArmLeft.burnFlash();
    intakeArmRight.burnFlash();

    intakePIDController.setTolerance(IntakeConstants.kPosToleranceDeg, IntakeConstants.kTurnRateToleranceDegPerS);
    slowIntakePIDController.setTolerance(IntakeConstants.kPosToleranceDeg, IntakeConstants.kTurnRateToleranceDegPerS);

    SmartDashboard.putData("intake", intakePIDController);
  }


  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/

  /**
   * Returns the current position goal of the profiled PID.
   * @return The position goal of the elevator PID.
   */
  @Log
  public double getSetpoint() {
    return intakePIDController.getGoal().position;
  }

  /**
   * Returns the encoder position of the left arm encoder.
   * @return The position of the intake arm encoder.
   */
  @Log
  public double getIntakeEncoderPosition() {
    return armLeftEncoder.getPosition();
  }





  @Log
  public Boolean intakeEmpty(){
    return intakeInnerBeamBreak.get() && intakeOuterBeamBreak.get();
  }

  /**
   * Returns true if both of the beams detect a ring, returning false otherwise
   * @return Whether or not the intake has (true) or doesn't have (false) a ring
   */
  @Log
  public Boolean intakeFull() {
    return !intakeInnerBeamBreak.get() && !intakeOuterBeamBreak.get();
  }

  @Log
  public Boolean outerIntakeFull() {
    return !intakeOuterBeamBreak.get();
  }

  @Log public boolean innerIntakeFull() {
    return !intakeInnerBeamBreak.get();
  }

  @Log
  public boolean atSetpoint() {
    return intakePIDController.atGoal();
  }

  @Log public boolean climbAtSetpoint() {
    return slowIntakePIDController.atGoal();
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  /**
   * Set the setpoint of the intake PID controller. 
   * Unit is Rotations?
   * @param setpoint The setpoint of the PID controller.
   */
  public void setArmSetpoint(double setpoint) {
    intakePIDController.setGoal(setpoint);
  }

  /**
   * Set the power of both intake arm motors, with the right one inverted.
   * @param power The power for both motors.
   */
  public void setArmSpeed(double speed){
    intakeArmLeft.set(speed);
    intakeArmRight.set(speed);
  }

  public void setWheelSpeed(double power) {
    wheelsMotor.set(power);
  }

  /**
   * Stop both intake motors.
   */
  public void stopArm() {
    intakeArmLeft.stopMotor();
    intakeArmRight.stopMotor();
  }

  /**
   * Sets the position of the left intake arm encoder to 0
   */
  public void resetArmEncoder() {
    armLeftEncoder.setPosition(0);
  }

  /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

  public Command intakeDefaultCommand(DoubleSupplier joystickInput, BooleanSupplier elevatorTooHigh){
    return run(() ->{
      if (elevatorTooHigh.getAsBoolean()){
        setArmSpeed(intakePIDController.calculate(getIntakeEncoderPosition(), IntakeConstants.climbPosition));
      }
      else if(!elevatorTooHigh.getAsBoolean()){
        setArmSpeed(joystickInput.getAsDouble());
      }
    });
  }

  /**
   * Returns a command that uses the intake arm PID command to go to its setpoint until interuppted.
   * @return A run command that moves the intake arm towards its setpoint.
   */
  public Command flipToSetpoint() {
    return new RunCommand(
      () -> {
        setArmSpeed((intakePIDController.calculate(getIntakeEncoderPosition())));
      }
    );
  }

  // intake to beam break 
  public Command intakeToBeamBreak(){
    return new StartEndCommand(
      ()-> setWheelSpeed(IntakeConstants.groundIntakeSpeed),
      ()-> setWheelSpeed(0)
    ).until(this::innerIntakeFull);
  }

  public Command reverseIntakeToBeamBreak(){
    return new StartEndCommand(
      ()-> setWheelSpeed(-IntakeConstants.groundIntakeSpeed),
      ()-> setWheelSpeed(0)
    ).until(this::outerIntakeFull);
  }

  /**
   * Runs continuously until it is holding a note, then runs until the note is out of the intake and in the transfer
   * @return A sequential Command Group
   */
  public Command intakeToMiddle(){
    return new SequentialCommandGroup(
      new RunCommand(
        ()-> setWheelSpeed(IntakeConstants.groundIntakeSpeed))
      .until(this::innerIntakeFull), 
      new StartEndCommand(
        ()-> setWheelSpeed(IntakeConstants.groundIntakeSpeed),
        ()-> stopSpin()
      )
      .until(this::intakeEmpty)
      );

  
    }
  /**
   * Starts spinning continuously with normal intake speed.
   * @return an InstantCommand
   */
  public Command startSpin(double power){
    return new InstantCommand(
      ()-> setWheelSpeed(power)
    );
  }
  
/**
 * Stops intaking.
 * @return an InstantCommand
 */
  public Command stopSpin(){
    return new InstantCommand(
      ()-> setWheelSpeed(0)
    );
  }

  /**
   * Ejects piece from intake.
   * @return a StartEndCommand
   */
  public Command ejectCommand(){
    return new StartEndCommand(
      ()->setWheelSpeed(-IntakeConstants.groundIntakeSpeed),
      ()->setWheelSpeed(0))
      .until(this::intakeEmpty);
  }

  /**
   * Brings the intake to the stow position.
   * @return A sequential command group
   */
  public Command flipToStowCommand(){
    return new SequentialCommandGroup(
      new InstantCommand(
        () -> setArmSetpoint(IntakeConstants.stowPosition)
      ),
      flipToSetpoint()
    );
  }

  /**
   * Brings the intake to the ground position.
   * @return A sequential command group
   */
      
  public Command flipToGroundCommand(){
    return new SequentialCommandGroup(
      new InstantCommand(
        () -> setArmSetpoint(IntakeConstants.groundPosition)
      ),
      flipToSetpoint()
    );
  }

  public Command flipToAmpCommand(){
    return new SequentialCommandGroup(
      new InstantCommand(
        () -> setArmSetpoint(IntakeConstants.ampPosition)
      ),
      flipToSetpoint()
    );
  }

  public Command flipToClimbCommand() {
    
    return new InstantCommand(() -> slowIntakePIDController.setGoal(IntakeConstants.climbPosition)).andThen(
    new RunCommand(() -> {
      setArmSpeed(
        slowIntakePIDController.calculate(getIntakeEncoderPosition())
      );
    }));
  }

  
  public Command lowFlipToClimbCommand() {
    return new InstantCommand(() -> slowIntakePIDController.setGoal(IntakeConstants.lowClimbPosition)).andThen(
    new RunCommand(() -> {
      setArmSpeed(
        slowIntakePIDController.calculate(getIntakeEncoderPosition())
      );
    }));
  }

  public Command flipToTrapCommand() {
    return new InstantCommand(() -> slowIntakePIDController.setGoal(IntakeConstants.trapPosition)).andThen(
    new RunCommand(() -> {
      setArmSpeed(
        slowIntakePIDController.calculate(getIntakeEncoderPosition())
      );
    }));
  }

/**
 * Runs a "payload" (eject or run intake) and brings the arm to the stow position with a set amount of delay on each.
 * @param payload Command to run
 * @param payloadDelay Delay on payload command
 * @param armDelay Delay before moving intake to stow position
 * @return A parallel command group
 */
  public Command flipToStowAndRunPayloadCommand(Command payload, double payloadDelay, double armDelay) {
    Command command = new ParallelCommandGroup(
    new SequentialCommandGroup(
        new WaitCommand(payloadDelay),
        payload
      ),
      new SequentialCommandGroup(
        new WaitCommand(armDelay),
        flipToStowCommand()
      )
    );
    command.addRequirements(this);
    return command;
  }

/**
 * Runs a "payload" (eject or run intake) and brings the arm to the ground with a set amount of delay on each.
 * @param payload Command to run
 * @param payloadDelay Delay on payload command
 * @param armDelay Delay before moving intake to ground
 * @return A parallel command group
 */
  public Command flipToGroundAndRunPayloadCommand(Command payload, double payloadDelay, double armDelay) {
    Command command = new ParallelCommandGroup(
      new SequentialCommandGroup(
        new WaitCommand(payloadDelay),
        payload
      ),
      new SequentialCommandGroup(
        new WaitCommand(armDelay),
        flipToGroundCommand()
      )
    );
    command.addRequirements(this);
    return command;
  }

  public Command flipToClimbAndRunPayloadCommand(Command payload, double payloadDelay, double armDelay) {
    Command command = new ParallelCommandGroup(
      new SequentialCommandGroup(
        new WaitCommand(payloadDelay),
        payload
      ),
      new SequentialCommandGroup(
        new WaitCommand(armDelay),
        flipToClimbCommand()
      )
    );
    command.addRequirements(this);
    return command;
  }

  public Command flipToAmpAndRunPayloadCommand(Command payload, double payloadDelay, double armDelay) {
    Command command = new ParallelCommandGroup(
    new SequentialCommandGroup(
        new WaitCommand(payloadDelay),
        payload
      ),
      new SequentialCommandGroup(
        new WaitCommand(armDelay),
        flipToAmpCommand()
      )
    );
    command.addRequirements(this);
    return command;
  }

  public Command flipToTrapAndRunPayloadCommand(Command payload, double payloadDelay, double armDelay) {
    Command command = new ParallelCommandGroup(
    new SequentialCommandGroup(
        new WaitCommand(payloadDelay),
        payload
      ),
      new SequentialCommandGroup(
        new WaitCommand(armDelay),
        flipToTrapCommand()
      )
    );
    command.addRequirements(this);
    return command;
  }

  /**
   * Run a command requiring intake.
   * @param payload The command that needs requirements.
   * @return The command with requirements added.
   */
  public Command runPayload(Command payload){
    Command command = payload;
    payload.addRequirements(this);
    return command;
  }

  // public Command flipToGroundAndRunPayloadCommandAlternate(Command payload, double payloadDelay, double armDelay) {
  //   Command command = new ParallelCommandGroup(
  //     new SequentialCommandGroup(
  //       new WaitCommand(payloadDelay),
  //       payload
  //     ),
  //     new SequentialCommandGroup(
  //       new WaitCommand(armDelay),
  //       new InstantCommand(()-> setArmSetpoint(IntakeConstants.groundPosition)),
  //       new  RunCommand( ()->
  //         setArmSpeed(intakePIDController.calculate(getIntakeEncoderPosition())
  //         )
  //       )
  //     )
  //   );
  //   command.addRequirements(this);
  //   return command;
  // }

  public Command trapCommand(BooleanSupplier elevatorAtTop){
    return runPayload(flipToClimbCommand())
    .until(elevatorAtTop)
    .andThen(flipToTrapAndRunPayloadCommand(
      startSpin(-.7),
      IntakeConstants.trapDelay,
      0
    )).withTimeout(4)
    .andThen(runPayload(lowFlipToClimbCommand())
    .withTimeout(2)
    .andThen(runPayload(flipToTrapCommand())));
    
  }
  
  public Command superFlippyTrapCommand(BooleanSupplier elevatorAtTop){
    return new SequentialCommandGroup(
      runPayload(flipToClimbCommand()).until(elevatorAtTop),
      runPayload(flipToTrapCommand()).until(this::climbAtSetpoint),
      flipToTrapAndRunPayloadCommand(startSpin(-.3), 0, 0).withTimeout(.5),
      runPayload(stopSpin()),
      runPayload(flipToClimbCommand()).until(this::climbAtSetpoint),
      runPayload(flipToTrapCommand()).until(this::climbAtSetpoint),
      flipToTrapAndRunPayloadCommand(startSpin(-.3), 0, 0)
    );
  }
  

  //flip stow, flip to ground, flips down and spins untill beam break, flip down and spin without limits, start spin, stop spin

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
   
  }
}
