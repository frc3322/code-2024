// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.ElevatorConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The elevator subsystem for 3322's 2024 robot.
 */
public class Elevator extends SubsystemBase implements Loggable {
  
  private final CANSparkMax elevatorLeftMotor = new CANSparkMax(CANIds.kElevatorLeftCanId, MotorType.kBrushless);
  private final CANSparkMax elevatorRightMotor = new CANSparkMax(CANIds.kElevatorRightCanId, MotorType.kBrushless);

  private final RelativeEncoder elevatorRightEncoder = elevatorRightMotor.getEncoder();

  private final DigitalInput elevatorLimitSwitch = new DigitalInput(0);

  //boolean climbUp = false;
  
  private final ProfiledPIDController elevatorPidController = new ProfiledPIDController(
    ElevatorConstants.elevatorP,
    ElevatorConstants.elevatorI,
    ElevatorConstants.elevatorD,
    new Constraints(
      ElevatorConstants.velocityConstraint, 
      ElevatorConstants.accelerationConstraint
    )
  );

  private final ProfiledPIDController elevatorClimbController = new ProfiledPIDController(
    ElevatorConstants.elevatorP, 
    ElevatorConstants.elevatorI,
    ElevatorConstants.elevatorD, 
    new Constraints(
      ElevatorConstants.velocityConstraint, 
      ElevatorConstants.slowAccelerationConstraint));
  
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorLeftMotor.restoreFactoryDefaults();
    elevatorRightMotor.restoreFactoryDefaults();

    elevatorLeftMotor.setIdleMode(IdleMode.kBrake);
    elevatorRightMotor.setIdleMode(IdleMode.kBrake);

    elevatorLeftMotor.setInverted(true);
    elevatorRightMotor.follow(elevatorLeftMotor, true);

    elevatorLeftMotor.setSmartCurrentLimit(80);
    elevatorRightMotor.setSmartCurrentLimit(80);

    elevatorRightEncoder.setPositionConversionFactor(ElevatorConstants.elevatorGearRatio);

    elevatorLeftMotor.burnFlash();
    elevatorRightMotor.burnFlash();

    SmartDashboard.putData("ElevatorPID", elevatorPidController);
    
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
    return elevatorPidController.getGoal().position;
  }

  /**
   * Get the position of the left elevator motor encoder.
   * @return The position of the elevator encoder.
   */
  @Log
  public double getElevatorEncoderPosition() {
    return elevatorRightEncoder.getPosition();
  }

  @Log
  public Boolean elevatorLimiSwitchPressed(){
    return elevatorLimitSwitch.get();
  }

  /**
   * Returns a boolean representing if the elevator position is below the bottom threshold
   * @return A boolean represening if the elevator is at the bottom
   */
  public boolean atBottom() {
    return getElevatorEncoderPosition() < ElevatorConstants.elevatorBottomThreshold;
  }

  public boolean atSetpoint() {
    return Math.abs(getElevatorEncoderPosition() - getSetpoint()) < ElevatorConstants.elevatorPositionThreshold;
  }

  public boolean atAmp() {
    return Math.abs(getElevatorEncoderPosition() - ElevatorConstants.elevatorAmpPosition) < ElevatorConstants.elevatorPositionThreshold;
  }

  /**
   * Returns a boolean representing if the elevator position is above the top threshold
   * @return A boolean represening if the elevator is at the top
   */
  @Log
   public boolean atTop() {
    return getElevatorEncoderPosition() > ElevatorConstants.elevatorTopThreshold;
  }

  @Log
  public boolean elevatorLimitIntake() {
    return getElevatorEncoderPosition() > ElevatorConstants.elevatorOnChainPosition;
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  /**
   * Set the power of both elevator motors, with one inverted.
   * @param power The power for both motors.
   */
  public void setElevatorPower(double power) {
    elevatorLeftMotor.set(power);
  }

  /**
   * Stop both elevator motors.
   */
  public void stopElevator(){
    elevatorLeftMotor.stopMotor();
  }

  /**
   * Set the setpoint of the elevator PID controller.
   * @param setpoint The setpoint of the PID controller.
   */
  @Config
  public void setElevatorSetpoint(double setpoint) {
    elevatorPidController.setGoal(setpoint);
  }

  /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

  /**
   * Returns a command that uses the elevator PID command to go to its setpoint until interuppeted.
   * @return A run command that moves the elevator towards its setpoint.
   */
  @Config
  public Command goToSetpoint() {
    return new RunCommand(
      () -> {
        elevatorLeftMotor.set(elevatorPidController.calculate(getElevatorEncoderPosition()));
        // Not working? Are your PID constants 0? Are your constraints 0? Are your setpoints 0?
      }, 
      this
    );
  }

  /**
   * A command that sets the PID goal to the bottom position and moves to it.
   * @return A sequential command group that moves the elevator to the bottom position.
   */
  public Command goToBottomCommand() {
    return new SequentialCommandGroup(
      new InstantCommand( 
        () -> setElevatorSetpoint(ElevatorConstants.elevatorBottomPosition)
      ),
      goToSetpoint()
    );
  }

  /**
   * A command that sets the PID goal to the AMP position and moves to it.
   * @return A sequential command group that moves the elevator to the AMP position.
   */
  public Command goToAmpCommand() {
    return new SequentialCommandGroup(
      new InstantCommand( 
        () -> setElevatorSetpoint(ElevatorConstants.elevatorAmpPosition)
      ),
      goToSetpoint()
    );
  }

   public Command goToTopAmpCommand() {
    return new SequentialCommandGroup(
      new InstantCommand( 
        () -> setElevatorSetpoint(ElevatorConstants.elevatorTopAmpPosition)
      ),
      goToSetpoint()
    );
  }

  /**
   * A command that sets the PID goal to the top position and moves to it.
   * @return A sequential command group that moves the elevator to the top position.
   */
  public Command goToTopCommand() {
    return new SequentialCommandGroup(
      new InstantCommand( 
        () -> setElevatorSetpoint(ElevatorConstants.elevatorTopPosition)
      ),
      goToSetpoint()
    );
  }

  public Command elevatorTrapCommand() {
    return new RunCommand(
      () -> setElevatorPower(elevatorClimbController.calculate(
        getElevatorEncoderPosition(), 
        ElevatorConstants.elevatorTopPosition
      )), 
      this);
  }

  // public Command elevatorStartClimbCommand(){
    
  //   return new SequentialCommandGroup(
  //     new InstantCommand(() -> climbUp = !climbUp),
  //     new RunCommand(
  //       () -> {
  //         if(climbUp){
  //           setElevatorPower(elevatorPidController.calculate(
  //             getElevatorEncoderPosition(), 
  //             ElevatorConstants.elevatorTopPosition
  //           ));
  //         }
  //         else{
  //           setElevatorPower(elevatorPidController.calculate(
  //             getElevatorEncoderPosition(), 
  //             ElevatorConstants.elevatorOnChainPosition
  //           ));
  //         }
  //       },
  //       this
  //     )
  //   )
  //   .handleInterrupt(() -> climbUp = false);
  // }
  
  // public Command elevatorClimbCommand() {
  //   return new RunCommand(
  //     () -> setElevatorPower(elevatorClimbController.calculate(
  //       getElevatorEncoderPosition(), 
  //       ElevatorConstants.elevatorBottomPosition
  //     )), 
  //     this);
  // }

  /**
   * An instant command that stops the elevator. Can be used to return to manual control.
   * @return An instant command that stops the elevator
   */
  public Command stopElevatorCommand() {
    return new InstantCommand(
      () -> {
        stopElevator();
      },
      this
    );
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    
  }
}
