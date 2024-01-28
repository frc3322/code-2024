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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.ElevatorConstants;

/**
 * The elevator subsystem for 3322's 2024 robot.
 */
public class Elevator extends SubsystemBase {
  
  private final CANSparkMax elevatorLeftMotor = new CANSparkMax(CANIds.kElevatorLeftCanId, MotorType.kBrushless);
  private final CANSparkMax elevatorRightMotor = new CANSparkMax(CANIds.kElevatorRightCanId, MotorType.kBrushless);

  private final RelativeEncoder elevatorLeftEncoder = elevatorLeftMotor.getEncoder();
  
  private final ProfiledPIDController elevatorPidController = new ProfiledPIDController(
    ElevatorConstants.elevatorP,
    ElevatorConstants.elevatorI,
    ElevatorConstants.elevatorD,
    new Constraints(
      ElevatorConstants.velocityConstraint, 
      ElevatorConstants.accelerationConstraint
    )
  );
  
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorLeftMotor.restoreFactoryDefaults();
    elevatorRightMotor.restoreFactoryDefaults();

    elevatorLeftMotor.setIdleMode(IdleMode.kBrake);
    elevatorRightMotor.setIdleMode(IdleMode.kBrake);

    elevatorLeftMotor.setInverted(false);
    elevatorRightMotor.follow(elevatorLeftMotor, true);

    elevatorLeftMotor.burnFlash();
    elevatorRightMotor.burnFlash();
  }

  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/

  /**
   * Returns the current position goal of the profiled PID.
   * @return The position goal of the elevator PID.
   */
  public double getSetpoint() {
    return elevatorPidController.getGoal().position;
  }

  /**
   * Get the position of the left elevator motor encoder.
   * @return The position of the elevator encoder.
   */
  public double getElevatorEncoderPosition() {
    return elevatorLeftEncoder.getPosition();
  }

  /**
   * Returns a boolean representing if the elevator position is below the bottom threshold
   * @return A boolean represening if the elevator is at the bottom
   */
  public boolean atBottom() {
    return getElevatorEncoderPosition() < ElevatorConstants.elevatorBottomThreshold;
  }

  /**
   * Returns a boolean representing if the elevator position is above the top threshold
   * @return A boolean represening if the elevator is at the top
   */
  public boolean atTop() {
    return getElevatorEncoderPosition() > ElevatorConstants.elevatorTopThreshold;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
