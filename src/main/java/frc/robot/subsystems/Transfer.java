// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIOids;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.TransferConstants;
import io.github.oblarg.oblog.Loggable;

/**
 * The transfer subsystem for 3322's 2024 robot.
 */
public class Transfer extends SubsystemBase implements Loggable {
  
  private final CANSparkMax transferMotor = new CANSparkMax(CANIds.kTransferCanId, MotorType.kBrushless);
  private final CANSparkMax shooterTransferMotor = new CANSparkMax(CANIds.kShooterTransferCanId, MotorType.kBrushless);

  private final DigitalInput transferBeamBreak = new DigitalInput(DIOids.transferBeamBreakDIO);
  private final DigitalInput shooterBeamBreak = new DigitalInput(DIOids.shooterBeamBreakDIO);
  
  /** Creates a new Transfer. */
  public Transfer() {
    transferMotor.restoreFactoryDefaults();
    shooterTransferMotor.restoreFactoryDefaults();

    transferMotor.setIdleMode(IdleMode.kBrake);
    shooterTransferMotor.setIdleMode(IdleMode.kBrake);

    /* Current limit so that we dont fry our 550s
    - https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing/
    - https://www.chiefdelphi.com/t/clear-concise-best-practices-for-sparkmax-neo-current-limiting/405541/5  
    */
    transferMotor.setSmartCurrentLimit(NeoMotorConstants.neo550CurrentLimitAmps);
    shooterTransferMotor.setSmartCurrentLimit(NeoMotorConstants.neo550CurrentLimitAmps);

    transferMotor.burnFlash();
    shooterTransferMotor.burnFlash();
  }

  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/

  /**
   * Returns whether the transfer beam break is tripped.
   * @return A boolean representing if the transfer has a game piece.
   */
  public boolean transferFull() {
    return transferBeamBreak.get();
  }

  /**
   * Returns whether the shooter beam break is tripped.
   * @return A boolean representing if the shooter has a game piece.
   */
  public boolean shooterFull() {
    return shooterBeamBreak.get();
  }

  public boolean shooterNotFull() {
    return !shooterBeamBreak.get();
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  /**
   * Sets the speed of the transfer.
   * @param power The speed of the transfer.
   */
  public void setTransferSpeeds(double power){
    transferMotor.set(power);
  }

  /**
   * Sets the speed of the shooter transfer.
   * @param power The speed of the shooter transfer.
   */
  public void setShooterTransferSpeeds(double power) {
    shooterTransferMotor.set(power);
  }

  /**
   * Stops the transfer motor.
   */
  public void stopTransfer(){
    transferMotor.stopMotor();
  }

  /**
   * Stops the shooter tranfer motor.
   */
  public void stopShooterTransfer(){
    shooterTransferMotor.stopMotor();
  }

  /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

  /**
   * Returns a command that moves a game piece from the intake to the shooter.
   * @return A start end command.
   */
  public Command intakeToShooterCommand() {
    return new StartEndCommand(
      () -> {
        setTransferSpeeds(TransferConstants.transferSpeed);
        setShooterTransferSpeeds(TransferConstants.transferSpeed);
      }, 
      () -> {
        stopTransfer();
        stopShooterTransfer();
      }, 
      this
    )
    .until(this::shooterFull);
  }

  /**
   * Returns a command that moves the game piece from the shooter to the intake.
   * @param intakeFull A boolean supplier representing if the intake is full or not.
   * @return A start end command.
   */
  public Command shooterToIntakeCommand(BooleanSupplier intakeFull) {
    return new StartEndCommand(
      () -> {
        setTransferSpeeds(-TransferConstants.transferSpeed);
        setShooterTransferSpeeds(-TransferConstants.transferSpeed);
      }, 
      () -> {
        stopTransfer();
        stopShooterTransfer();
      }, 
      this
    ).until(intakeFull);
  }

  /**
   * Returns a command that feeds the game piece into the shooter wheels.
   * @return A RunCommand.
   */
  public Command shootCommand() {
    return new RunCommand(
      () -> {
        setShooterTransferSpeeds(TransferConstants.transferSpeed);
      }, 
      this
    )
    .until(this::shooterNotFull)
    .andThen(new WaitCommand(TransferConstants.shootWaitTime))
    .andThen(new InstantCommand(
      () -> {
        stopShooterTransfer();
      },
      this
    ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
