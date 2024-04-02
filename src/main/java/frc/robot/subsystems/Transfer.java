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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.commands.WaitUntilConditionCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The transfer subsystem for 3322's 2024 robot.
 */
public class Transfer extends SubsystemBase implements Loggable {
  
  private final CANSparkMax transferMotor = new CANSparkMax(CANIds.kTransferCanId, MotorType.kBrushless);
  private final CANSparkMax shooterTransferMotor = new CANSparkMax(CANIds.kShooterTransferCanId, MotorType.kBrushless);

  private final DigitalInput shooterBeamBreak = new DigitalInput(5);
  
  /** Creates a new Transfer. */
  public Transfer() {
    transferMotor.restoreFactoryDefaults();
    shooterTransferMotor.restoreFactoryDefaults();

    transferMotor.setIdleMode(IdleMode.kCoast);
    shooterTransferMotor.setIdleMode(IdleMode.kBrake);

    transferMotor.setInverted(true);
    shooterTransferMotor.setInverted(true);

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
   * Returns whether the shooter beam break is tripped.
   * @return A boolean representing if the shooter has a game piece.
   */
  @Log
  public boolean shooterFull() {
    return !shooterBeamBreak.get();
  }

  @Log
  public boolean shooterNotFull() {
    return shooterBeamBreak.get();
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  /**
   * Sets the speed of the transfer.
   * @param power The speed of the transfer.
   */
  @Config
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
   * Runs the transfer in a direction dependent on the paramater
   * @param in True if towards shooter, false if away from shooter
   * @return A command to run the transfer
   */
  public Command runTransferCommand(boolean in){
    return new StartEndCommand(
      ()-> {
        double speed = in ? TransferConstants.transferSpeed : -TransferConstants.transferSpeed;
        setTransferSpeeds(speed);
        setShooterTransferSpeeds(speed);
        },
      ()-> {
        stopTransfer();
        stopShooterTransfer();
      },
      this
    );
  }

   

  /**
   * Returns a command that moves a game piece from the intake to the shooter.
   * @return A start end command.
   */
  public Command intakeToShooterCommand() {
    Command command = runTransferCommand(true).until(this::shooterFull);
    command.addRequirements(this);
    return command;
  }

  //   public Command intakeToShooterWithDelayCommand() {
  //   Command command = new SequentialCommandGroup(
  //     new InstantCommand(()-> {
  //       setTransferSpeeds(TransferConstants.transferSpeed);
  //       setShooterTransferSpeeds(TransferConstants.transferSpeed);
  //     }),
  //     new WaitUntilConditionCommand(this::shooterFull),
  //     new WaitCommand(.05),
  //     new InstantCommand(()-> {
  //       setTransferSpeeds(0);
  //       setShooterTransferSpeeds(0);
  //     })

  //   );
  //   command.addRequirements(this);
  //   return command;
  // }

  /**
   * Returns a command that moves the game piece from the shooter to the intake.
   * @param intakeFull A boolean supplier representing if the intake is full or not.
   * @return A start end command.
   */
  public Command shooterToIntakeCommand(BooleanSupplier intakeFull) {
    return runTransferCommand(false).until(intakeFull);
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
   
  }
}
