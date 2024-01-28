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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIOids;
import frc.robot.Constants.TransferConstants;

public class Transfer extends SubsystemBase {
  
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

    transferMotor.burnFlash();
    shooterTransferMotor.burnFlash();
  }

  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/

  public boolean transferFull() {
    return transferBeamBreak.get();
  }

  public boolean shooterFull() {
    return shooterBeamBreak.get();
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  public void setTransferSpeeds(double power){
    transferMotor.set(power);
  }

  public void setShooterTransferSpeeds(double power) {
    shooterTransferMotor.set(power);
  }

  public void stopTransfer(){
    transferMotor.stopMotor();
  }

  public void stopShooterTransfer(){
    shooterTransferMotor.stopMotor();
  }

  /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

  public Command intakeToShooter() {
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

  public Command shooterToIntake(BooleanSupplier intakeFull) {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
