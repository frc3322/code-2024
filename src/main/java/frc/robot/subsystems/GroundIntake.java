// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.annotations.Log;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */

  //Creates motor objects
  private final CANSparkMax topRoller = new CANSparkMax(Constants.CANIds.kTopRollerCanId, MotorType.kBrushless);
  private final CANSparkMax bottomRoller = new CANSparkMax(Constants.CANIds.kBottomRollerCanId, MotorType.kBrushless);
  private final CANSparkMax intakeArm = new CANSparkMax(Constants.CANIds.kArmCanId, MotorType.kBrushless);

  //Creates encoder for intakeArm
  private final RelativeEncoder armEncoder = intakeArm.getEncoder();
  
  @Log private double armPosition;
  @Log private double armPower;
  @Log private boolean intakeAtTop;

  public GroundIntake() {
    topRoller.setIdleMode(IdleMode.kBrake);
    bottomRoller.setIdleMode(IdleMode.kBrake);
    intakeArm.setIdleMode(IdleMode.kBrake);

    topRoller.burnFlash();
    bottomRoller.burnFlash();
    intakeArm.burnFlash();
  }

  public Boolean atTop() {
    return armEncoder.getPosition() < 1;
  }

  public Boolean atGround() {
    return armEncoder.getPosition() > Constants.GroundIntakeConstants.bottomZoneLimit;
  }

    public void setFlipperSpeed(double speed){
    //moves the entire arm
    intakeArm.set(speed);
  }

  public Boolean atAmpAngle() {
    return armEncoder.getPosition() > Constants.GroundIntakeConstants.ampZoneLimit;
  }

  @Log
  public double calculateIntakeFlipAmp() {
    if (armEncoder.getPosition() > Constants.GroundIntakeConstants.ampZoneLimit) {
      return 0;
    } else if (armEncoder.getPosition() > Constants.GroundIntakeConstants.bottomZoneLimit && armEncoder.getPosition() < Constants.GroundIntakeConstants.ampZoneLimit){
      return Constants.GroundIntakeConstants.armDownSlowSpeed;
    } else {
      return Constants.GroundIntakeConstants.armDownSpeed;
    }   
  }

  @Log
  public double calculateIntakeFlipUp(){
    if (armEncoder.getPosition() < Constants.GroundIntakeConstants.topZoneLimit){
      return 0;
    } else if (armEncoder.getPosition() < Constants.GroundIntakeConstants.slowZoneStart){
      return Constants.GroundIntakeConstants.armUpSlowSpeed;
    } else {
      return Constants.GroundIntakeConstants.armUpSpeed;
    }
  }

  @Log
  public double calculateIntakeFlipGround(){
    if (armEncoder.getPosition() > Constants.GroundIntakeConstants.bottomZoneLimit){
      return 0;  
    }else {
      return Constants.GroundIntakeConstants.armDownSpeed;
    }
  }

  public Command flipUp(){
      return new RunCommand(
        //set flipper to correct setting until it is true that the flipper is at the top.
        () -> setFlipperSpeed(calculateIntakeFlipUp())
      )
      .until(()->atTop());
      
    }

  public Command flipToGround(){
    //flips to bottom, does not spin. may be able to delete
    return new RunCommand(
      () -> setFlipperSpeed(calculateIntakeFlipGround())
    )
    .until(()->atGround());
      
  }

  public Command flipToAmp() {
    return new RunCommand(
      () -> setFlipperSpeed(calculateIntakeFlipAmp()) 
      ).until(() -> atAmpAngle());
  }

  public void spinRollers(int volts) {
    topRoller.setVoltage(volts);
    bottomRoller.setVoltage(volts);
  }

  public void resetArmEncoder() {
    armEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armPosition = armEncoder.getPosition();
    armPower = intakeArm.getAppliedOutput();

  }

}
