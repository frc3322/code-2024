// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWMIds;

public class Forks extends SubsystemBase {
  /** Creates a new Forks. */

  private final Servo leftServo = new Servo(PWMIds.leftServoID);
  private final Servo rightServo = new Servo(PWMIds.rightServoID);
  
  public Forks() {}


  /**
   * Sets the spin speeds of both servos so left and right move in opposite directions.
   * @param speed The speed of the servo, on a scale of 0-1, with 0 being full reverse, 1 being full forward, and 0.5 being full stop.
   */
  public void setSpinSpeeds(double speed){
    leftServo.set(speed);
    rightServo.set(1-speed);
    
  }
  public Command spinServosCommand(){
    return new InstantCommand(
      ()-> setSpinSpeeds(1),
      this
      );
  }
  public Command stopServosCommand(){
    return new InstantCommand(
      ()-> setSpinSpeeds(.5),
      this
    );
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
