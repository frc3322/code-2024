// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
//import io.github.oblarg.oblog.Loggable;
//import io.github.oblarg.oblog.annotations.Log;

public class Elevator extends SubsystemBase{
  private final CANSparkMax elevatorLeft = new CANSparkMax(CANIds.kElevatorLeftId, MotorType.kBrushless);
  private final CANSparkMax elevatorRight = new CANSparkMax(CANIds.kElevatorRightId, MotorType.kBrushless);
  private final DigitalInput bottomLimit = new DigitalInput(1);
  private final RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
  private final RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder();
  
  public double elevatorLeftEncoderVal;
  public double elevatorRightEncoderVal;

  private boolean slowModeOn = false;

  
  private int currentSetpointIndex = 0;

  private double setpoint = 0;
  
  public Elevator() {

   

    //not sure which we need to invert. Make up positive though.
    // left cw up

    elevatorRight.restoreFactoryDefaults();
    elevatorLeft.restoreFactoryDefaults();
     
    elevatorLeft.setInverted(false);
     elevatorRight.follow(elevatorLeft, true);
    
     
     elevatorLeft.setIdleMode(IdleMode.kBrake);
     elevatorRight.setIdleMode(IdleMode.kBrake);

     elevatorRight.burnFlash();
     elevatorLeft.burnFlash();

  }

  /*◇─◇──◇─◇
     Getters
  ◇─◇──◇─◇*/

  public double getElevatorLeftEncoder(){
    return elevatorLeftEncoderVal;
  }

  
  public boolean atBottom(){
    return bottomLimit.get();
  }

  
  public double getCurrentCycleSetpoint() {
    return ElevatorConstants.elevatorSetpoints[currentSetpointIndex];
  }

  public double getSetpoint(){
    return setpoint;
  }

  // public double inchesToEncoder(double inches){
  //   
  // }
  
  /*◇─◇──◇─◇
     Setters
  ◇─◇──◇─◇*/

  public Command exampleCommand(){
    return new InstantCommand(() -> {});
  }

  public void setMotor(double left){
    elevatorLeft.set(left);
  }

  public void setpointCycleUp(){
    if (currentSetpointIndex < ElevatorConstants.elevatorSetpoints.length - 1) currentSetpointIndex++;
    setpoint = getCurrentCycleSetpoint();
  }

  public void setpointCycleDown(){
    
    if (currentSetpointIndex > 0) currentSetpointIndex--;
    setpoint = getCurrentCycleSetpoint();
  }

  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
  }

  public void setSetpointCycleIndex(int index){
    currentSetpointIndex = index;
    setpoint = getCurrentCycleSetpoint();
  }

  public void resetElevatorEncoders(){
    elevatorLeftEncoder.setPosition(0);
    elevatorRightEncoder.setPosition(0);
  }

  public void toggleElevatorSlowMode(){
    if(slowModeOn){
      slowModeOn = false;
    }else{
      slowModeOn = true;
    }
  }

  /*◇─◇──◇─◇
    Commands
  ◇─◇──◇─◇*/
  
  public Command setpointCycleUpCommand() {
    return new InstantCommand( () -> setpointCycleUp(), this);
  }

  public Command setpointCycleDownCommand() {
    return new InstantCommand( () -> setpointCycleDown(), this);
  }

  
  public Command resetElevatorEncodersCommand(){
    return new InstantCommand( () -> resetElevatorEncoders());

  }

  public Command stopElevator() {
    return new InstantCommand( () -> elevatorLeft.set(0), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorLeftEncoderVal = elevatorLeftEncoder.getPosition();
    elevatorRightEncoderVal = elevatorRightEncoder.getPosition();

    if(atBottom()){
     resetElevatorEncoders();
    }

  }

  
}
