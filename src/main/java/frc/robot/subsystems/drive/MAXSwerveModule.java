// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.drive.IO.ModuleIO;
import frc.robot.subsystems.drive.IO.ModuleIOInputsAutoLogged;

public class MAXSwerveModule {

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Logging IO
  private ModuleIO moduleIO;
  
  // Inputs from IO
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public final String name;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(ModuleIO moduleIO, double chassisAngularOffset, String moduleID) {

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(inputs.turnPosition);
    moduleIO.setDrivePosition(0);

    // IO logging
    this.moduleIO = moduleIO;

    name = moduleID;
  }

  public void periodic() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(inputs.driveVelocity,
        new Rotation2d(inputs.turnPosition - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        inputs.drivePosition,
        new Rotation2d(inputs.turnPosition - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
  

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(inputs.turnPosition));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    moduleIO.setDriveMotorSetpoint(optimizedDesiredState.speedMetersPerSecond);
    moduleIO.setTurnMotorSetpoint(optimizedDesiredState.angle.getRadians());

    m_desiredState = desiredState;
  }

  public SwerveModuleState getCurrentDesiredState(){
    return m_desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    moduleIO.setDrivePosition(0);
  }
}
