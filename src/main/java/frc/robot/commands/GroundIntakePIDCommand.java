// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GroundIntake;
import io.github.oblarg.oblog.Loggable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class GroundIntakePIDCommand extends PIDCommand implements Loggable {

  private GroundIntake groundIntake;
  /** Creates a new GroundIntakePIDCommand. */
  public GroundIntakePIDCommand(double targetAngle, GroundIntake groundIntake) {
    super(
        // The controller that the command will use
        new PIDController(Constants.GroundIntakeConstants.intakeP, Constants.GroundIntakeConstants.intakeI, Constants.GroundIntakeConstants.intakeD),
        // This should return the measurement
        groundIntake::getArmEncoderVal,
        // This should return the setpoint (can also be a constant)
        // NOTE: targetAngle is in degrees, while the getArmEncoderVal is in ticks or rotations. Test to find the conversion between degrees to rotations.
        targetAngle,
        // This uses the output
        output -> groundIntake.setFlipperSpeed(output),
        //Require GroundIntake Subsystem
        groundIntake);

    // Configure additional PID options by calling `getController` here.
    this.groundIntake = groundIntake;
    
    // Puts the Arm Angle to SmartDashboard
    SmartDashboard.putData("Arm Angle Controller", getController());

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    
      // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
      // setpoint before it is considered as having reached the reference
      getController()
        .setTolerance(Constants.GroundIntakeConstants.kTurnToleranceDeg, Constants.GroundIntakeConstants.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }

  
  @Override
  public void initialize() {
    // Resets the Arm Encoder when PIDCommand is created
    groundIntake.resetArmEncoder();
  }
}
