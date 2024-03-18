// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivot extends SubsystemBase {
  
  private CANSparkMax pivotMotor = new CANSparkMax(CANIds.kShooterPivotCanId, MotorType.kBrushless);

  private RelativeEncoder pivotRelativeEncoder = pivotMotor.getEncoder();

  public final ProfiledPIDController pivotPIDController = new ProfiledPIDController(
    ShooterPivotConstants.shooterPivotP, 
    ShooterPivotConstants.shooterPivotI, 
    ShooterPivotConstants.shooterPivotD, 
    new Constraints(
      ShooterPivotConstants.shooterPivotVelocityConstraint, 
      ShooterPivotConstants.shooterPivotAccelerationConstraint
    )
  );

  private InterpolatingDoubleTreeMap PIVOT_DOUBLE_TREE_MAP = new InterpolatingDoubleTreeMap();
  
  /** Creates a new ShooterPivot. */
  public ShooterPivot() {
    for(int i = 0; i < ShooterPivotConstants.setpointList.length; i++){
      PIVOT_DOUBLE_TREE_MAP.put(
        ShooterPivotConstants.distanceList[i], 
        ShooterPivotConstants.setpointList[i]
      );
    }
  }

  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/

  public double getAngleAtDistance(double distance) {
    return PIVOT_DOUBLE_TREE_MAP.get(distance);
  }

  public double getPivotRelativeEncoderPosition() {
    return pivotRelativeEncoder.getPosition();
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  public void setPivotMotor(double speed) {
    pivotMotor.set(speed);
  }
  
  public void setPivotGoal(double goal) {
    pivotPIDController.setGoal(goal);
  }

  /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

  public Command defaultCommand(Supplier<Pose2d> robotPose, Supplier<Pose2d> speakerPose) {
    return new RunCommand(
      () -> {
        Translation2d robotTranslation = robotPose.get().getTranslation();
        Translation2d speakeTranslation = speakerPose.get().getTranslation();

        double distance = robotTranslation.getDistance(speakeTranslation);

        double setpoint = getAngleAtDistance(distance);

        setPivotGoal(setpoint);
        setPivotMotor(pivotPIDController.calculate(getPivotRelativeEncoderPosition()));
      },
      this
    );
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
