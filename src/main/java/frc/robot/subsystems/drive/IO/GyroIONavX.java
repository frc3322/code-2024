// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IO;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GyroIONavX implements GyroIO{
    
    private AHRS gyro = new AHRS();

    private final boolean reversed;
    
    public GyroIONavX(boolean reversed) {
        this.reversed = reversed;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.yawAngle = reversed ? -gyro.getYaw() : gyro.getYaw();
        inputs.yawRate = reversed ? -gyro.getRate() : gyro.getRate();
        inputs.yawRotation = new Rotation2d(inputs.yawAngle);
    }

    @Override
    public void reset() {
        gyro.reset();
    }   


}
