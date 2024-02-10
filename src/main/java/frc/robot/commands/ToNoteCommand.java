// Credit to team 2996 Coygars Gone Wired

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ToNoteCommand extends Command {
    NetworkTable contoursTable;
    NetworkTableEntry areaEntry;
    NetworkTableEntry centerXEntry;
    int contourIndex;
    private DriveSubsystem m_drivetrainSubsystem;
    private DoubleSupplier m_translationXSupplier;
    private DoubleSupplier m_translationYSupplier;
    private BooleanSupplier m_botRelativeSupplier;



    public ToNoteCommand(DriveSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, BooleanSupplier botRelativeSupplier) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        m_translationXSupplier = () -> translationXSupplier.getAsDouble() * Constants.AutoConstants.kMaxSpeedMetersPerSecond;
        m_translationYSupplier = () -> translationYSupplier.getAsDouble() * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        m_botRelativeSupplier = () -> botRelativeSupplier.getAsBoolean();
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        contoursTable =NetworkTableInstance.getDefault().getTable("GRIP/contours");
        areaEntry = contoursTable.getEntry("area");
        centerXEntry = contoursTable.getEntry("centerX");
    }

    @Override
    public void execute() {
        contourIndex = -1;
        Double[] areaValues = areaEntry.getDoubleArray(new Double[]{});
        Double currentMax = 0.0;
        for(int i = 0; i < areaValues.length; i++) {
            if (areaValues[i]>currentMax) {
                currentMax = areaValues[i];
                contourIndex = i;
                
            }
        }

        String noteString;

        Double[] xValues = centerXEntry.getDoubleArray(new Double[]{});
        Double xValue = 0.0;
        if (xValues.length<1) {
            noteString = "No value found.";
        }else{
            xValue = xValues[contourIndex] - 80;
            noteString = xValue + " ";
            //80=middle
            if (xValue > 0) {
                noteString += "R";
            } else if (xValue == 0) {
                noteString += "C";
            } else{
                noteString += "L";
            }
        }

        SmartDashboard.putString("Note State", noteString);

        double rotSpeed = 0.5 * Math.PI * -xValue / 80.0;
        double xSpeed;
        double ySpeed;
        boolean isRelative;
        if (m_botRelativeSupplier.getAsBoolean()) {

            xSpeed = m_translationXSupplier.getAsDouble();
            ySpeed = m_translationYSupplier.getAsDouble();
            isRelative = true;
            //I think this may have issues with being field-relative. They had a special class that I think dealt with it, but I'm not sure what it did.
        }
        else{ 
            xSpeed = m_translationXSupplier.getAsDouble();
            ySpeed = m_translationYSupplier.getAsDouble();
            isRelative = true;
        }
        m_drivetrainSubsystem.drive(xSpeed, ySpeed, rotSpeed, isRelative, true);
    }
        // m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), rotSpeed, m_drivetrainSubsystem.getRotation()));


    @Override
    public void end(boolean interrupted) {
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}