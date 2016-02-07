// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc841.Reptar;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController drivetrainLeftDrive1;
    public static SpeedController drivetrainLeftDrive2;
    public static SpeedController drivetrainRightDrive1;
    public static SpeedController drivetrainRightDrive2;
    public static Encoder drivetrainleftQuad;
    public static Encoder drivetrainrightQuad;
    public static DoubleSolenoid drivetrainShifter;
    public static Compressor drivetrainCompressor1;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drivetrainLeftDrive1 = new VictorSP(10);
        LiveWindow.addActuator("Drivetrain", "LeftDrive1", (VictorSP) drivetrainLeftDrive1);
        
        drivetrainLeftDrive2 = new VictorSP(11);
        LiveWindow.addActuator("Drivetrain", "LeftDrive2", (VictorSP) drivetrainLeftDrive2);
        
        drivetrainRightDrive1 = new VictorSP(8);
        LiveWindow.addActuator("Drivetrain", "RightDrive1", (VictorSP) drivetrainRightDrive1);
        
        drivetrainRightDrive2 = new VictorSP(9);
        LiveWindow.addActuator("Drivetrain", "RightDrive2", (VictorSP) drivetrainRightDrive2);
        
        drivetrainleftQuad = new Encoder(3, 4, false, EncodingType.k4X);
        LiveWindow.addSensor("Drivetrain", "leftQuad", drivetrainleftQuad);
        drivetrainleftQuad.setDistancePerPulse(1.0);
        drivetrainleftQuad.setPIDSourceType(PIDSourceType.kRate);
        drivetrainrightQuad = new Encoder(6, 7, false, EncodingType.k4X);
        LiveWindow.addSensor("Drivetrain", "rightQuad", drivetrainrightQuad);
        drivetrainrightQuad.setDistancePerPulse(1.0);
        drivetrainrightQuad.setPIDSourceType(PIDSourceType.kRate);
        drivetrainShifter = new DoubleSolenoid(0, 2, 5);
        LiveWindow.addActuator("Drivetrain", "Shifter", drivetrainShifter);
        
        drivetrainCompressor1 = new Compressor(0);
        
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
