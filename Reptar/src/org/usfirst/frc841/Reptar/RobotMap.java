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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GearTooth;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
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
    public static SpeedController drivetrainRightDrive2;
    public static SpeedController drivetrainRightDrive1;
    public static Encoder drivetrainleftQuad;
    public static Encoder drivetrainrightQuad;
    public static Solenoid drivetrainClimbingSolenoid;
    public static Solenoid drivetrainShiftingSolenoid;
    public static Compressor drivetrainCompressor1;
    public static DigitalInput shooterBallPresentSensor;
    public static DoubleSolenoid shooterHopper;
    public static GearTooth shooterUpperWheelSpeedSensor;
    public static Solenoid shooterShooterActuator;
    public static GearTooth shooterLowerWheelSpeedSensor;
    public static SpeedController shooterLowerWheelDrive;
    public static SpeedController shooterUpWheelDrive;
    public static DigitalInput intakeElbowZeroSensor;
    public static Encoder intakeElbowQuadEncoder;
    public static SpeedController intakeElbowMotor;
    public static SpeedController intakeRollerMotor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drivetrainLeftDrive1 = new VictorSP(0);
        LiveWindow.addActuator("Drivetrain", "LeftDrive1", (VictorSP) drivetrainLeftDrive1);
        
        drivetrainLeftDrive2 = new VictorSP(1);
        LiveWindow.addActuator("Drivetrain", "LeftDrive2", (VictorSP) drivetrainLeftDrive2);
        
        drivetrainRightDrive2 = new VictorSP(3);
        LiveWindow.addActuator("Drivetrain", "RightDrive2", (VictorSP) drivetrainRightDrive2);
        
        drivetrainRightDrive1 = new VictorSP(2);
        LiveWindow.addActuator("Drivetrain", "RightDrive1", (VictorSP) drivetrainRightDrive1);
        
        drivetrainleftQuad = new Encoder(2, 3, false, EncodingType.k4X);
        LiveWindow.addSensor("Drivetrain", "leftQuad", drivetrainleftQuad);
        drivetrainleftQuad.setDistancePerPulse(1.0);
        drivetrainleftQuad.setPIDSourceType(PIDSourceType.kRate);
        drivetrainrightQuad = new Encoder(0, 1, false, EncodingType.k4X);
        LiveWindow.addSensor("Drivetrain", "rightQuad", drivetrainrightQuad);
        drivetrainrightQuad.setDistancePerPulse(1.0);
        drivetrainrightQuad.setPIDSourceType(PIDSourceType.kRate);
        drivetrainClimbingSolenoid = new Solenoid(0, 0);
        LiveWindow.addActuator("Drivetrain", "ClimbingSolenoid", drivetrainClimbingSolenoid);
        
        drivetrainShiftingSolenoid = new Solenoid(0, 1);
        LiveWindow.addActuator("Drivetrain", "ShiftingSolenoid", drivetrainShiftingSolenoid);
        
        drivetrainCompressor1 = new Compressor(0);
        
        
        shooterBallPresentSensor = new DigitalInput(6);
        LiveWindow.addSensor("Shooter", "BallPresentSensor", shooterBallPresentSensor);
        
        shooterHopper = new DoubleSolenoid(0, 7, 5);
        LiveWindow.addActuator("Shooter", "Hopper", shooterHopper);
        
        shooterUpperWheelSpeedSensor = new GearTooth(4, false);
        LiveWindow.addSensor("Shooter", "UpperWheelSpeedSensor", shooterUpperWheelSpeedSensor);
        
        shooterShooterActuator = new Solenoid(0, 3);
        LiveWindow.addActuator("Shooter", "ShooterActuator", shooterShooterActuator);
        
        shooterLowerWheelSpeedSensor = new GearTooth(5, false);
        LiveWindow.addSensor("Shooter", "LowerWheelSpeedSensor", shooterLowerWheelSpeedSensor);
        
        shooterLowerWheelDrive = new VictorSP(6);
        LiveWindow.addActuator("Shooter", "LowerWheelDrive", (VictorSP) shooterLowerWheelDrive);
        
        shooterUpWheelDrive = new VictorSP(4);
        LiveWindow.addActuator("Shooter", "UpWheelDrive", (VictorSP) shooterUpWheelDrive);
        
        intakeElbowZeroSensor = new DigitalInput(9);
        LiveWindow.addSensor("Intake", "ElbowZeroSensor", intakeElbowZeroSensor);
        
        intakeElbowQuadEncoder = new Encoder(7, 8, false, EncodingType.k4X);
        LiveWindow.addSensor("Intake", "ElbowQuadEncoder", intakeElbowQuadEncoder);
        intakeElbowQuadEncoder.setDistancePerPulse(1.0);
        intakeElbowQuadEncoder.setPIDSourceType(PIDSourceType.kRate);
        intakeElbowMotor = new VictorSP(7);
        LiveWindow.addActuator("Intake", "ElbowMotor", (VictorSP) intakeElbowMotor);
        
        intakeRollerMotor = new VictorSP(5);
        LiveWindow.addActuator("Intake", "RollerMotor", (VictorSP) intakeRollerMotor);
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
