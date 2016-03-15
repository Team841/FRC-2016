// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc841.Reptar.subsystems;

import org.usfirst.frc841.Reptar.RobotMap;
import org.usfirst.frc841.Reptar.commands.*;
import org.usfirst.frc841.lib.PID.PIDControlLoop;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GearTooth;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *This is subsystem class for the shooter.
 */
public class Shooter extends Subsystem {
	
	private double SHOOTERWHEELDIAMETER = 3;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final DigitalInput ballPresentSensor = RobotMap.shooterBallPresentSensor;
    private final DoubleSolenoid hopper = RobotMap.shooterHopper;
    private final GearTooth upperWheelSpeedSensor = RobotMap.shooterUpperWheelSpeedSensor;
    private final Solenoid shooterActuator = RobotMap.shooterShooterActuator;
    private final GearTooth lowerWheelSpeedSensor = RobotMap.shooterLowerWheelSpeedSensor;
    private final SpeedController lowerWheelDrive = RobotMap.shooterLowerWheelDrive;
    private final SpeedController upWheelDrive = RobotMap.shooterUpWheelDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public Shooter.UpperCLoop uppercloop;
    public Shooter.LowerCLoop lowercloop;
    double x[]= {1,2,3};
    double y[] = {0,0,0};
    private double period = .02; //Seconds
    
    
    /**
     * Constructor of the Shooter Subsystem class.
     */
    public Shooter(){
    	initSensors();
    	uppercloop = new Shooter.UpperCLoop(this, x, y, (long) Math.abs( period * 100));
    	lowercloop = new Shooter.LowerCLoop(this, x, y, (long) Math.abs( period * 100));
    	
		uppercloop.setTunings(1000, 0, 0);
		uppercloop.SetOutputLimits(-1, 0);
		uppercloop.updateSetpoint(65);
		lowercloop.setTunings(1000, 0, 0);
		lowercloop.SetOutputLimits(-1, 0);
		lowercloop.updateSetpoint(16.25);
    	
    }
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    
    /**
     * This set hooper low
     */
    public void setHopperLow(){
    	hopper.set(DoubleSolenoid.Value.kForward);
    }
    /**
     * This set hooper high
     */
    public void setHopperHigh(){
    	hopper.set(DoubleSolenoid.Value.kReverse);
    }
    /**
     * This set shooter to shooting position so we can shoot
     */
    public void setToShooterPosition(){
    	shooterActuator.set(true);
    }
    /**
     * This sets the shooter to retract so we can drive under low bar
     */
    public void setToRetractedPosition(){
    	shooterActuator.set(false);
    }
    /**
     * Gives us speed of the upper wheel in feet/second
     * @return Returns current speed
     */
    public double getUpperWheelSpeed(){
    	return upperWheelSpeedSensor.getRate()*Math.PI*SHOOTERWHEELDIAMETER/12;
    }
    /**
     * Gives us speed of the lower wheel in feet/second
     * @return Returns current speed
     */
    public double getLowerWheelSpeed(){
    	return lowerWheelSpeedSensor.getRate()*Math.PI*SHOOTERWHEELDIAMETER/12;
    }
    /**
     * This sets the speed of the upper wheel shooter
     *
     * @param speed
     */
    public void setUpperWheelSpeed(double speed){
    	upWheelDrive.set(speed);
    }
    /**
     * This sets the speed for the lower wheel shooter
     * @param speed
     */
    public void setLowerWheelSpeed(double speed){
    	lowerWheelDrive.set(speed);
    }
    /**
     * This initializes the hall effect sensors
     */
    public void initSensors(){
    	upperWheelSpeedSensor.reset();
    	upperWheelSpeedSensor.setDistancePerPulse(1);
    	lowerWheelSpeedSensor.reset();
    	lowerWheelSpeedSensor.setDistancePerPulse(1);
    	
    	
    }
    
    /**
     * This Class links the Upper Wheel Control loop to the subsystem
     *
     */
    public class UpperCLoop extends PIDControlLoop{
    	Shooter subsystem;
		public UpperCLoop(Shooter subsystem,double[] X, double[] Y, long SampleTime) {
			super(X, Y, SampleTime);
			this.subsystem = subsystem;
			// TODO Auto-generated constructor stub
		}
		@Override
		public void setOutput(double value) {
			subsystem.setUpperWheelSpeed(value);
			//System.out.println("Output: " + value);
		}
		@Override
		public double getSensorReading(){
			//System.out.println("in: " + subsystem.getUpperWheelSpeed());
			return subsystem.getUpperWheelSpeed();
		}    	
    }
    /**
     * This Class links the Lower Wheel Control Loop to the subsystem
     *
     */
    public class LowerCLoop extends PIDControlLoop{
    	Shooter subsystem;
		public LowerCLoop(Shooter subsystem,double[] X, double[] Y, long SampleTime) {
			super(X, Y, SampleTime);
			this.subsystem = subsystem;
			// TODO Auto-generated constructor stub
		}
		@Override
		public void setOutput(double value) {
			subsystem.setLowerWheelSpeed(value);
		}
		@Override
		public double getSensorReading(){
			return subsystem.getLowerWheelSpeed();
		}
		@Override
		public void update(){
			SmartDashboard.putString("DB/String 3","Upper: " + Math.floor(this.subsystem.getUpperWheelSpeed()*100/100.0)
			+ " lower: " + Math.floor(this.subsystem.getLowerWheelSpeed()*100/100.0) );
		}
    }
  
}
