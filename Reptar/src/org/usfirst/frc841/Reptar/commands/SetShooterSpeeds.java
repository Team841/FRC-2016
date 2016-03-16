// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc841.Reptar.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc841.Reptar.Robot;

/**
 *
 */
public class SetShooterSpeeds extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double m_UpperSpeed;
    private double m_LowerSpeed;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public SetShooterSpeeds(double UpperSpeed, double LowerSpeed) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_UpperSpeed = UpperSpeed;
        m_LowerSpeed = LowerSpeed;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.shooter);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.uppercloop.updateSetpoint(m_UpperSpeed);
    	Robot.shooter.lowercloop.updateSetpoint(m_LowerSpeed);
    	
    	if(m_UpperSpeed != 0 ){
    		Robot.shooter.uppercloop.enablePID();
    	}
    	else{
    		Robot.shooter.uppercloop.disablePID();;
    		Robot.shooter.setUpperWheelSpeed(0);
    	}
    	if(m_LowerSpeed  != 0){
    		Robot.shooter.lowercloop.enablePID();
    		
    	}
    	else{
    		Robot.shooter.lowercloop.disablePID();
    		Robot.shooter.setLowerWheelSpeed(0);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        
    	return true;
    	/*
    	return Robot.shooter.lowercloop.isDestinationReached()
        		&& Robot.shooter.uppercloop.isDestinationReached()
        		|| (!Robot.shooter.lowercloop.isPIDEnabled()
        				&& !Robot.shooter.uppercloop.isPIDEnabled());
    	*/
    }

    // Called once after isFinished returns true
    protected void end() {
    	if(!Robot.shooter.lowercloop.isPIDEnabled() && !Robot.shooter.uppercloop.isPIDEnabled()){
    		Robot.shooter.setLowerWheelSpeed(0);
    		Robot.shooter.setUpperWheelSpeed(0);
    	}
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
