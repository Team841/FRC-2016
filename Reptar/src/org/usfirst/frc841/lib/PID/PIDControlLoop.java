package org.usfirst.frc841.lib.PID;

import java.util.Timer;
import java.util.TimerTask;

public class PIDControlLoop {

	//PID variables  
	private double Output =0; 
	public double Setpoint =0 ;
	private double ITerm, lastInput = 0;
	private double kp, ki, kd = 0;
	private double SampleTime = 1000;
	private double outMin, outMax =0;

	//PID output reversal
	static private boolean DIRECT = false;
	static private boolean REVERSE = false;
	 
	private boolean controllerDirection = DIRECT;
	
	//PID implementations variables
	public boolean isDestinationReached = false;
	public boolean enablePID = false;
	
	
	//feed-forward variables; 
	private int ffpointer = 0;
	private double ffTableY[]; 
	private double ffTableX[];
	private double slope[];
	private double b[];
	 
	private Timer timer;

	public PIDControlLoop(double X[] , double Y[], long SampleTime){
		//initialize the tables
		ffTableY = new double[Y.clone().length];
		ffTableX = new double[X.clone().length];
		ffTableY = Y.clone();
		ffTableX = X.clone();
		slope = new double[ffTableY.length];
		b = new double[ffTableY.length];
	  
	     //Calculate Y = mX + b equations
		for(int i=0;i<(ffTableY.length-1);i++){
			slope[i] = (ffTableY[i] - ffTableY[(i+1)]) / 
			(ffTableX[i] - ffTableX[(i+1)]);
			b[i] = ffTableY[i] - slope[i]*ffTableX[i];
	    }
		
		//SetsSampleTime
		this.SampleTime = SampleTime;

		timer = new Timer();
    	timer.schedule(new Update(this),0L, SampleTime);
		
	}
	  class Update extends TimerTask{
	    	private PIDControlLoop c;
	    	
	    	
	    	public Update(PIDControlLoop pidLoop){
	    		this.c = pidLoop;
	    	}
	    	@Override
	    	public void run(){
	    		//This runs the PID loop.
	    		c.update();
	    		//Run PID loop when enabled
	    		if(enablePID){
	    			//This reads the sensor input, Computes the PID value and sets the output 
	    			c.SetOutput(c.Compute(c.getSensorReading()));
	    		
	    		}
	    	}
	    }
	   
	
	//returns current ff value. 
	public double getFFValue(double input){
		//find current ff data;
		double data =Math.ceil(input);
	    
	      //find current ff data;
		for (int i=0; i < (slope.length-1);i++){ 
			if(data > ffTableX[i]){
				ffpointer = i;
			}
		}
	   return (slope[ffpointer]*input+b[ffpointer]);
	 }
	
	// Sets positive PID values 
	public void setTunings(double Kp, double Ki, double Kd){
		if(Kp > 0 || Ki > 0 || Kd > 0 ){
			double SampleTimeInSec = SampleTime/1000;
	   
			kp = Kp;
			ki = Ki * SampleTimeInSec;
			kd = Kd / SampleTimeInSec;
			if(controllerDirection == REVERSE){
				kp = (0 - kp);
				ki = (0 - ki);
				kd = (0 - kd);
	     
			}
		}
	}
	
	// Sets Sample Time in milliseconds
	public void setSampleTime( double NewSampleTime){
		if (NewSampleTime > 0){
			double ratio = NewSampleTime / SampleTime;
	   
			ki *=ratio;
			kd /=ratio;
	   
			SampleTime = NewSampleTime;
			//System.out.println(ki);
		}
	}
	
	// Sets Limit to Output
	public void SetOutputLimits( double Min, double Max){
		if(Min < Max){
			outMin = Min;
			outMax = Max;
	   
			if(Output > outMax){
				Output = outMax;
			}
			else if(Output < outMin){
				Output = outMin;
			}
			if (ITerm > outMax){
				ITerm = outMax;
			}
			else if(ITerm < outMin){
				ITerm = outMin;
			}
		}
	}
/*
	//Clears up all data from past PID loop
	public void initialize(double input){
		lastInput = input;
		ITerm = Output;
		if (ITerm > outMax){
			ITerm = outMax;
		}
		else if(ITerm < outMin){
			ITerm = outMin;
		}
	 }
	 */
	//Clears up all data from past PID loop
		public void initialize(){
			lastInput = 0;
			ITerm = 0;
			Output =0;
			
		 }
	
	//Set the PID output polarity
	public void SetControllerDirection (boolean Direction){
		controllerDirection = Direction;
	}
	 
	//Computes the PID values to be updated
	 public double Compute(double input){
	    
	    double error = Setpoint - input;
	    ITerm += (ki * error);
	    if( ITerm > outMax ){
	     ITerm = outMax;
	    }
	    else if( ITerm < outMin ){
	     ITerm = outMin;
	    }
	    double dInput = (input - lastInput);
	    
	    //Compute PID Output
	    Output = kp * error + ITerm - kd * dInput + this.getFFValue(Setpoint);
	    
	    if( Output > outMax ){
	     Output = outMax;
	    }
	    else if( Output < outMin ){
	     Output = outMin;
	    }
	    
	    //Remember some variables for next time
	    lastInput = input;
	  
	    
	    //Return calculated values.
	    return Output;
	   }

	 
	 // Update Setpoint
	 public void SetReference(double ref){
	  Setpoint =  ref;
	 }

	//This output method needs to be defined be the subclass for it to do anything
	public void SetOutput(double value){
		System.out.println("Need to override the SetOutput Methode of PIDLoop to Work");
		
	}
	//This input method needs to be defined be the subclass for it to do anything
	public double getSensorReading(){
		System.out.println("Need to override tje getSensorReading of PIDLoop to Work");
		return 0;
	}
	//This method is meant to be overriden
	public void update(){
		
	}

}