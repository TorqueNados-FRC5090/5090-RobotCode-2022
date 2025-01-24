package frc.robot.subsystems;

// Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.wrappers.LaserDetector;

public class Elevator {

    // Declare variables
    private LaserDetector topSensor;
    private LaserDetector bottomSensor;
    private SparkMax motor; 
    private boolean bottomSensorHasBall;
    private boolean topSensorHasBall;

    // Constructor 
    public Elevator(int motorID, int bottomSensorID, int topSensorID) {
        // Initialize motors
        motor = new SparkMax(motorID, MotorType.kBrushless);
        // Invert motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize Laser
        topSensor = new LaserDetector(bottomSensorID);
        bottomSensor = new LaserDetector(topSensorID);
    }

    // Accessor methods
    public boolean topSensorHasBall() { return topSensorHasBall; }
    public boolean bottomSensorHasBall() { return bottomSensorHasBall; }

    // Sets the motor to full speed
    public void fullForward() { motor.set(1); }
    // shoot() is an alias for fullForward()
    public void shoot() { fullForward(); }

    // Sets the motor to full speed in reverse
    public void fullBackward() { motor.set(-1); }
    // Ejects a ball at half speed
    public void ejectBall() { motor.set(-0.5); }

    // Turns on the motor 
    public void set(double pwr) { motor.set(pwr); }

    // Turns off the motor
    public void off() { motor.set(0); }

    // Updates the state of the elevator's storage
    public void update() {
        bottomSensorHasBall = bottomSensor.isBlocked();
        topSensorHasBall = topSensor.isBlocked();
    }

    // Automatically runs the elevator based on 
    // the state of its sensors
    public void auto() {
        if(topSensorHasBall)
            off();
        else if(bottomSensorHasBall)
            set(0.4);
        else
            off();
    }

    // Raises the contents of the elevator until
    // the top sensor is triggered
    public void lift() { 
        if(!topSensorHasBall)
            set(0.4);
        else
            off();
    }
}
