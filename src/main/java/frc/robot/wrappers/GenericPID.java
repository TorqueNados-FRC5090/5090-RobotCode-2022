package frc.robot.wrappers;

// Imports
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class GenericPID {
    // Declare variables
    private SparkMaxPIDController controller;
    private CANSparkMax motor;
    private double P; // Proportional gain
    private double I; // Integral gain
    private double D; // Derivative gain
    private double setpoint;
    private CANSparkMax.ControlType controlType;
    private double min = Integer.MIN_VALUE;
    private double max = Integer.MAX_VALUE;

    // Constructor with only P
    public GenericPID(CANSparkMax motor, CANSparkMax.ControlType controlType, double P) {
        this.motor = motor;
        controller = motor.getPIDController();

        this.controlType = controlType;

        this.P = P;
        controller.setP(P);
    }

    // Constructor with P, I, and D
    public GenericPID(CANSparkMax motor, CANSparkMax.ControlType controlType, double P, double I, double D) {
        this.motor = motor;
        controller = motor.getPIDController();

        this.controlType = controlType;

        this.P = P;
        controller.setP(P);

        this.I = I;
        controller.setI(I);

        this.D = D;
        controller.setD(D);
    }

    // Accessor methods
    public double getP() { return controller.getP(); }
    public double getI() { return controller.getI(); }
    public double getD() { return controller.getD(); }
    public double getSetpoint() { return setpoint; }
    public CANSparkMax.ControlType getControlType() { return controlType; }
    public double getRPM() { return motor.getEncoder().getVelocity(); }
    public double getPosition() { return motor.getEncoder().getPosition(); }
    public double getMin() { return min; }
    public double getMax() { return max; }
    public SparkMaxPIDController getController() { return controller; }
    public CANSparkMax getMotor() { return motor; }

    // Setter Methods
    public void setP(double P) { this.P = P; controller.setP(P); }
    public void setI(double I) { this.I = I; controller.setI(I); }
    public void setD(double D) { this.D = D; controller.setD(D); }
    public void setControlType(CANSparkMax.ControlType controlType) { this.controlType = controlType; }
    public void setMax(double min) { this.min = min; }
    public void setMin(double max) { this.max = max; }
    public void setDomain(double min, double max) { this.min = min; this.max = max; }
    
    // Forces the setpoint in bounds when it is set
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint > max ? max : setpoint;
        this.setpoint = setpoint > min ? min : setpoint;
    }

    // Make sure that the PID gains match the object settings
    public void updatePID() { 
        if( this.P != controller.getP())
            controller.setP(this.P);

        if( this.I != controller.getI())
            controller.setI(this.I);

        if( this.D != controller.getD())
            controller.setD(this.D);
    }

    // Starts the PID controller
    public void activate() {
        controller.setReference(setpoint, controlType);
    }

    // Sets the PID gains to 0, without changing the stored values
    public void pause(){
        controller.setP(0);
        controller.setI(0);
        controller.setD(0);
    }

    // Sets the PID gains to 0, as well as the stored values
    public void stop() {
        this.setP(0);
        this.setI(0);
        this.setD(0);
    }
}
