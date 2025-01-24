package frc.robot.wrappers;

// Imports
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;

/** Wraps over {@link SparkPIDController} for ease of use */
public class GenericPID {
    /** The motor being controlled */
    private SparkBase motor;
    /** The motor's PID controller */
    private SparkClosedLoopController controller;
    /** A config object to store PID values */
    SparkMaxConfig config = new SparkMaxConfig();

    /** Proportional gain */
    private double P; 
    /** Integral gain */
    private double I; 
    /** Derivative gain */
    private double D;

    /** The PID controller's target */
    private double setpoint = 0;
    /** {@link CANSparkMax.ControlType How} the motor should be controlled */
    private SparkBase.ControlType controlType;

    /** The minimum setpoint to be allowed */
    private double min = Integer.MIN_VALUE;
    /** The maximum setpoint to be allowed */
    private double max = Integer.MAX_VALUE;

    // NOTE: ratio is typically used by position controllers to covert
    // from revolutions to a more useful unit.
    /** Incoming setpoint related values will be multiplied by this.
      * Outgoing setpoint related values will be divided by this. */
    private double ratio = 1;

    /** Constructs a GenericPID object 
     *  @param motor The {@link CANSparkMax motor} to control 
     *  @param controlType {@link CANSparkMax.ControlType How} the motor should be controlled
     *  @param P The P value to be used by the controller */
    public GenericPID(SparkBase motor, SparkBase.ControlType controlType, double P) {
        this(motor, controlType, P, 0, 0);
    }

    /** Constructs a GenericPID object 
     *  @param motor The {@link CANSparkMax motor} to control 
     *  @param controlType {@link CANSparkMax.ControlType How} the motor should be controlled
     *  @param P The P value to be used by the controller 
     *  @param I The I value to be used by the controller 
     *  @param D The D value to be used by the controller */
    public GenericPID(SparkBase motor, SparkBase.ControlType controlType, double P, double I, double D) {
        this.motor = motor;
        controller = motor.getClosedLoopController();

        this.controlType = controlType;
        
        this.P = P;
        this.I = I;
        this.D = D;
        config.closedLoop.pid(P, I, D);
        applyConfig();
    }

    // Private helper methods
    private boolean usingPositionControl() {return controlType.equals(ControlType.kPosition); }
    private void applyConfig() {motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}


    // Accessor methods
    public double getP() { return P; }
    public double getI() { return I; }
    public double getD() { return D; }
    public double getSetpoint() { return usingPositionControl() ? setpoint/ratio : setpoint; }
    public double getRatio(){ return ratio; }
    public SparkBase.ControlType getControlType() { return controlType; }
    public double getMin() { return min; }
    public double getMax() { return max; }
    /** Calculates whether the motor has reached its setpoint based off of the set control type
     *  @param tolerance How far the actual value is allowed to be from the setpoint
     *  @return Whether the PID has reached its setpoint */
    public boolean atSetpoint(double tolerance) { return Math.abs(getMeasurement() - getSetpoint()) <= tolerance; }
    /** @return The measured value from the motor based on the set {@link CANSparkMax.ControlType control type}. */
    public double getMeasurement() {
        switch(controlType) {
            case kDutyCycle: return motor.getAppliedOutput();
            case kVelocity: return motor.getEncoder().getVelocity();
            case kVoltage: return motor.getAppliedOutput() * motor.getBusVoltage();
            case kPosition: return motor.getEncoder().getPosition() / ratio;
            case kCurrent: motor.getOutputCurrent();
            default: return 0;
        }
    }

    public void setPID(double P, double I, double D) { 
        this.P = P;
        this.I = I;
        this.D = D;
        config.closedLoop.pid(P, I, D);
        applyConfig();
    }
    public void setControlType(SparkBase.ControlType controlType) { this.controlType = controlType; }
    /** @param ratio Incoming position instructions are multiplied by this, outgoing ones are divided */
    public void setRatio(double ratio){ this.ratio = ratio;}
    public void setMin(double min) { this.min = min*ratio; setSetpoint(this.setpoint); }
    public void setMax(double max) { this.max = max*ratio; setSetpoint(this.setpoint); }
    /** Set the min and max input values */
    public void setInputRange(double min, double max) {setMin(min); setMax(max); setSetpoint(this.setpoint); }
    /** Set the min and max output speed [-1,1] */
    public void setOutputRange(double min, double max) { config.closedLoop.outputRange(min, max); }
    
    /** Sets the setpoint and forces it within user-set bounds [min,max] */
    public void setSetpoint(double set) {
        if (usingPositionControl()) set *= ratio;
        
        this.setpoint = set < min ? min : 
                      ( set > max ? max : set );
    }

    /** Set the PID gains to match the object settings */
    public void updatePID() { 
        setPID(P, I, D);
    }

    /** Activates the PID controller using last known setpoint */
    public void activate() { activate(this.setpoint); }
    /** Activate the PID controller
     *  @param setpoint The PID controller's target */
    public void activate(double setpoint) {
        updatePID();
        setSetpoint(setpoint);
        controller.setReference(this.setpoint, this.controlType);
    }
    /** Alias for activate. Starts the PID controller using last known setpoint */
    public void start() { activate(); }

    /** Sets the PID gains to 0, without changing the stored values */
    public void pause(){
        config.closedLoop.pid(0, 0, 0);
        applyConfig();
    }

    /** Sets the PID gains to 0, as well as the stored values */
    public void stop() {
        this.setPID(0, 0, 0);
    }
}
