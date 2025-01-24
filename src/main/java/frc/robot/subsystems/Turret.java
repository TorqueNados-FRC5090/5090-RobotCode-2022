package frc.robot.subsystems;

// Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Controls the turret system
public class Turret {
    SparkMax motor;

    // Constructor
    public Turret(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);
    }

    // Accessors
    public double getRPM() { return motor.getEncoder().getVelocity(); }
    public double getPosition() { return motor.getEncoder().getPosition(); }
    public SparkMax getMotor() { return motor; }

    // Sets motor to specified power
    public void setPower(double pwr) { motor.set(pwr); }

    // Stops the turret
    public void off() { motor.set(0); }
}
