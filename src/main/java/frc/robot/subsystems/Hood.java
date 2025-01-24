package frc.robot.subsystems;

// Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Controls the hood of the shooter
public class Hood {
    SparkMax motor;

    // Constructor
    public Hood(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);

        // Invert motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Accessors
    public double getRPM() { return motor.getEncoder().getVelocity(); }
    public double getPosition() { return motor.getEncoder().getPosition(); }
    public SparkMax getMotor() { return motor; }

    // Sets motor to specified power
    public void setPower(double pwr) { motor.set(pwr); }

    // Stops the hood
    public void off() { motor.set(0); }
}