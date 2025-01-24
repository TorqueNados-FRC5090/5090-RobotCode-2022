package frc.robot.subsystems;

// Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake {
  // Variable Declaration
  private DoubleSolenoid dubs;
  private SparkMax motor;

  // Is true if the arms are up 
  // Motor should NOT move if this is true
  private boolean solenoidIsUp;

  // Constructor
  public Intake(int motorID) {
    // Initialize Variables
    this.dubs = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    this.motor = new SparkMax(motorID, MotorType.kBrushless);

    // Arms start in the up position
    this.solenoidIsUp = false;
  }

  // Accessor methods
  public boolean armsAreUp() { return this.solenoidIsUp; }
  public int getMotorRPM() { return (int) Math.round(this.motor.getEncoder().getVelocity()); }

  // Put arms up
  public void up() { dubs.set(DoubleSolenoid.Value.kForward); }

  // Put arms down
  public void down() { dubs.set(DoubleSolenoid.Value.kReverse); }

  // Set the motor to specified power
  public void set(double power) { if(!solenoidIsUp) motor.set(power); }

  // Turn motor off
  public void motorOff() { motor.set(0); }

  // If arms are up, they will move down
  // If arms are down they will move up
  public void toggleArms() {
    if(solenoidIsUp){
      dubs.set(DoubleSolenoid.Value.kReverse);
      solenoidIsUp = false;
    }
    else {
      dubs.set(DoubleSolenoid.Value.kForward);
      solenoidIsUp = true;
    }
  }
}
