package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.wrappers.LimitSwitch;

// Controls the climber
public class Climber {

    // Declare motors
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    
    // Declare limit switches
    private LimitSwitch leftLimitSwitch;
    private LimitSwitch rightLimitSwitch;

    // Constructor
    public Climber (int motorID1, int motorID2, int leftLimitSwitchID, int rightLimitSwitchID) {
        // Initialize motors
        leftMotor = new CANSparkMax(motorID1, MotorType.kBrushless);
        rightMotor = new CANSparkMax(motorID2, MotorType.kBrushless);

        leftLimitSwitch = new LimitSwitch(leftLimitSwitchID);
        rightLimitSwitch = new LimitSwitch(rightLimitSwitchID);
    }

    // Accessor methods
    public double getLeft() { return leftMotor.get(); }
    public double getRight() { return rightMotor.get(); }

    // Set left arm's power
    public void setLeft(double pwr) { 
        leftMotor.set(
            leftLimitSwitch.isPressed() ? pwr : Math.abs(pwr));
    }

    // Set right arm's power
    public void setRight(double pwr) { 
        rightMotor.set(
            rightLimitSwitch.isPressed() ? pwr : Math.abs(pwr));
    }

    // Set the power of the motors together
    public void set(double pwr) {
        leftMotor.set(
            leftLimitSwitch.isPressed() ? pwr : Math.abs(pwr));

        rightMotor.set(
            rightLimitSwitch.isPressed() ? pwr : Math.abs(pwr));
    }

    // Stop the climber
    public void off() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
