package frc.robot.misc_subclasses;

// Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.wrappers.GenericPID;
import static frc.robot.Constants.*;

// This object is used to handle SmartDashboard outputs
public class Dashboard {
    // Constructor method
    public Dashboard() {}

    // Prints the rpm of the shooter
    public void printShooterRPM( Shooter shooter ) {
        SmartDashboard.putNumber("ShooterRPM", Math.round(shooter.getLeaderMotorRPM()));
    }

    // Prints the state of the elevator's storage
    public void printElevatorStorage(Elevator elevator) {
        SmartDashboard.putBoolean("Top Sensor", elevator.topSensorHasBall());
        SmartDashboard.putBoolean("Bottom Sensor", elevator.bottomSensorHasBall());
    }

    // Prints the current position of a turret in degrees
    public void printTurretDegrees(Turret turret) {
        SmartDashboard.putNumber("TurretPos", turret.getPosition() / TURRET_RATIO);
    }

    // Prints the current position of the hood in degrees
    public void printHoodDegrees(Hood hood) {
        SmartDashboard.putNumber("HoodPos", hood.getPosition());
    }

    // Prints the current position of the left climber
    public void printLeftClimberPosition(Climber leftMotor) {
        SmartDashboard.putNumber("LeftClimbPos", leftMotor.getLeftClimberPosition());
    }

    // Prints the current position of the right climber
    public void printRightClimberPosition(Climber rightMotor) {
        SmartDashboard.putNumber("RightClimbPos", rightMotor.getRightClimberPosition());
    }

    // Prints the current position of a target relative to limelight
    public void printLimelightData(Limelight limelight) {
        SmartDashboard.putNumber("Distance from Target", limelight.getDistance());
        SmartDashboard.putNumber("Rotational Angle to Target", limelight.getRotationAngle());
    }

    // Prints relevant data from a PID controller. If a value is manually 
    // input into the dashboard, the pid setpoint will update accordingly
    public void PIDtoDashboard(GenericPID pid, int id) {
        double setpointD = SmartDashboard.getNumber("PID-" + id + " Setpoint", pid.getSetpoint());

        if( setpointD != pid.getSetpoint() )
            pid.setSetpoint(setpointD);

        SmartDashboard.putNumber("PID-" + id + " Setpoint", pid.getSetpoint());
        SmartDashboard.putNumber("PID-" + id + " Measurement", pid.getMeasurement());
        SmartDashboard.putString("PID-" + id + " Domain", "[" + pid.getMin() + ", " + pid.getMax() + "]");
    }

    // Prints relevant data from a PID controller. If a value is manually 
    // input into the dashboard, the pid setpoint will update accordingly
    public void PIDtoDashboard(GenericPID pid, String name) {
        double setpointD = SmartDashboard.getNumber(name + " Setpoint", pid.getSetpoint());

        if( setpointD != pid.getSetpoint() )
            pid.setSetpoint(setpointD);
            
        SmartDashboard.putNumber(name + " Setpoint", pid.getSetpoint());
        SmartDashboard.putNumber(name + " Measurement", pid.getMeasurement());
        SmartDashboard.putString(name + " Domain", "[" + pid.getMin() + ", " + pid.getMax() + "]");
    }
}
