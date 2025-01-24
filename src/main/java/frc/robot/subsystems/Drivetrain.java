package frc.robot.subsystems;

// Imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Drivetrain {
    // Declare motors
    // Motors are named based on their position
    // eg. Rear Left Motor --> RLMotor
    // eg. Front Right Motor --> FRMotor
    private SparkMax FLMotor;
    private SparkMax FRMotor;
    private SparkMax RLMotor;
    private SparkMax RRMotor; 
    private DifferentialDrive drivetrain;

    // Constructor method
    public Drivetrain( int FLMotorID, int FRMotorID, int RLMotorID, int RRMotorID ) {
        // Initialize motors
        this.FLMotor = new SparkMax(FLMotorID, MotorType.kBrushless);
        this.FRMotor = new SparkMax( FRMotorID, MotorType.kBrushless);
        this.RLMotor = new SparkMax( RLMotorID, MotorType.kBrushless);
        this.RRMotor = new SparkMax( RRMotorID, MotorType.kBrushless);

        // Config RL to follow FL
        SparkMaxConfig RLConfig = new SparkMaxConfig();
        RLConfig.follow(FLMotor);
        RLMotor.configure(RLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Config RR to follow FR
        SparkMaxConfig RRConfig = new SparkMaxConfig();
        RRConfig.follow(FRMotor);
        RRMotor.configure(RRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Init drivetrain
        drivetrain = new DifferentialDrive(FLMotor, FRMotor);
    }

    // Accessor methods (getters)
    public DifferentialDrive getDrivetrain() { return drivetrain; }
}

