package frc.robot;

// Controller Imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

// Actuation imports (Motors, Compressors, etc.)
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.wrappers.GenericPID;
import com.revrobotics.CANSparkMax.ControlType;

// Camera imports
import edu.wpi.first.cameraserver.CameraServer;

// Subsystem imports
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;

// Misc imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.misc_subclasses.Dashboard;
import frc.robot.misc_subclasses.Limelight;
import static frc.robot.Constants.*;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Controller ojects
  private Joystick joystick; 
  private XboxController xbox;

  // Subsystem objects
  private Shooter shooter;
  private Drivetrain drivetrain;
  private Dashboard dashboard;
  private Limelight limelight;
  private Elevator elevator; 
  private Intake intake;
  private Climber climber;
  private Turret turret;
  private Hood hood;

  // Misc variables/objects
  private DifferentialDrive robotDrive;
  private Compressor comp;
  private GenericPID turretPID;
  private GenericPID shooterPID;
  private double autonStartTime;
  
  // This function is run when the robot is first started up and should be used
  // for any initialization code.
  @Override
  public void robotInit() {
    // Initialize variables
    joystick = new Joystick(0);
    xbox  = new XboxController(1);
  
    drivetrain = new Drivetrain(7, 3, 6, 2);
    robotDrive = new DifferentialDrive(
      drivetrain.getLeftMotorGroup(), drivetrain.getRightMotorGroup());

    CameraServer.startAutomaticCapture();
    limelight = new Limelight();

    turret = new Turret(14);
    turretPID = new GenericPID(turret.getMotor(), ControlType.kPosition, .25);

    shooter = new Shooter(9, 5);    
    shooterPID = new GenericPID(shooter.getLeaderMotor(), ControlType.kVelocity, .00022, .0000005, 0);
    shooterPID.setOutputRange(-1,1);

    hood = new Hood(15);

    elevator = new Elevator(13, 0, 1);

    intake = new Intake(10);
    comp = new Compressor(0, PneumaticsModuleType.CTREPCM);

    climber = new Climber(11, 12, 2, 3);

    dashboard = new Dashboard();

  }

  // This function is called once at the start of auton
  @Override
  public void autonomousInit() {
    autonStartTime = Timer.getFPGATimestamp();
  }

  // This function is called every 20ms during auton
  @Override
  public void autonomousPeriodic() { 
    double currentTime = Timer.getFPGATimestamp() - autonStartTime;

    if((currentTime > 2) && currentTime < 10)
     shooter.set(-.55);

    if(currentTime > 5 && currentTime < 10) {
      elevator.set(1);
    }

    if(currentTime > 10 && currentTime < 11.5) {
      drivetrain.getLeftMotorGroup().set(0.35);
      drivetrain.getRightMotorGroup().set(-0.35);
      elevator.off();
      shooter.off();
    } 

    if(currentTime > 11.5) {
      drivetrain.getLeftMotorGroup().set(0);
      drivetrain.getRightMotorGroup().set(0);
    }

    if(currentTime > 12)
    intake.down();

  }
  
  // This function is called once at the start of teleop
  @Override
  public void teleopInit() {

    turretPID.setInputRange(-75 * TURRET_RATIO, 75 * TURRET_RATIO);
    turretPID.setSetpoint(0);

    comp.enableDigital();
  }

  // This function is called every 20ms during teleop
  @Override
  public void teleopPeriodic() {
    // Puts the robot in arcade drive
    robotDrive.arcadeDrive(-joystick.getRawAxis(0), joystick.getRawAxis(1));

    // Joystick trigger activates motor
    if(joystick.getTrigger())
      intake.set(.75);
    else if(joystick.getRawButton(2))
      intake.set(-.75);
    else
      intake.motorOff();

    // Manually control the turret with bumpers
    if(xbox.getLeftBumper()) {
      turretPID.pause();
      turret.setPower(.3);
    }
    else if(xbox.getRightBumper()) {
      turretPID.pause();
      turret.setPower(-.3);
    }
    else if(turretPID.getP() == 0)
      turret.off();

    // Dpad controls
    switch(xbox.getPOV()){
      case 0: // UP
        elevator.set(.3);
        break;
      case 180: // DOWN
        elevator.set(-.3);
        break;
      case 90: // RIGHT
        shooter.increasePowerBy(.004);
        break;
      case 270: // LEFT
        shooter.decreasePowerBy(.004);
        break;
      case -1: // NOT PRESSED
        elevator.off();
    }

    // Right trigger pushes a ball into the shooter
    if(xbox.getRightTriggerAxis() > 0)
      elevator.shoot();
    else
      elevator.auto();

    if(xbox.getLeftTriggerAxis() > 0) {
      turretPID.activate(
        ((turret.getPosition() / TURRET_RATIO) - limelight.getRotationAngle()) * TURRET_RATIO );

      shooterPID.activate((.056918 * Math.pow(limelight.getDistance(), 2)) + (1.96339 * limelight.getDistance()) + (3022.7));
    }
      
    // Left stick Y-axis controls left climber arm
    if(Math.abs(xbox.getLeftY()) > CLIMBER_DEADZONE )
      climber.setLeft(xbox.getLeftY());
    else  
      climber.leftOff();

    // Right stick Y-axis controls left climber arm
    if(Math.abs(xbox.getRightY()) > CLIMBER_DEADZONE )
      climber.setRight(xbox.getRightY());
    else  
      climber.rightOff();

    // X button lowers intake
    if(xbox.getXButton())
      intake.down();
    // Y button raises intake
    else if(xbox.getYButton())
      intake.up();

    // joystick controls intake state
    if(joystick.getRawButton(3))
      intake.down();
    else if (joystick.getRawButton(4))
      intake.up();

    // Start and back control the hood
    if(xbox.getStartButton())
      hood.setPower(-.1);
    else if(xbox.getBackButton())
      hood.setPower(.1);
    else
      hood.setPower(0);
    
    // preset motor value to shoot ball
    if(xbox.getLeftStickButton()) {
      shooter.set(-.65);
    }

    // preset motor value to shoot ball at low speed (reject ball)
    if(xbox.getRightStickButton()) {
      shooter.set(-.3);
    }


    // B is essentially an e-stop
    if(xbox.getBButton()){
      shooter.off();
      elevator.off();
      climber.off();
      turret.off();
      hood.off();
      turretPID.pause();
    }

    // Update anything that needs to update
    shooter.updateCurrentPower();
    dashboard.PIDtoDashboard(shooterPID, "Shooter");
    elevator.update();
    dashboard.printElevatorStorage(elevator);
    dashboard.printTurretDegrees(turret);
    dashboard.PIDtoDashboard(turretPID, "Turret");
    limelight.updateLimelightTracking();
    dashboard.printLimelightData(limelight);
  }
}