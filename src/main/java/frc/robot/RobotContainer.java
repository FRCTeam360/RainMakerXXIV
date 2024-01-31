// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import frc.robot.commands.Autos;
=======
import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerFlywheel;
>>>>>>> Woodbot
import frc.robot.commands.PowerIntake;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerLinkage;
<<<<<<< HEAD
import frc.robot.commands.PowerShooter;
import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.RunLinkage;
import frc.robot.commands.SetFlywheel;
import frc.robot.commands.SetIntake;
=======
import frc.robot.commands.RobotOrientedDrive;
//import frc.robot.commands.SetFlywheel;
import frc.robot.commands.SetLinkage;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.FieldOrientedDrive;
>>>>>>> Woodbot
import frc.robot.generated.TunerConstants;
import frc.robot.hardware.IntakeIOSparkMax;
import frc.robot.hardware.LinkageIOSparkMax;
import frc.robot.hardware.ShooterIOSparkMax;
import frc.robot.io.IntakeIO;
import frc.robot.io.ShooterIO;
import frc.robot.sim.ShooterIOSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
<<<<<<< HEAD
=======
import frc.robot.subsystems.Flywheel;
>>>>>>> Woodbot
import frc.robot.subsystems.Linkage;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Command;
=======
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
>>>>>>> Woodbot
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
<<<<<<< HEAD
  // declared as final in example code, but gives error in our code
  private Intake intake;
  private Shooter shooter;
  private Linkage linkage;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;

  private PowerIntake powerIntake;
  private PowerIntakeReversed powerIntakeReversed;
  private PowerLinkage powerLinkage;
  private PowerShooter powerShooter;
  private RunExtendIntake runExtendIntake;
  private RunLinkage runLinkage;
  private SetFlywheel setFlywheel;
  private SetIntake setIntake;
  // private SetLinkage setLinkage;
  // private SetpointFlywheel setpointFlywheel;

  
=======
  private final SendableChooser<Command> autoChooser;
>>>>>>> Woodbot
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER);
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);

<<<<<<< HEAD
  final double MaxSpeed = 13.7; // used to be 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI * 3; // Half a rotation per second max angular velocity
=======
  // subsystems
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = Intake.getInstance();
  private final Flywheel flywheel = Flywheel.getInstance();
  private final Linkage linkage = Linkage.getInstance();

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();


  // auto commands
  //private final SetFlywheel setFlywheel = new SetFlywheel();

  // tele commands
  private final RunExtendIntake runExtendIntake = new RunExtendIntake();
  private final PowerIntakeReversed powerIntakeReversed = new PowerIntakeReversed();
  private final PowerIntake powerIntake = new PowerIntake();
  private final PowerFlywheel powerFlywheel = new PowerFlywheel();
  private final PowerLinkage powerLinkage = new PowerLinkage();
  private final SetLinkage setLinkage = new SetLinkage();
  private final FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive();
  private final RobotOrientedDrive robotOrientedDrive = new RobotOrientedDrive();
  

>>>>>>> Woodbot

  // public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(Constants.MAX_SPEED * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * 0.1)
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // // driving in open loop
  
  final Rotation2d setAngle = Rotation2d.fromDegrees(0);

  /* Setting up bindings for necessary control of the swerve drive platform */
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(Constants.MAX_SPEED_MPS);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
<<<<<<< HEAD
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        shooter = new Shooter(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        linkage = new Linkage(new LinkageIOSparkMax());
        // commandSwerveDrivetrain = new CommandSwerveDrivetrain(new CommandSwerveDrivetrainIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // CHANGE SHOOTER AND SHOOTER LINKAGE TO SIM LATER
        // shooter = new Shooter(new ShooterIOSparkMax());
        // intake = new Intake(new IntakeIOSparkMax());
        // linkage = new Linkage(new LinkageIOSparkMax());
        // commandSwerveDrivetrain = new CommandSwerveDrivetrain(new CommandSwerveDrivetrainIOSparkMax());
        break;

      default:
        // Replayed robot, disable IO implementations
        // shooter = new Shooter(new ShooterIO() {});
        // intake = new Intake(new IntakeIO() {});
        // shooterLinkage = new ShooterLinkage(new ShooterLinkageIO() {});
        break;
    }
=======
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
>>>>>>> Woodbot
    // Configure the trigger bindings
    NamedCommands.registerCommand("RunExtendIntake", runExtendIntake);
    NamedCommands.registerCommand("Wait1", new WaitCommand(1));
    NamedCommands.registerCommand("Shoot", powerFlywheel);
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
<<<<<<< HEAD
    shooter.setDefaultCommand(powerShooter);
    intake.setDefaultCommand(powerIntake);
    linkage.setDefaultCommand(powerLinkage);
     drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> drive.withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1) * MaxSpeed) //drive forward with negative y
                // negative Y (forward)
                .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1) * MaxSpeed) // drive left with negative x
                .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(), 0.1) * MaxAngularRate) // drive counterclockwise with negative x                                                                                                  
    ));
=======
    //flywheel.setDefaultCommand(setFlywheel);
    //intake.setDefaultCommand(runIntake);
    //linkage.setDefaultCommand(powerLinkage);
    drivetrain.setDefaultCommand(fieldOrientedDrive);
  }

  public void onDisable() {
    drivetrain.robotOrientedDrive(0, 0, 0);
>>>>>>> Woodbot
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
<<<<<<< HEAD
   */  


  private void configureBindings() {
  
    powerIntake = new PowerIntake(intake);
    powerIntakeReversed = new PowerIntakeReversed(intake);
    powerLinkage = new PowerLinkage(linkage);
    powerShooter = new PowerShooter(shooter);
    runExtendIntake = new RunExtendIntake(intake);
    runLinkage = new RunLinkage(linkage);
    setIntake = new SetIntake(intake);
    // setLinkage = new SetLinkage(linkage);
    // not sure whether adding in the speed and velocity parameters will make a difference, but it won't work without them
    // setpointFlywheel = new SetpointFlywheel(0.0, shooter);
    setFlywheel = new SetFlywheel(0.0, shooter);
    
    operatorController.rightTrigger(.005).whileTrue(powerIntake);
    operatorController.leftTrigger(.005).whileTrue(powerIntakeReversed);
    // operatorController.leftBumper().whileTrue(runIntake);
    // operatorController.rightBumper().whileTrue(runIntakeReversed);
    operatorController.a().whileTrue(powerShooter);
    operatorController.x().whileTrue(new InstantCommand(() -> linkage.zero(), linkage));
=======
   */
  private void configureBindings() {
    driverController.a().whileTrue(drivetrain.turntoCMD(180.0, 0.0, 0.0));
    driverController.x().whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));

    operatorController.y().whileTrue(new InstantCommand(() -> flywheel.runTop(0.8), flywheel));
    operatorController.b().whileTrue(new InstantCommand(() -> flywheel.setTopRPM((2000))));

    operatorController.x().whileTrue(new InstantCommand(() -> flywheel.runBottom(0.8), flywheel));
    operatorController.a().whileTrue(new InstantCommand(() -> flywheel.setBottomRPM((2000))));

    operatorController.pov(0).whileTrue(new InstantCommand(() -> flywheel.setBothRPM(5000)));
    // OPERATOR CONTROLLER BINDINGS
    operatorController.leftTrigger(.005).whileTrue(powerIntakeReversed);
    operatorController.rightTrigger(.005).whileTrue(powerIntake);
    // operatorController.a().whileTrue(new SetFlywheel());
    operatorController.b().toggleOnTrue(runExtendIntake);
    // operatorController.y().whileTrue(powerIntake);
    // //operatorController.x().whileTrue(new InstantCommand(() -> linkage.setTo90(),linkage));

    //operatorController.a().whileTrue(new InstantCommand(() -> flywheel.setSpeed(5500), flywheel));
    //operatorController.b().whileTrue(new InstantCommand(() -> linkage.setAngle(85), linkage));
    //.x().toggleOnTrue(new InstantCommand(() -> linkage.setAngle(100), linkage));

    // // DRIVER CONTROLLER BINDINGS
    // driverController.x().whileTrue(new InstantCommand(() -> drivetrain.xOut(), drivetrain));
    // driverController.a().whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));
>>>>>>> Woodbot
    
    drivetrain.registerTelemetry(logger::telemeterize);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // An example command will be run in autonomous
  }
}
