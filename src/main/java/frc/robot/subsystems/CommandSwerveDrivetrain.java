package frc.robot.subsystems;

import java.sql.Driver;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.CompBotConstants;
import frc.robot.utils.CommandLogger;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private static SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle();
    private PhoenixPIDController headingController;
    private PIDController visionTargetPIDController;
    final double MAX_SPEED_MPS = Constants.MAX_SPEED_MPS;

    GenericEntry kPEntry;
    GenericEntry kIEntry;
    GenericEntry kDEntry;

    private double headingKP = 10;
    private double headingKI = 0.2;
    private double headingIZone = 0.17;
    private double headingKD = 0.069;

    // Point to vision target PID gains
    private double visionTargetKP = 0.3;
    private double visionTargetKI = 0.0;
    private double visionTargetKD = 0.0;
    private double visionTargetIZone = 0.0;

    // Create a new SysId Routine for characterizing the drive
    public SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, null, null, // Use default config
                    (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                    (voltage) -> this.runCharacterizationVolts(voltage.in(Volts)),
                    null, // No log consumer, since data is recorded by AdvantageKit
                    this));

    // See Robot Container: The methods below return Command objects

    public void runCharacterizationVolts(double voltage) {
        double velocityX = voltage / 12.0 * MAX_SPEED_MPS;
        this.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(velocityX)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
    }

    private void setupShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("angle");
        tab.addNumber("current angle", () -> this.getPigeon2().getAngle());
        tab.addNumber("error", () -> this.getAngleError());
        tab.addNumber("setpoint", () -> Math.toDegrees(this.headingController.getSetpoint()));

        // kPEntry = tab.add("kP", 0.0).getEntry();
        // kIEntry = tab.add("kI", 0.0).getEntry();
        // kDEntry = tab.add("kD", 0.0).getEntry();
    }

    private void configurePID() {
        headingController = new PhoenixPIDController(headingKP, headingKI, headingKD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(2));
        headingController.setIntegratorRange(-headingIZone, headingIZone);
        visionTargetPIDController = new PIDController(visionTargetKP, visionTargetKI, visionTargetKD);
        visionTargetPIDController.setTolerance(Math.toRadians(1));
        visionTargetPIDController.setIntegratorRange(-visionTargetIZone, visionTargetIZone);
    }

    public void updateDriveGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        Slot0Configs slot0Configs = new Slot0Configs()
                .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV);
        for (int i = 0; i < 4; i++) {
            this.getModule(i).getDriveMotor().getConfigurator().apply(slot0Configs, 0.05);
        }
    }

    public void setupPathPlanner() {
        // numbers are not 100% accurate, needs to be verified
        double weight  = Units.lbsToKilograms(115.0+12.0);
        double radius = Units.inchesToMeters(20.0);
        double momentOfInertia = (1.0 / 6.0) * weight * Math.pow(radius, 2.0);
        double trackwidth = Units.inchesToMeters(19.25);
        double wheelbase = Units.inchesToMeters(19.25);
        double coefficientOfFriction = 0.7;
        DCMotor motor = DCMotor.getKrakenX60(1);
        double driveCurrentLimit = 80.0;

        double wheelRadius = Units.inchesToMeters(2.0);
        ModuleConfig moduleConfig = new ModuleConfig(wheelRadius, Constants.MAX_SPEED_MPS, coefficientOfFriction, motor, driveCurrentLimit, 4);
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config = new RobotConfig(
            weight,
            momentOfInertia,
            moduleConfig,
            trackwidth, 
            wheelbase);

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedForwards) -> this.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.75, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePID();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        setupPathPlanner();
        // setupShuffleboard();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePID();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setupPathPlanner();
        // setupShuffleboard();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public double getRadiansPerSecond() {
        return this.getState().speeds.omegaRadiansPerSecond;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command turntoCMD(boolean shouldEnd, Rotation2d desiredAngle, double velocityX, double velocityY) {
        desiredAngle = Rotation2d.fromDegrees(flipAngle(desiredAngle.getDegrees()));
        FieldCentricFacingAngle facingAngleCommand = drive
                .withTargetDirection(desiredAngle)
                .withVelocityX(velocityX)
                .withVelocityY(velocityY);
        facingAngleCommand.HeadingController = headingController;

        if (shouldEnd) {
            return this.applyRequest(() -> facingAngleCommand)
                    .raceWith(new EndWhenFacingAngle(headingController));

        }

        return this.applyRequest(() -> facingAngleCommand);

    }

    public Command turntoCMD(boolean shouldEnd, double desiredAngle, double velocityX, double velocityY) {
        Rotation2d rotation = Rotation2d.fromDegrees(desiredAngle);
        return turntoCMD(shouldEnd, rotation, velocityX, velocityY);

    }

    /**
     * Flips a given angle by 180 degrees
     * 
     * @param angle
     */
    public double flipAngle(double angle) {
        return angle + 180;
    }

    private double getAngleError() {
        return Math.toDegrees(headingController.getPositionError());
    }

    public boolean isFacingAngle() {
        return headingController.atSetpoint();
    }

    public double getAngle() {
        return getPose().getRotation().getDegrees();
    }

    public Rotation2d getRotation2d() {
        return getPose().getRotation();
    }

    public void zero() {
        this.getPigeon2().reset();
    }

    public void xOut() {
        this.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    public void fieldCentricDrive(double left, double forward, double rotation, double MAX_SPEED, double MAX_ANGULAR) {
        this.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(forward * MAX_SPEED)
                .withVelocityY(left * MAX_SPEED)
                .withRotationalRate(-rotation * MAX_ANGULAR));
    }

    public void robotCentricDrive(double left, double forward, double rotation) {
        this.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(forward * Constants.MAX_SPEED_MPS)
                .withVelocityY(left * Constants.MAX_SPEED_MPS)
                .withRotationalRate(-rotation * Constants.MAX_ANGULAR_RATE));
    }

    // drive robot with field centric angle
    public void driveFieldCentricFacingAngle(double forward, double left, double desiredAngle) {

        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(forward * Constants.MAX_SPEED_MPS)
                .withVelocityY(left * Constants.MAX_SPEED_MPS)
                .withTargetDirection(Rotation2d.fromDegrees(desiredAngle));
        request.HeadingController = headingController;
        // request.ForwardReference = SwerveRequest.ForwardReference.RedAlliance;
        this.setControl(request);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    private void resetPose(Pose2d pose) {
        seedFieldRelative(pose);
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = this.getModule(i).getCurrentState();
        }
        ChassisSpeeds k = this.m_kinematics.toChassisSpeeds(states);
        double x = k.vxMetersPerSecond;
        double y = k.vyMetersPerSecond;
        return k;
    }

    private void driveRobotRelative(ChassisSpeeds speed) {
        this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speed));
    }

    private class EndWhenFacingAngle extends Command {
        PhoenixPIDController headingController;

        public EndWhenFacingAngle(PhoenixPIDController headingController) {
            this.headingController = headingController;
        }

        @Override
        public boolean isFinished() {
            return headingController.atSetpoint();
        }
    }

    private double pastTX = 100.0;
    private Rotation2d visionTargetRotation = new Rotation2d();

    public void setPastTX(double tx) {
        pastTX = tx;
    }

    /**
     * this method should take in three doubles, forward, left, and tx value from
     * vision system
     * it should then use the tx value to turn the robot to the target, and then
     * drive forward and left in field centric mode
     */
    public void pointAtTarget(double forward, double left, double tx) {
        if (pastTX != tx) {
            pastTX = tx;
            visionTargetRotation = this.getPose().getRotation().minus(Rotation2d.fromDegrees(tx));
        }
        // Call fieldCentric request with forward and left and the ouput of our
        // PIDController
        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(forward * Constants.MAX_SPEED_MPS)
                .withVelocityY(left * Constants.MAX_SPEED_MPS)
                .withTargetDirection(visionTargetRotation);
        request.HeadingController = headingController;
        this.setControl(request);
    }

    // checks if vision PID is at setpoint
    public boolean isAtVisionTarget() {
        return visionTargetPIDController.atSetpoint();
    }

    public boolean isFlat() {
        double currentPitch = this.getPigeon2().getPitch().getValueAsDouble();
        if (Math.abs(currentPitch - Constants.DRIVETRAIN_PITCH_AUTO_INIT) < 2.0 || DriverStation.isTeleop()) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Swerve: Current Pose", this.getPose());
        Logger.recordOutput("Swerve: Rotation", this.getRotation2d());
        Logger.recordOutput("Swerve: Angle", this.getAngle());
        Logger.recordOutput("swerve: pithc", this.isFlat());
        // String moduleName = "null";
        // for (int i = 0; i < 4; i++) {
        // switch (i) {
        // case 0:
        // moduleName = "Front Left: ";
        // break;
        // case 1:
        // moduleName = "Front Right: ";
        // break;
        // case 2:
        // moduleName = "Back Left: ";
        // break;
        // case 3:
        // moduleName = "Back Right: ";
        // break;
        // }
        // Logger.recordOutput("Swerve " + moduleName + "Drive Voltage",
        // this.getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Swerve " + moduleName + "Drive Current",
        // this.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Swerve " + moduleName + "CANCoder Position",
        // this.getModule(i).getCANcoder().getPosition().getValueAsDouble());
        // Logger.recordOutput("Swerve " + moduleName + "Steer Voltage",
        // this.getModule(i).getSteerMotor().getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Swerve " + moduleName + "Steer Current",
        // this.getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble());

        // }
        // Logger.recordOutput("Rotation2d", this.getPigeon2().getRotation2d());
        Logger.recordOutput("Swerve: CurrentState", this.getState().ModuleStates);
        Logger.recordOutput("Swerve: TargetState", this.getState().ModuleTargets);
    }
}