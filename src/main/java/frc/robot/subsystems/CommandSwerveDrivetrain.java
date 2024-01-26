package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    static SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
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

    public Command turntoCMD(Rotation2d desiredAngle, double velocityX, double velocityY) {
        return this.applyRequest(
                () -> drive.withTargetDirection(desiredAngle).withVelocityX(velocityX).withVelocityY(velocityY));
    }

    public float getAngle() {
        return (float) this.getPigeon2().getAngle();
    }

    public Rotation2d getRotation2d() {
        return this.getPigeon2().getRotation2d();
    }

    public void zero() {
        this.getPigeon2().reset();
    }

    public void fieldOrientedDrive(double leftX, double leftY, double rightX) {
        this.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(MathUtil.applyDeadband(leftY, 0.1) * Constants.MAX_SPEED_MPS)
                .withVelocityY(MathUtil.applyDeadband(leftX, 0.1) * Constants.MAX_SPEED_MPS)
                .withRotationalRate(MathUtil.applyDeadband(-rightX, 0.1) * Constants.MAX_ANGULAR_RATE)
                );
    }

    public void robotOrientedDrive(double leftX, double leftY, double rightX) {
        this.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(MathUtil.applyDeadband(leftY, 0.1) * Constants.MAX_SPEED_MPS)
                .withVelocityY(MathUtil.applyDeadband(leftX, 0.1) * Constants.MAX_SPEED_MPS)
                .withRotationalRate(MathUtil.applyDeadband(-rightX, 0.1) * Constants.MAX_ANGULAR_RATE)
                );
    }
}