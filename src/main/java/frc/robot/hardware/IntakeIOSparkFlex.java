package frc.robot.hardware;

import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.io.IntakeIO;

public class IntakeIOSparkFlex implements IntakeIO{ 
    /* Creates a new IntakeIOSparkFlex */
    private final CANSparkFlex sparkFlex = new CANSparkFlex(Constants.INTAKE_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = sparkFlex.getEncoder(); 
    private final SparkPIDController pidController = sparkFlex.getPIDController();
    //Sensors: ports?
    private final DigitalInput sideSensor = new DigitalInput(Constants.INTAKE_SIDE_SENSOR_PORT);
    private final DigitalInput highSensor = new DigitalInput(Constants.INTAKE_HIGH_SENSOR_PORT);
    public IntakeIOSparkFlex(){
        sparkFlex.restoreFactoryDefaults(); 
        sparkFlex.setInverted(false);
        sparkFlex.setIdleMode(IdleMode.kBrake);
        final double GEAR_RATIO = 2.0;
        // get shuffleboard tab intake
        ShuffleboardTab tab = Shuffleboard.getTab("intake");
        tab.addBoolean("sensor 1", () -> this.getSideSensor());
        tab.addBoolean("sensor 2", () -> this.getHighSensor());
    }
    @Override 
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeSpeed = sparkFlex.get();
        inputs.output = sparkFlex.getAppliedOutput();
        inputs.amps = sparkFlex.getOutputCurrent();
        inputs.sensor = sideSensor.get();
        inputs.sensor = highSensor.get();
    }
    @Override
    public void set(double speed){
        sparkFlex.set(speed);
    }
    @Override
    public void stopMotor(){
        sparkFlex.stopMotor();
    }
    @Override
    public double getPower(){
        return sparkFlex.get();
    }
    @Override
    public double getOutputCurrent(){
        return sparkFlex.getOutputCurrent();
    }
    @Override
    public boolean getSideSensor(){
        return sideSensor.get();
    }
    @Override
    public boolean getHighSensor(){
        return highSensor.get();
    }

    @Override
    public double getEncoderValue() {
        return sparkFlex.getEncoder().getPosition();
    }

    @Override
    public void moveEncoder(double setpoint) {

    }
    @Override
    public void setEncoderValue(double encoderPosition) {
        sparkFlex.getEncoder().setPosition(encoderPosition);
    }
}
