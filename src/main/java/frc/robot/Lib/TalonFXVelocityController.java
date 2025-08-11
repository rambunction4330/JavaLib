package frc.robot.Lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class TalonFXVelocityController {
    enum isInverted {
        NOT_INVERTED,
        INVERTED
    }

    enum brakeMode {
        BRAKE,
        NEUTRAL
    }

    private TalonFX talonMotorController;

    public record motorConfig(int id, isInverted inverted,
            brakeMode mode, double minOutput, double maxOutput) {
    }

    public record pidConfig(double p, double i, double d, double kS, double kV) {
    }

    public record limitsConfig(boolean statorCurrentEnable, Current statorCurrentLimit, boolean supplyCurrentEnable,
            Current supplyCurrentLimit) {
    }

    // Gear Ratio is labeled sensorToMech ratio in configuration
    public record feedBack(double gearRatio) {
    }

    public record range(boolean isContinuous, Angle minPos, Angle maxPos) {
    }

    public record createInfo(motorConfig motorConfig, pidConfig pidConfig, limitsConfig currentLimits, range range,
            feedBack feedBack) {
    }

    private final createInfo info;

    TalonFXVelocityController(createInfo info) {
        this.info = info;
        TalonFXConfiguration config = new TalonFXConfiguration();

        talonMotorController = new TalonFX(info.motorConfig.id); 

        config.MotorOutput.Inverted = InvertedValue.valueOf(info.motorConfig.inverted.ordinal());
        config.MotorOutput.NeutralMode = NeutralModeValue.valueOf(info.motorConfig.mode.ordinal());
        config.MotorOutput.PeakReverseDutyCycle = info.motorConfig.minOutput;
        config.MotorOutput.PeakForwardDutyCycle = info.motorConfig.maxOutput;

        config.Slot0.kP = info.pidConfig.p;
        config.Slot0.kI = info.pidConfig.i;
        config.Slot0.kD = info.pidConfig.d;
        config.Slot0.kS = info.pidConfig.kS;
        config.Slot0.kV = info.pidConfig.kV; 

        config.CurrentLimits.StatorCurrentLimitEnable = info.currentLimits.statorCurrentEnable;
        config.CurrentLimits.StatorCurrentLimit = info.currentLimits.statorCurrentLimit.magnitude();
        config.CurrentLimits.SupplyCurrentLimitEnable = info.currentLimits.supplyCurrentEnable;
        config.CurrentLimits.SupplyCurrentLimit = info.currentLimits.supplyCurrentLimit.magnitude();

        config.Feedback.SensorToMechanismRatio = info.feedBack.gearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = info.range.isContinuous;

        talonMotorController.getConfigurator().apply(config);

    }

    public void setVelocity(AngularVelocity velocity){

        talonMotorController.setControl(new VelocityDutyCycle(velocity)); 

    }

    public AngularVelocity getVelocity(){
        StatusSignal<AngularVelocity> signal = talonMotorController.getVelocity(true);
        return signal.getValue(); 
    }

    public void setPower(double power){

        talonMotorController.set(power);
    }

    public double getPower(){
        return talonMotorController.get(); 
    }

    public void stop(){
        talonMotorController.stopMotor();
    }

    public void disable(){
        talonMotorController.disable();
    }


    public void follow(TalonFXVelocityController followController) {

        boolean invert; 

        if(followController.info.motorConfig.inverted.ordinal() == 0){ invert = false; }
        else{ invert = true;  }

        talonMotorController.setControl(new Follower(followController.info.motorConfig.id, invert));
    }

    public TalonFX getMotor(){
        return talonMotorController; 
    }
}
