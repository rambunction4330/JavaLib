package frc.robot.Lib;

import java.util.ArrayList;
import java.util.Queue;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;
import com.fasterxml.jackson.databind.util.IgnorePropertiesUtil;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class TalonFXPositionController {

    enum isInverted {
        NOT_INVERTED,
        INVERTED
    }

    enum brakeMode {
        BRAKE,
        NEUTRAL
    }

    private TalonFX talonMotorController;

    private final createInfo info;

    public record motorConfig(int id, isInverted inverted,
            brakeMode mode, double minOutput, double maxOutput) {
    }

    public record pidConfig(double p, double i, double d, double kS, double kG) {
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

    public TalonFXPositionController(createInfo info) {
        this.info = info;

        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
        talonMotorController = new TalonFX(info.motorConfig.id);

        talonFXConfig.MotorOutput.Inverted = InvertedValue.valueOf(info.motorConfig.inverted.ordinal());
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.valueOf(info.motorConfig.mode.ordinal());
        talonFXConfig.MotorOutput.PeakReverseDutyCycle = info.motorConfig.minOutput;
        talonFXConfig.MotorOutput.PeakForwardDutyCycle = info.motorConfig.maxOutput;

        talonFXConfig.Slot0.kP = info.pidConfig.p;
        talonFXConfig.Slot0.kI = info.pidConfig.i;
        talonFXConfig.Slot0.kD = info.pidConfig.d;
        talonFXConfig.Slot0.kS = info.pidConfig.kS;
        talonFXConfig.Slot0.kG = info.pidConfig.kG;

        // Refer to these links when implementing any sort of Current Limits
        // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/4 and
        // https://www.chiefdelphi.com/t/current-limiting-on-swerve/454392/2

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = info.currentLimits.statorCurrentEnable;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = info.currentLimits.statorCurrentLimit.magnitude();
        talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = info.currentLimits.supplyCurrentEnable;
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = info.currentLimits.supplyCurrentLimit.magnitude();

        talonFXConfig.Feedback.SensorToMechanismRatio = info.feedBack.gearRatio;
        talonFXConfig.ClosedLoopGeneral.ContinuousWrap = info.range.isContinuous;

        talonMotorController.getConfigurator().apply(talonFXConfig);
    }

    public void setPosition(Angle position) {
        Angle targetPos = Angle.ofBaseUnits(MathUtil.clamp(position.magnitude(), info.range.maxPos.magnitude(),
                info.range.maxPos.magnitude()), Rotations);

        PositionDutyCycle request = new PositionDutyCycle(position);

        talonMotorController.setControl(request);

    }

    public Angle getPosition() {

        StatusSignal<Angle> signal = talonMotorController.getRotorPosition(true);

        return signal.getValue();

    }

    public void setPower(double power) {
        talonMotorController.set(power); 
    }

    public double getPower(){
        return talonMotorController.get(); 
    }

    public void setEncoderPosition() {

        

    }

    public void disable() {
        talonMotorController.disable();
    }

    public void stop() {
        talonMotorController.stopMotor();
    }

    public void follow(TalonFXPositionController followController) {

        boolean invert; 

        if(followController.info.motorConfig.inverted.ordinal() == 0){ invert = false; }
        else{ invert = true;  }
        
        talonMotorController.setControl(new Follower(followController.info.motorConfig.id, invert));
    }
}
