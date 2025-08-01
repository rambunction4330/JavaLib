package frc.robot.Lib;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class SparkMaxVelocityController {
    private  SparkMax sparkMax;

    private final ControlType controlType;

    private final createInfo info;

    public record motorConfig(int id, SparkLowLevel.MotorType type, boolean inverted,
            int maxCurrent) {
    }

    public record pidConfig(double p, double i, double d, Angle tolerance, double iZone,
            double iMaxAccumulator, double maxOutput, double minOutput) {
    }

    public record feedBack(FeedbackSensor encoderType, double gearRatio) {
    }

    public record profiling(boolean usingMaxMotion, AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    }

    public record createInfo(motorConfig motorConfig, pidConfig pidConfig,
            SparkMaxVelocityController leader, feedBack feedBack, profiling profileConfig) {
    }

    public SparkMaxVelocityController(createInfo info) {

        this.info = info;
        SparkMaxConfig config = new SparkMaxConfig();
        sparkMax = new SparkMax(info.motorConfig.id, info.motorConfig.type);

        config.smartCurrentLimit(info.motorConfig.maxCurrent);
        config.inverted(info.motorConfig.inverted);
        config.encoder.countsPerRevolution(42);

        config.closedLoop.feedbackSensor(info.feedBack.encoderType);
        config.closedLoop.pid(info.pidConfig.p, info.pidConfig.i, info.pidConfig.d);
        config.closedLoop.iZone(info.pidConfig.iZone);
        config.closedLoop.iMaxAccum(info.pidConfig.iMaxAccumulator);
        config.closedLoop.maxOutput(info.pidConfig.maxOutput);
        config.closedLoop.minOutput(info.pidConfig.minOutput);

        if (info.profileConfig.usingMaxMotion) {
            controlType = SparkBase.ControlType.kMAXMotionVelocityControl;
            config.closedLoop.maxMotion
                    .maxVelocity(info.profileConfig.maxVelocity.magnitude() * info.feedBack.gearRatio);
            config.closedLoop.maxMotion
                    .maxAcceleration(info.profileConfig.maxAcceleration.magnitude() * info.feedBack.gearRatio);
        } else {
            controlType = SparkBase.ControlType.kVelocity;
        }

        config.follow(info.leader.getMotor(), info.leader.info.motorConfig.inverted);

        sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    void setVelocity(AngularVelocity velocity) {
        AngularVelocity targetVelocity = AngularVelocity.ofBaseUnits(velocity.magnitude(), RevolutionsPerSecond);
        sparkMax.getClosedLoopController().setReference(targetVelocity.magnitude(), controlType);
    }

    void setPower(double power) {
        sparkMax.set(power);
    }

    void disable() {
        sparkMax.disable();
    }

    void stop() {
        sparkMax.stopMotor();
    }

    SparkMax getMotor() {
        return sparkMax;
    }
}
