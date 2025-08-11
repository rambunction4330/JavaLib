package frc.robot.Lib;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class SparkMaxPositionController {

    private final SparkMax sparkMax;

    private final ControlType controlType;

    private final createInfo info;

    public record motorConfig(int id, SparkLowLevel.MotorType type, boolean inverted,
            int maxCurrent) {
    }

    public record pidConfig(double p, double i, double d, Angle tolerance, double iZone,
            double iMaxAccumulator, double maxOutput, double minOutput) {
    }

    public record range(boolean isContinuous, Angle minPosition, Angle maxPosition) {
    }

    public record feedBack(FeedbackSensor encoderType, double gearRatio) {
    }

    public record profiling(boolean usingMaxMotion, AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    }

    public record createInfo(motorConfig motorConfig, pidConfig pidConfig,
            SparkMaxPositionController leadController, range range, feedBack feedBack, profiling profileConfig) {
    }

    public SparkMaxPositionController(final createInfo info) {

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

        if (info.range.isContinuous) {
            config.closedLoop.positionWrappingEnabled(true);
            config.closedLoop.positionWrappingInputRange(info.range.minPosition.magnitude() * info.feedBack.gearRatio,
                    info.range.maxPosition.magnitude() * info.feedBack.gearRatio);

        }

        if (info.profileConfig.usingMaxMotion) {
            controlType = SparkBase.ControlType.kMAXMotionPositionControl;
            config.closedLoop.maxMotion
                    .maxVelocity(info.profileConfig.maxVelocity.magnitude() * info.feedBack.gearRatio);
            config.closedLoop.maxMotion
                    .maxAcceleration(info.profileConfig.maxAcceleration.magnitude() * info.feedBack.gearRatio);
        } else {
            controlType = SparkBase.ControlType.kPosition;
        }

        config.follow(info.leadController.getMotor(), info.leadController.info.motorConfig.inverted);

        sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setPosition(Angle position, double ff) {

        Angle targetPos = Angle.ofBaseUnits(MathUtil.clamp(position.magnitude(), info.range.minPosition.magnitude(),
                info.range.maxPosition.magnitude()), Rotations);

        sparkMax.getClosedLoopController().setReference(targetPos.magnitude() * info.feedBack.gearRatio, controlType,
                ClosedLoopSlot.kSlot0, ff);

    }

    public Angle getPosition() {
        return Angle.ofBaseUnits(sparkMax.getEncoder().getPosition() / info.feedBack.gearRatio, Rotations);
    }

    public void setEncoderPosition(Angle position) {
        sparkMax.getEncoder().setPosition(position.magnitude());
    }

    public void setPower(double power) {
        sparkMax.set(power);
    }

    public double getPower() {
        return sparkMax.get();
    }

   public Angle getMaxPos() {
        return info.range.maxPosition;
    }

    public Angle getMinPos() {
        return info.range.minPosition;
    }

    public void stop() {
        sparkMax.stopMotor();
    }

    public void disable() {
        sparkMax.disable();
    }

    public SparkMax getMotor() {
        return sparkMax;
    }

}
