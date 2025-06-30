package frc.robot.Lib;

import java.util.ArrayList;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;

public class SparkMaxPositionController {

    private final SparkMax sparkMax;

    private final ControlType controlType;

    private final double gearRatio;
    private final createInfo info;

    // private final TrapezoidProfile profile;

    public record motorConfig(int id, SparkLowLevel.MotorType type, boolean inverted,
            int maxCurrent) {
    }

    public record pidConfig(double p, double i, double d, Angle tolerance, double iZone,
            double iMaxAccumulator, double maxOutput, double minOutput, TrapezoidProfile profile) {
    }

    public record range(boolean isContinuous, int minPosition, int maxPosition) {
    }

    public record feedBack(FeedbackSensor encoderType, double gearRatio) {
    }

    public record createInfo(motorConfig motorConfig, pidConfig pidConfig,
            ArrayList<SparkMaxPositionController> following, range range, feedBack feedBack) {
    }

    public SparkMaxPositionController(final createInfo info) {

        this.info = info;
        SparkMaxConfig config = new SparkMaxConfig();
        sparkMax = new SparkMax(info.motorConfig.id, info.motorConfig.type);
        this.gearRatio = info.feedBack.gearRatio;

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
            // fill when understood
            config.closedLoop.positionWrappingInputRange(0, 0);

        }

        controlType = SparkBase.ControlType.kPosition;

        for (SparkMaxPositionController leader : info.following) {
            config.follow(leader.info.motorConfig.id, leader.info.motorConfig.inverted);
        }
       
        sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

}
