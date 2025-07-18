package frc.robot.subsystems;

import frc.robot.Lib.SparkMaxPositionController;
import frc.robot.Lib.SparkMaxPositionController.createInfo;
import frc.robot.Lib.SparkMaxPositionController.motorConfig;
import frc.robot.Lib.SparkMaxPositionController.pidConfig;
import frc.robot.Lib.SparkMaxPositionController.range;
import frc.robot.Lib.SparkMaxPositionController.feedBack;
import frc.robot.Lib.SparkMaxPositionController.profiling;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.units.Units;


public class ExampleConstants {
    public static class SparkConstants {
 
        SparkMaxPositionController controller1 = new SparkMaxPositionController(
            new createInfo(
            new motorConfig(50,SparkLowLevel.MotorType.kBrushless,  false, 40), 
            new pidConfig(0, 0, 0, null, 0, 0, 0, 0),
            null, 
            new range(true, Rotations.of(0), Rotations.of(0.5)), 
            new feedBack(FeedbackSensor.kPrimaryEncoder, 9), 
            new profiling(false, null, null)));

        
        SparkMaxPositionController controller2 = new SparkMaxPositionController(
            new createInfo(
            new motorConfig(51,SparkLowLevel.MotorType.kBrushless,  false, 40), 
            new pidConfig(0, 0, 0, null, 0, 0, 0, 0),
            controller1, 
            new range(true, Rotations.of(0), Rotations.of(0.5)), 
            new feedBack(FeedbackSensor.kPrimaryEncoder, 9), 
            new profiling(false, null, null)));

    }
}
