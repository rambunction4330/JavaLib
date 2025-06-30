package frc.robot.Lib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;

/*
 * This is an implementation of the Logitech Controller Class. It may be useful to future users to understand 
 * its redundancy because all Logitech gamepads can be mapped to XBoxControllers, which already have implementations
 * in implementations in WPILib. Additionally, if the user is in need of command functionality with the Logitech Controller
 * specifically, one can do so using the GenericCommandHD Class.
 */
public class LogitechGamepad extends GenericHID {

    // Buttons
    public enum Buttons {
        kA(1),
        kB(2),
        kX(3),
        kY(4),
        leftBumper(5),
        rightBumper(6),
        backButton(7),
        startButton(8),
        leftStick(9),
        riightStick(10);

        public final int value;

        Buttons(int value) {
            this.value = value;
        }
    }

    // Axis
    public enum Axis {
        leftX(0),
        leftY(1),
        rightX(4),
        rightY(5),
        leftTrigger(2),
        rightTrigger(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }
    public LogitechGamepad(int port) {
        super(port);
    }
    
    //Axis

    public double getLeftX(){
        return getRawAxis(Axis.leftX.value); 
    }

    public double getLeftY(){
        return getRawAxis(Axis.leftY.value); 
    }

    public double getRightX(){
        return getRawAxis(Axis.rightX.value); 
    }

    public double getRightY(){
        return getRawAxis(Axis.rightY.value); 
    }

    public double getLeftTriggerAxis(){
        return getRawAxis(Axis.leftTrigger.value); 
    }

    public Trigger leftTrigger(EventLoop loop, double threshold){
        BooleanEvent event = axisGreaterThan(Axis.leftTrigger.value, threshold, loop);  
        return event.castTo(Trigger::new);  
    }

    public Trigger lefTrigger(EventLoop loop){
        BooleanEvent event = axisGreaterThan(Axis.leftTrigger.value, 0.5, loop);
        return event.castTo(Trigger::new); 
    }

    public double getRightTriggerAxis(){
        return getRawAxis(Axis.rightTrigger.value); 
    }

    //Buttons

    public boolean getAButton(){
        return getRawButton(Buttons.kA.value); 
    }

    public boolean getAButtonPressed(){
        return getRawButtonPressed(Buttons.kA.value); 
    }

    public boolean getAButtonReleased(){
        return getRawButtonReleased(Buttons.kA.value); 
    }


    public boolean getBButton(){
        return getRawButton(Buttons.kB.value); 
    }

    public boolean getBButtonPressed(){
        return getRawButtonPressed(Buttons.kB.value); 
    }

    public boolean getBButtonReleased(){
        return getRawButtonReleased(Buttons.kB.value); 
    }

    public boolean getXButton(){
        return getRawButton(Buttons.kX.value); 
    }

    public boolean getXButtonPressed(){
        return getRawButtonPressed(Buttons.kX.value); 
    }

    public boolean getXButtonReleased(){
        return getRawButtonReleased(Buttons.kX.value); 
    }


    public boolean getYButton(){
        return getRawButton(Buttons.kY.value); 
    }

    public boolean getYButtonPressed(){
        return getRawButtonPressed(Buttons.kY.value); 
    }

    public boolean getYButtonReleased(){
        return getRawButtonReleased(Buttons.kY.value); 
    }


    public boolean getLeftBumperButton(){
        return getRawButton(Buttons.leftBumper.value); 
    }

    public boolean getLeftBumperButtonPressed(){
        return getRawButtonPressed(Buttons.leftBumper.value); 
    }

    public boolean getLeftBumperButtonReleased(){
        return getRawButtonReleased(Buttons.leftBumper.value); 
    }


    public boolean getRightBumperButton(){
        return getRawButton(Buttons.rightBumper.value); 
    }

    public boolean getRightBumperButtonPressed(){
        return getRawButtonPressed(Buttons.rightBumper.value); 
    }

    public boolean getRightBumperButtonReleased(){
        return getRawButtonReleased(Buttons.rightBumper.value); 
    }


    public boolean getBackButton(){
        return getRawButton(Buttons.backButton.value); 
    }

    public boolean getBackButtonPressed(){
        return getRawButtonPressed(Buttons.backButton.value); 
    }

    public boolean getBackButtonReleased(){
        return getRawButtonReleased(Buttons.backButton.value); 
    }


    public boolean getStartButton(){
        return getRawButton(Buttons.startButton.value); 
    }

    public boolean getStartButtonPressed(){
        return getRawButtonPressed(Buttons.startButton.value); 
    }

    public boolean getStartButtonReleased(){
        return getRawButtonReleased(Buttons.startButton.value); 
    }


    public boolean getLeftStickButton(){
        return getRawButton(Buttons.leftStick.value); 
    }

    public boolean getLeftStickButtonPressed(){
        return getRawButtonPressed(Buttons.leftStick.value); 
    }

    public boolean getLeftStickButtonReleased(){
        return getRawButtonReleased(Buttons.leftStick.value); 
    }


    public boolean getRightStickButton(){
        return getRawButton(Buttons.riightStick.value); 
    }

    public boolean getRightStickButtonPressed(){
        return getRawButtonPressed(Buttons.riightStick.value); 
    }

    public boolean getRightStickButtonReleased(){
        return getRawButtonReleased(Buttons.riightStick.value); 
    }

}
