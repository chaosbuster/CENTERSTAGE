package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class ActiveIntakeWithServo extends BlocksOpModeCompanion {
    
    // Define features of the active intake that is using a continuous servo
    static final private int STATE_MOVINGIN = 1;
    static final private int STATE_MOVINGOUT = -1;
    static final private int STATE_STOPPED = 0;
    static int stateOfIntake = STATE_STOPPED;
    static final double SPEED_TOMOVE = 10;
    static final double SPEED_STOP = 0.5;
    static private String servoName = null;
    static private Servo servo    = null;
    static private double currentSpeed  = SPEED_STOP;



    @ExportToBlocks(
            heading = "Intake: Initialize",
            color = 255,
            comment = "Initialize active intake",
            tooltip = "The motor should be a servo.",
            parameterLabels = {"Intake Servo Name"
            }
    )
    /** Initialize active intake
     */
    // Initializes the active intake and sets it to stopped
    public static void init(String _servoName) {

        servoName = _servoName;

        // Connect to servo
        servo = hardwareMap.get(Servo.class, servoName);

        // Set the servo position the stop speed
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(SPEED_STOP);

        currentSpeed  = SPEED_STOP;
        stateOfIntake = STATE_STOPPED;
    }


    @ExportToBlocks(
            heading = "Intake: Eject a game element",
            color = 255,
            comment = "Will move intake outward for a certain amount of seconds",
            tooltip = "Right now the time is in the method."
    )
    /** Initialize active intake
     */   // Ejects a game object
    public static void ejectOneGameObject() {

        moveOutward();
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        stop();
    }

    @ExportToBlocks(
            heading = "Intake: Stop motion",
            color = 255,
            comment = "Will stop movement of intake",
            tooltip = "Do you really want it to stop?"
    )
    // Stops running servo
    public static void stop() {
        servo.setPosition(SPEED_STOP);
        stateOfIntake = STATE_STOPPED;
        currentSpeed = SPEED_STOP;
    }

    @ExportToBlocks(
            heading = "Intake: Starts moving inward",
            color = 255,
            comment = "Will start moving intake inward",
            tooltip = "Do not need to continuously send this command."
    )
    // Starts running servo inward
    public static void moveInward() {

        currentSpeed = servo.getPosition();
        if (stateOfIntake != STATE_MOVINGIN) {
            servo.setPosition(STATE_STOPPED);            
            servo.setDirection(Servo.Direction.REVERSE);
            stateOfIntake = STATE_MOVINGIN;
        }
        servo.setPosition(SPEED_TOMOVE);
    }

    // Starts (or continues) running servo outward
    @ExportToBlocks(
            heading = "Intake: Starts moving outward",
            color = 255,
            comment = "Will start moving intake outward",
            tooltip = "Do not need to continuously send this command."
    )
    public static void moveOutward() {

        currentSpeed = servo.getPosition();
        if (stateOfIntake != STATE_MOVINGOUT) {
            servo.setPosition(STATE_STOPPED);
            servo.setDirection(Servo.Direction.FORWARD);
            stateOfIntake = STATE_MOVINGOUT;
        }
        servo.setPosition(SPEED_TOMOVE);
    }

    // Returns the name of the joint
    public static String getName() {
        return servoName;
    }
    
    // Loosens the joint
    public static void loosen() {
        // Command below disabled all servos, but all but one were set back to a position after
        // one servo was set to a position:  servo.getController().pwmDisable();

        // Let's loosen this specific servo.
        if (servo instanceof PwmControl)
            ((PwmControl) servo).setPwmDisable();
    }

    
}