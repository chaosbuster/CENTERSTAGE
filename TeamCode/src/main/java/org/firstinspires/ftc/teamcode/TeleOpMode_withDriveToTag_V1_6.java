package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.Arrays;

@TeleOp(name = "TeleOpMode_withDriveToTag_V1_6 (Blocks to Java)")
public class TeleOpMode_withDriveToTag_V1_6 extends LinearOpMode {

  private Servo motor_dropPixels;
  double drive, strafe, turn = 0.000;
  int stateMovingIn;
  int stateMovingOut;
  int stateStopped;
  int stateOfIntake;
  double dropPixel_stop;
  int dropPixel_speed;

  final String cameraNameFront = "Webcam_front";
  String cameraNameBack = "Webcam_back";
  String currentCameraName = cameraNameFront;
  boolean useVision = false;

  final int driveConfigA = 0;
  final int driveConfigB = 1;
  int currentDriveConfig = driveConfigA;
  ElapsedTime runtime;

  final int REDALLIANCE = 1;
  final int BLUEALLIANCE = 2;
  int currentAlliance = BLUEALLIANCE;

  // April Tag detection and drive to variables
  int desiredTagID = 0;
  boolean targetTagFound = false;
  int desiredTagDistance = 10;  // Inches

  //  Effort factors that are divided into the requested drive, strafe or turn requests
  final double SCALEFACTOR_SPEEDHIGH = 2.0;
  final double SCALEFACTOR_SPEEDLOW = 4.0;
  final double SCALEFACTOR_STRAFEHIGH = 2.0;
  final double SCALEFACTOR_STRAFELOW = 4.0;
  final double SCALEFACTOR_TURNHIGH = 3.0;
  final double SCALEFACTOR_TURNLOW= 4.0;
  // Default scale factors for our drivetrain movements
  double scalefactorSpeed = SCALEFACTOR_SPEEDHIGH;
  double scalefactorStrafe = SCALEFACTOR_STRAFEHIGH;
  double scalefactorTurn = SCALEFACTOR_TURNHIGH;


  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    motor_dropPixels = hardwareMap.get(Servo.class, "motor_dropPixels");
    motor_dropPixels.setDirection(Servo.Direction.FORWARD);
    initPixelLowerEject();

    // Initialize vision libraries
    useVision = Vision.initVision2Cameras("Webcam_front", "Webcam_back", "bp_253_ssd_v2_fpnlite_320x320_metadata.tflite", Arrays.asList("Bolt"));

    runtime = new ElapsedTime();

    // Initialize Drivetrain
    DrivetrainMecanumWithSmarts.initDriveTrainWithSmarts("left_front_drive", "left_back_drive", "right_front_drive", "right_back_drive", "distance_left_front");

    // Set drive to A Config.
    DrivetrainMecanumWithSmarts.setDriveToAConfig();

    // Initialize Arm and Poses
    Arm.initArm("motor_shoulder", "motor_elbow", "motor_wrist");

    // Initialize gripper
    Gripper.init("motor_gripper", 1, 0.2, 0.5, 1);

    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();

    runtime.reset();
    
    // Run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      targetTagFound = false;
      desiredTagID = 0;

      // Runs arm based if on a state.
      Arm.runArmIteration();

      // By gamepad1 left bumper / trigger
      IfAskedToggleSpeedOrCameraDriveConfig();
      // Drive requests are by Stick or to Tags with DPad
      IfRequestingToDrive();

      // By gamepad1 right bumper / trigger
      IfAskedEjectLowerPixel();

      // By gamepad2 XYAB
      IfAskedPose();

      // By gampad2 DPAD
      IfAskedDoGripper();

      addGeneralTelemetry();

      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void initPixelLowerEject() {
    stateMovingIn = 1;
    stateMovingOut = -1;
    stateStopped = 0;
    stateOfIntake = stateMovingIn;
    dropPixel_speed = 10;
    dropPixel_stop = 0.5;
    motor_dropPixels.setPosition(dropPixel_stop);
  }

  /**
   * Describe this function...
   */
  private void IfAskedPose() {
    if (gamepad2.x) {
      telemetry.addLine("FUNCTION: Move to Pose X");
      // Moves the arm to an UP pose.
      Arm.moveToPose_UP();
    } else if (gamepad2.a) {
      telemetry.addLine("FUNCTION: Move to Pose FLOOR");
      // Moves the arm to an FLOOR pose.
      Arm.moveToPose_FLOOR();
    } else if (gamepad2.b) {
      telemetry.addLine("FUNCTION: Move to Pose TRAVEL");
      // Moves the arm to an TRAVEL pose.
      Arm.moveToPose_TRAVEL();
    } else if (gamepad2.y) {
      telemetry.addLine("FUNCTION: Move to Pose BACKDROP");
      // Moves the arm to an BACKDROP pose.
      Arm.moveToPose_BACKDROP();
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedDoGripper() {
    if (gamepad2.dpad_left) {
      // Opens gripper
      Gripper.GripNone();
    } else if (gamepad2.dpad_down) {
      // Grips one only
      Gripper.GripOne();
    } else if (gamepad2.dpad_right) {
      // Tightens gripper to hold two pixels
      Gripper.GripTwo();
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedEjectLowerPixel() {
    if (gamepad1.right_bumper) {
      if (stateOfIntake != stateMovingIn) {
        motor_dropPixels.setDirection(Servo.Direction.FORWARD);
        stateOfIntake = stateMovingIn;
      }
      motor_dropPixels.setPosition(dropPixel_speed);
    } else if (gamepad1.right_trigger != 0) {
      if (stateOfIntake != stateMovingOut) {
        motor_dropPixels.setDirection(Servo.Direction.REVERSE);
        stateOfIntake = stateMovingOut;
      }
      motor_dropPixels.setPosition(dropPixel_speed);
    } else {
      motor_dropPixels.setPosition(dropPixel_stop);
      stateOfIntake = stateStopped;
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedToggleSpeedOrCameraDriveConfig() {
    if (gamepad1.left_bumper == true) {
      // Toggle Speed
      if (scalefactorSpeed == SCALEFACTOR_SPEEDHIGH) {
        // Changing to low
        scalefactorSpeed = SCALEFACTOR_SPEEDLOW;
        scalefactorStrafe = SCALEFACTOR_STRAFELOW;
        scalefactorTurn = SCALEFACTOR_TURNLOW;
        
      } else {
        // Changing to high
        scalefactorSpeed = SCALEFACTOR_SPEEDHIGH;
        scalefactorStrafe = SCALEFACTOR_STRAFEHIGH;
        scalefactorTurn = SCALEFACTOR_TURNHIGH;        
      }
    } else if (gamepad1.left_trigger != 0) {
      // Toggle Camera & Drive Configuration
      if (currentCameraName.equals(cameraNameFront)) {

        // Changing to Back camera.  Switches based on input.
        Vision.doCameraSwitching(cameraNameBack);
        currentCameraName = cameraNameBack;

        // Set drive to B Config.
        DrivetrainMecanumWithSmarts.setDriveToBConfig();
        currentDriveConfig = driveConfigB;

      } else {

        // Changing to Front camera
        // Switches the based on input.
        Vision.doCameraSwitching(cameraNameFront);
        currentCameraName = cameraNameFront;

        // Set drive to A Config.
        DrivetrainMecanumWithSmarts.setDriveToAConfig();
        currentDriveConfig = driveConfigA;
      }
    }
  }

  /**
   * Describe this function...
   */
  private void IfRequestingToDrive() {
    int relativeTargetTag = 0;

    if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
      final double SPEED_GAIN = 0.02;
      final double STRAFE_GAIN = 0.015;
      final double TURN_GAIN = 0.01;
      final double MAX_AUTO_SPEED = 0.05;
      final double MAX_AUTO_STRAFE = 0.05;
      final double MAX_AUTO_TURN = 0.03;

      if (gamepad1.dpad_down) relativeTargetTag = 4;
      if (gamepad1.dpad_left) relativeTargetTag = 1;
      if (gamepad1.dpad_up) relativeTargetTag = 2;
      if (gamepad1.dpad_right) relativeTargetTag = 3;

      telemetry.addData("RELATIVE TAG REQUESTED", relativeTargetTag);

      // Let's determine the AprilTag target
      desiredTagID = Vision.getCENTERSTAGEDesiredTag(currentAlliance, relativeTargetTag);

      if (desiredTagID == 0) return;

      // Determines and sets latest values for desired AprilTag
      targetTagFound = Vision.getTagInfo(desiredTagID);

      if (targetTagFound) {

        // Determine range, heading and yaw (tag image rotation) error so we can use them to
        // control the robot automatically.
        double rangeError = (Vision.getTagRange(desiredTagID) - desiredTagDistance);
        double headingError = Vision.getTagBearing(desiredTagID);
        double yawError = Vision.getTagYaw(desiredTagID);

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("AUTO: DRIVE, STRAFE, TURN", JavaUtil.formatNumber(drive, 4, 2) + ", " + JavaUtil.formatNumber(strafe, 4, 2) + ", " + JavaUtil.formatNumber(turn, 4, 2));

      } else {
        return;
      }

    } else {

      // Drive using manual POV Joystick mode.  Slow things down to make the robot more controllable.
      drive = -gamepad1.left_stick_y / scalefactorSpeed;
      strafe = -gamepad1.left_stick_x / scalefactorStrafe;
      turn = -gamepad1.right_stick_x / scalefactorTurn;

      telemetry.addData("MANUAL DRIVE", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
    }

    // Apply desired axes motions to the drivetrain.
    DrivetrainMecanumWithSmarts.driveXYZ(drive, strafe, turn);

    sleep(10);
  }



  /**
   * Describe this function...
   */
  private void addGeneralTelemetry() {
    telemetry.addData("ALLIANCE", currentAlliance);
    telemetry.addData("Status", "Run Time: " + runtime);
  }

}
