package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TeleOpMode_V1_8")
public class TeleOpMode_V1_8 extends LinearOpMode {

  double drive, strafe, turn = 0.000;

  String cameraNameFront = "Webcam_front";
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

  final String LOCATION_BACKSTAGE = "Backstage";
  final String LOCATION_AUDIENCE = "Audience";
  String currentLocation = LOCATION_BACKSTAGE;

  // April Tag detection and drive to variables
  int desiredTagID = 0;
  boolean targetTagFound = false;
  int desiredTagDistance = 10;  // Inches

  //  Effort factors that are divided into the requested drive, strafe or turn requests
  public static final double SCALEFACTOR_SPEEDHIGH = 2.0;
  public static final double SCALEFACTOR_SPEEDLOW = 4.0;
  public static final double SCALEFACTOR_STRAFEHIGH = 2.0;
  public static final double SCALEFACTOR_STRAFELOW = 4.0;
  public static final double SCALEFACTOR_TURNHIGH = 3.0;
  public static final double SCALEFACTOR_TURNLOW= 4.0;
  // Default scale factors for our drivetrain movements
  public static double scalefactorSpeed = SCALEFACTOR_SPEEDHIGH;
  public static double scalefactorStrafe = SCALEFACTOR_STRAFEHIGH;
  public static double scalefactorTurn = SCALEFACTOR_TURNHIGH;

  DrivetrainMecanumWithSmarts ourDrivetrain = new DrivetrainMecanumWithSmarts();
  Vision ourVision = new Vision();
  Arm ourArm = new Arm();
  ActiveIntakeWithServo ourActiveIntake = new ActiveIntakeWithServo();
  Winch ourWinch = new Winch();
  
  
  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    // Initialize vision libraries
    useVision = ourVision.initVision2Cameras(cameraNameFront, cameraNameBack);
    if (useVision) ourVision.disableTFOD();

    runtime = new ElapsedTime();

    // Initialize Drivetrain
    ourDrivetrain.initDriveTrainWithSmarts("left_front_drive", "left_back_drive", "right_front_drive", "right_back_drive", "distance_left_front");

    // Set drive to A Config.
    ourDrivetrain.setDriveToAConfig();

    // Initialize Arm and Poses
    ourArm.initArm("motor_shoulder", "motor_elbow", "motor_wrist");

    // Initialize gripper
    ourActiveIntake.init("motor_intake");

    // Initialize winch
    ourWinch.init("motor_winch");

    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Initialized");


    while (opModeInInit() && !gamepad1.back) {

      // Using the X,Y,A or B on gamepad 1 to designate location and Alliance
      getLocationPlusAlliance();

      telemetry.addData("ALLIANCE [1=Red, 2=Blue, 0=Unset]", currentAlliance);
      telemetry.addData("LOCATION", currentLocation);
      telemetry.addData("USE VISION", useVision);
      telemetry.addData("Preview Vision Detection", "3 dots, Camera Stream");
      telemetry.addData(">", "Touch [Gamepad 1 Start] when all setup is done");

      telemetry.addData("Select Location [Gamepad 1]: ", "[A=Audience, B=Backstage]");
      telemetry.addData("Select Alliance [Gamepad 1]: ", "[X=Blue Alliance, Y=Red Alliance]");

      telemetry.update();
    }

    currentAlliance = ourVision.setAlliance(currentAlliance);

    telemetry.addData("ALLIANCE [1=Red, 2=Blue, 0=Unset]", currentAlliance);
    telemetry.addData("LOCATION", currentLocation);
    telemetry.addData("USE VISION", useVision);
    telemetry.addData("Preview Vision Detection", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch [Gamepad 1 Start] when all setup is done");

    telemetry.update();
    waitForStart();

    runtime.reset();
    
    // Run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      targetTagFound = false;
      desiredTagID = 0;

      // Runs arm based if on a state.
      ourArm.runArmIteration();

      // By gamepad1 left bumper / trigger
      IfAskedToggleSpeedOrCameraDriveConfig();

      // Drive requests are by Stick or to Tags with DPad
      IfRequestingToDrive();

      // By gamepad1 right bumper / trigger
      //IfAskedEjectLowerPixel();

      // By gamepad2 XYAB
      IfAskedPose();

      // Move wrist (left bumper / trigger) and elbow (right bumper / trigger)
      IfAskedMoveArmByIncrement();

      // By gampad2 DPAD
      IfAskedDoIntake();

      // By gamepad2 BACK
      IfAskedToLoosenArm();

      // By gamepad2 leftstick.Y
      IfAskedToUseWinch();

      telemetry.addData("ALLIANCE [1=Red, 2=Blue, 0=Unset]", currentAlliance);
      telemetry.addData("LOCATION", currentLocation);
      telemetry.addData("DESIRED TAG ID", desiredTagID);
      telemetry.addData("Target Found", targetTagFound);

      addGeneralTelemetry();

      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedPose() {
    if (gamepad2.y) {
      telemetry.addLine("FUNCTION: Move to Pose UP");
      // Moves the arm to an UP pose.
      ourArm.moveToPose_UP();
    } else if (gamepad2.a) {
      telemetry.addLine("FUNCTION: Move to Pose FLOOR");
      // Moves the arm to an FLOOR pose.
      ourArm.moveToPose_FLOOR();
    } else if (gamepad2.b) {
      telemetry.addLine("FUNCTION: Move to Pose TRAVEL");
      // Moves the arm to an TRAVEL pose.
      ourArm.moveToPose_TRAVEL();
    } else if (gamepad2.x) {
      telemetry.addLine("FUNCTION: Move to Pose BACKDROP");
      // Moves the arm to an BACKDROP pose.
      ourArm.moveToPose_BACKDROP();
    }
  }

  private void IfAskedMoveArmByIncrement() {
    double threshold = 0.2;

    if (gamepad2.left_bumper) {
      ourArm.moveWristByIncrement(0.003);
    } else if (gamepad2.left_trigger != 0) {
      ourArm.moveWristByIncrement(-0.003);
    }
    if (gamepad2.right_bumper) {
      ourArm.moveElbowByIncrement(0.003);
    } else if (gamepad2.right_trigger != 0) {
      ourArm.moveElbowByIncrement(-0.003);
    }
    if (gamepad2.right_stick_y < 0 && abs(gamepad2.right_stick_y) > threshold) {// Increase
      ourArm.moveShoulderByIncrement(0.001);
    } else if (gamepad2.right_stick_y > 0 && abs(gamepad2.right_stick_y) > threshold) {// Decrease
      ourArm.moveShoulderByIncrement(-0.001);
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedDoIntake() {
    if (gamepad2.dpad_down) {
      // Moves intake inward
      ourActiveIntake.moveInward();
    } else if (gamepad2.dpad_up) {
      // Stops the intake
      ourActiveIntake.moveOutward();
    } else  {
      // Moves intake outward
      ourActiveIntake.stop();
    }
  }

  public void IfAskedToUseWinch() {
    double threshold = 0.2;

    if (gamepad2.left_stick_y < 0 && abs(gamepad2.left_stick_y) > threshold) // Move up
      ourWinch.moveInward();
    else if (gamepad2.left_stick_y > 0 && abs(gamepad2.left_stick_y) > threshold) // Move down
      ourWinch.moveOutward();
    else
      ourWinch.stop();
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
        ourVision.doCameraSwitching(cameraNameBack);
        currentCameraName = cameraNameBack;

        // Set drive to B Config.
        ourDrivetrain.setDriveToBConfig();
        currentDriveConfig = driveConfigB;

      } else {

        // Changing to Front camera
        // Switches the based on input.
        ourVision.doCameraSwitching(cameraNameFront);
        currentCameraName = cameraNameFront;

        // Set drive to A Config.
        ourDrivetrain.setDriveToAConfig();
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
      final double MAX_AUTO_SPEED = 0.3;
      final double MAX_AUTO_STRAFE = 0.2;
      final double MAX_AUTO_TURN = 0.2;

      if (gamepad1.dpad_down) relativeTargetTag = 4;
      if (gamepad1.dpad_left) relativeTargetTag = 1;
      if (gamepad1.dpad_up) relativeTargetTag = 2;
      if (gamepad1.dpad_right) relativeTargetTag = 3;

      telemetry.addData("RELATIVE TAG REQUESTED", relativeTargetTag);

      // Let's determine the AprilTag target
      desiredTagID = ourVision.getCENTERSTAGEDesiredTag(currentAlliance, relativeTargetTag);

      telemetry.addData("AUTO: Relative Target Tag requested", relativeTargetTag);
      telemetry.addData("AUTO: Desired Tag ID", desiredTagID);

      if (desiredTagID == 0) return;

      // Determines and sets latest values for desired AprilTag
      targetTagFound = ourVision.getTagInfo(desiredTagID);

      telemetry.addData("AUTO: Target Found", targetTagFound);

      if (targetTagFound) {

        // Determine range, heading and yaw (tag image rotation) so we can use them to
        // control the robot automatically.
        double range = ourVision.getTagRange(desiredTagID);
        double heading = ourVision.getTagBearing(desiredTagID);
        double yaw = ourVision.getTagYaw(desiredTagID);

        telemetry.addData("TAG: range, heading, yaw", JavaUtil.formatNumber(range, 4, 2) + ", " + JavaUtil.formatNumber(heading, 4, 2) + ", " + JavaUtil.formatNumber(yaw, 4, 2));

        ourDrivetrain.driveToHeading(desiredTagDistance, range, heading, yaw);
        
      } else {

        return;
      }

    } else {

      // Drive using manual POV Joystick mode.  Slow things down to make the robot more controllable.
      drive = -gamepad1.left_stick_y / scalefactorSpeed;
      strafe = -gamepad1.left_stick_x / scalefactorStrafe;
      turn = gamepad1.right_stick_x / scalefactorTurn;

      telemetry.addData("MANUAL DRIVE", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

      // Apply desired axes motions to the ourDrivetrain.
      ourDrivetrain.driveXYZ(drive, strafe, turn);
    }

    sleep(10);
  }

  private void IfAskedToLoosenArm() {
    if (gamepad2.back) ourArm.loosenArmJoints();
  }

  /**
   * Describe this function...
   */
  private void addGeneralTelemetry() {
    telemetry.addData("ALLIANCE", currentAlliance);
    telemetry.addData("Status", "Run Time: " + runtime);
  }



  /**
   * Provides a way during Init mode to identify starting location and alliance with gamepad 1
   */
  private void getLocationPlusAlliance() {
    if (gamepad1.a) {
      currentLocation = LOCATION_AUDIENCE;
    } else if (gamepad1.b) {
      currentLocation = LOCATION_BACKSTAGE;
    } else if (gamepad1.x) {
      currentAlliance = BLUEALLIANCE;
    } else if (gamepad1.y) {
      currentAlliance = REDALLIANCE;
    }

  }

}
