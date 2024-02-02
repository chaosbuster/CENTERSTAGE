package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "Auto_v1_5")
public class Auto_V1_5 extends LinearOpMode {
  
  final int REDALLIANCE = 1;
  final int BLUEALLIANCE = 2;
  int currentAlliance = BLUEALLIANCE;

  final String LOCATION_BACKSTAGE = "Backstage";
  final String LOCATION_AUDIENCE = "Audience";
  String currentLocation = LOCATION_BACKSTAGE;

  boolean useVision = false;
  // Using pixels on image to determine which spike mark the object is on
  String camera_back = "Webcam_back";
  int MINX_FORSPIKE_RIGHT = 300;  // X pixels on image where left of X is Center Spike, right of X is Right Spike
  final int SPIKE_LEFT = 1;
  final int SPIKE_CENTER = 2;
  final int SPIKE_RIGHT = 3;
  int targetSpike = SPIKE_LEFT; // Default to left spike
  
  DrivetrainMecanumWithSmarts ourDrivetrain = null;
  Vision ourVision = null;
  Arm ourArm = null;
  ActiveIntakeWithServo ourActiveIntake = null;

  
  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    ourDrivetrain = new DrivetrainMecanumWithSmarts();
    ourVision = new Vision();
    ourArm = new Arm();
    ourActiveIntake = new ActiveIntakeWithServo();

    // Initialize Drivetrain
    ourDrivetrain.initDriveTrainWithSmarts("left_front_drive", "left_back_drive", "right_front_drive", "right_back_drive", "distance_left_front");

    // Initialize Arm and Poses
    ourArm.initArm("motor_shoulder", "motor_elbow", "motor_wrist");

    // Initialize gripper
    ourActiveIntake.init("motor_intake");

    // Initialize our vision
    useVision = ourVision.initVision2Cameras("Webcam_front", "Webcam_back");

    telemetry.addData("ALLIANCE [1=Red, 2=Blue, 0=Unset", currentAlliance);
    telemetry.addData("LOCATION", currentLocation);
    telemetry.addData("TARGET SPIKE (1=Left, 2=Center, 3=Right)", targetSpike);
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();


    while (opModeInInit() && !gamepad1.back) {
      if (useVision) {
        // Indicates whether the Team Prop is in sight.
        targetSpike = ourVision.getTeamPropLocation("Bolt", MINX_FORSPIKE_RIGHT);
      } else {
        targetSpike = SPIKE_LEFT;
      }

      // Using the X,Y,A or B on gamepad 1 to designate robot Location and Alliance
      getLocationPlusAlliance();
      // Using DPAD on gamepad 1 to move the intake
      IfAskedDoIntake();

      telemetry.addData("ALLIANCE [1=Red, 2=Blue, 0=Unset", currentAlliance);
      telemetry.addData("LOCATION", currentLocation);
      telemetry.addData("USE VISION", useVision);
      telemetry.addData("TARGET SPIKE (1=Left, 2=Center, 3=Right)", targetSpike);
      telemetry.addData("Preview Vision Detection", "3 dots, Camera Stream");
      telemetry.addData(">", "Touch [Gamepad 1 Start] when all setup is done");
      
      telemetry.addData("Select Location [Gamepad 1]: ", "[A=Audience, B=Backstage]");
      telemetry.addData("Select Alliance [Gamepad 1]: ", "[X=Blue Alliance, Y=Red Alliance]");
      telemetry.addData("Load purple pixel [Gamepad 1]: ", "[Right Bumper-In, Right Trigger-Out]");
      telemetry.addData("Load yellow pixel [Gamepad 2 DPAD]: ", "[LEFT-None, DOWN-One, RIGHT-Two]");
      telemetry.update();
    }

    currentAlliance = ourVision.setAlliance(currentAlliance);

    while (opModeInInit()) {
      if (useVision) {
        // Indicates whether the Team Prop is in sight.
        targetSpike = ourVision.getTeamPropLocation("Bolt", MINX_FORSPIKE_RIGHT);
      } else {
        targetSpike = SPIKE_LEFT;
      }
      telemetry.addData("ALLIANCE [1=Red, 2=Blue, 0=Unset", currentAlliance);
      telemetry.addData("LOCATION", currentLocation);
      telemetry.addData("USE VISION", useVision);
      telemetry.addData("TARGET SPIKE (1=Left, 2=Center, 3=Right)", targetSpike);
      telemetry.update();
    }

    waitForStart();

    if (opModeIsActive()) {
      telemetry.addData("ALLIANCE [1=Red, 2=Blue, 0=Unset", currentAlliance);
      telemetry.addData("LOCATION", currentLocation);
      telemetry.addData("USE VISION", useVision);
      telemetry.addData("TARGET SPIKE (1=Left, 2=Center, 3=Right)", targetSpike);
      telemetry.update();

      // Should already have our target spike location so disabling TFOD process to save CPU
      if (useVision) ourVision.disableTFOD();

      // Make sure our drivetrain and arm are waiting for a command
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand() && ourArm.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
        // Runs arm based on the state.
        ourArm.runArmIteration();
        telemetry.update();
      }

      // Ready to perform FIRST GOAL of placing purple pixel on correct spike mark
      double speedForSpikeTasks = 0.3;  // speed to use for all spike tasks
      double distanceToSpikeTile = 16.0;  // Distance (inches) to the middle of the Spike tile
      double initialHeading = 0; // Heading of the robot when it first starts up

      // Yaw: Clockwise to right spike or counterclockwise to left spike
      double yawToRightSpike_BBRA = -50.0;  // Heading for right spike location in BB & RA location
      double yawToLeftSpike_BBRA = 50.0;  // Heading for left spike location in BB & RA location
      if ((currentLocation == LOCATION_BACKSTAGE && currentAlliance == BLUEALLIANCE) ||
          (currentLocation == LOCATION_AUDIENCE && currentAlliance == REDALLIANCE)) {

        moveAndDropPurplePixel(initialHeading, speedForSpikeTasks, distanceToSpikeTile,
                               yawToLeftSpike_BBRA, yawToRightSpike_BBRA);

      } else {

        // NOTE: The left and right yaw values will be negative from other two starting location
        moveAndDropPurplePixel(initialHeading, speedForSpikeTasks, distanceToSpikeTile,
                               -yawToLeftSpike_BBRA, -yawToRightSpike_BBRA);

      }

      // Prepare bot to avoid purple pixel but "see" the backdrop
      double speedForMoveTasks = 0.3;  // speed to use for all move tasks
      double distanceToStrafe = 5.0;  // Distance (inches) to avoid purple pixel close to backdrop
      // Yaw: Clockwise (-) to right spike or counterclockwise (+) to left spike
      double yawToBackdropFarBB = -90.0;  // Heading where back is facing Backdrop in BB location

      if (currentLocation == LOCATION_BACKSTAGE) {
        if (currentAlliance == BLUEALLIANCE) {
          moveToSeeBackdropAndAvoidPurplePixel(speedForMoveTasks, distanceToStrafe, yawToLeftSpike_BBRA, yawToBackdropFarBB);
        } else {
          // NOTE: Some of the values for RED ALLIANCE will be negative of BLUE BACKSTAGE starting location
          moveToSeeBackdropAndAvoidPurplePixel(speedForMoveTasks, -distanceToStrafe, -yawToLeftSpike_BBRA, -yawToBackdropFarBB);
        }
      }

      // Ready to perform second goal of scoring yellow pixel above correct AprilTag
      // moveToBackdropAndDropYellowPixel()

      // Ready to perform last goal of parking

    }
  }


  /**
   * Provides a way to identify starting location and alliance with gamepad 1
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
    ourVision.setAlliance(currentAlliance);
  }

  /**
   * FIRST GOAL: Move and drop the purple pixel on the correct spike location
   */
  private void moveAndDropPurplePixel(double _initialHeading, double _speed, double _distanceToSpikeTile,
                double _yawToLeftSpike, double _yawToRightSpike) {

    // Let's move to the middle of the spike tile
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      ourDrivetrain.driveStraight(_speed, _distanceToSpikeTile, _initialHeading);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
        telemetry.update();
      }
    }

    // Go to FLOORDROP pose
    if (opModeIsActive() && ourArm.waitingForCommand()) {
      ourArm.moveToPose_FLOORDROP();
      // Iterates on arm until it has moved to our requested pose
      while (opModeIsActive() && !ourArm.waitingForCommand()) {
        // Runs arm based on the state.
        ourArm.runArmIteration();
        telemetry.update();
      }
      sleep(1000);
    }

    // Turn to left or right spike if necessary
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      if (targetSpike == SPIKE_RIGHT) {
        ourDrivetrain.turnToHeading(_speed, _yawToRightSpike);
      } else if (targetSpike == SPIKE_LEFT) {
        ourDrivetrain.turnToHeading(_speed, _yawToLeftSpike);
      }
      // Iterates on drivetrain until it has moved to our requested heading
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on our request
        ourDrivetrain.runDrivetrainIteration();
      }
    }

    // ***** EJECT OUR PURPLE PIXEL *****
    if (opModeIsActive()) {
      ourActiveIntake.ejectOneGameObject();
    }

  }  // moveAndDropPurplePixel()

  /**
   *  PREPARE: Based on spike location move to see Backdrop
   *  LOCATION: BACKSTAGE
   */
  private void moveToSeeBackdropAndAvoidPurplePixel(double _speed, double _distanceToStrafe, double _headingOfStrafe, double _yawToBackdropFarSpikes) {

    // Go to TRAVEL pose
    if (opModeIsActive() && ourArm.waitingForCommand()) {
      ourArm.moveToPose_TRAVEL();
      // Iterates on arm until it has moved to our requested pose
      while (opModeIsActive() && !ourArm.waitingForCommand()) {
        // Runs arm based on the state.
        ourArm.runArmIteration();
        telemetry.update();
      }
      sleep(1000);
    }


    // Let's move to avoid purple pixel for spike mark closest to Backdrop
    if ((currentAlliance == BLUEALLIANCE && targetSpike == SPIKE_LEFT) || (currentAlliance == REDALLIANCE && targetSpike == SPIKE_RIGHT)) {

      if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
        // Drives Straight either Forward or Reverse.
        ourDrivetrain.driveLeft(_speed, _distanceToStrafe, _headingOfStrafe);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
          // Runs drivetrain based on the state.
          ourDrivetrain.runDrivetrainIteration();
          telemetry.update();
        }
      }
    } else {

      // Otherwise turn to "see" Backdrop for farthest spike locations
      if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {

        ourDrivetrain.turnToHeading(_speed, _yawToBackdropFarSpikes);

        // Iterates on drivetrain until it has moved to our requested heading
        while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
          // Runs drivetrain based on our request
          ourDrivetrain.runDrivetrainIteration();
        }
      }

    }

  }  // moveToSeeBackdropAndAvoidPurplePixel()

  /**
   * SECOND GOAL: Move to Backdrop using targeted AprilTag and drop Yellow Pixel
   */
  private void moveToBackdropAndDropYellowPixel() {
    double speedForMoveTasks = 0.3;  // speed to use for all move tasks
    // Yaw: Clockwise (-) to right spike or counterclockwise (+) to left spike
    double yawToBackdropBLUE = -90.0;  // Heading where back is facing Backdrop when BLUE
    double yawToBackdropRED = 90.0;  // Heading where back is facing Backdrop when RED
    double desiredTagDistance = 10.0; // Distance (inches) from tag from which to stop
    double range = 0;
    double heading =0;
    double yaw = 0;

    ourDrivetrain.setDriveToBConfig();
    ourVision.doCameraSwitching(camera_back);

    // Let's determine the AprilTag target
    // NOTE: currentAlliance is set by GamePad 1 during INIT
    int desiredTagID = ourVision.getCENTERSTAGEDesiredTag(currentAlliance, targetSpike);

    telemetry.addData("AUTO: Relative Target Tag requested", targetSpike);
    telemetry.addData("AUTO: Desired Tag ID", desiredTagID);

    if (desiredTagID == 0) return;

    // Determines and sets latest values for desired AprilTag
    boolean targetTagFound = ourVision.getTagInfo(desiredTagID);

    // Determine range, heading and yaw (tag image rotation) so we can use them to
    // control the robot automatically.
    if (targetTagFound) {
      range = ourVision.getTagRange(desiredTagID);
      heading = ourVision.getTagBearing(desiredTagID);
      yaw = ourVision.getTagYaw(desiredTagID);
    }

    while (opModeIsActive() && targetTagFound && range > desiredTagDistance) {

      // Post and start moving based on current values
      telemetry.addData("TAG: range, heading, yaw", JavaUtil.formatNumber(range, 4, 2) + ", " + JavaUtil.formatNumber(heading, 4, 2) + ", " + JavaUtil.formatNumber(yaw, 4, 2));
      ourDrivetrain.driveToHeading(desiredTagDistance, range, heading, yaw);

      // **** ASSESS STATE AFTER MOVEMENT ****
      // Determines and sets latest values for desired AprilTag
      targetTagFound = ourVision.getTagInfo(desiredTagID);

      // Determine range, heading and yaw (tag image rotation) so we can use them to
      // control the robot automatically.
      if (targetTagFound) {
        range = ourVision.getTagRange(desiredTagID);
        heading = ourVision.getTagBearing(desiredTagID);
        yaw = ourVision.getTagYaw(desiredTagID);
      }
    }

    // Stop movement
    ourDrivetrain.driveStraight(0,0,0);

    // Go to BACKDROP pose
    if (opModeIsActive() && ourArm.waitingForCommand()) {
      ourArm.moveToPose_BACKDROP();
      // Iterates on arm until it has moved to our requested pose
      while (opModeIsActive() && !ourArm.waitingForCommand()) {
        // Runs arm based on the state.
        ourArm.runArmIteration();
        telemetry.update();
      }
      sleep(1000);
    }

    // Let's move to the backdrop
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      // NOTE: currentAlliance is set by GAMEPAD 1 in INIT
      if (currentAlliance == BLUEALLIANCE) {
        ourDrivetrain.driveStraight(speedForMoveTasks, desiredTagDistance, yawToBackdropBLUE);
      } else {
        ourDrivetrain.driveStraight(speedForMoveTasks, desiredTagDistance, yawToBackdropRED);
      }
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
        telemetry.update();
      }
    }

    // ***** EJECT OUR YELLOW PIXEL *****
    if (opModeIsActive()) {
      ourActiveIntake.ejectOneGameObject();
    }

  }  // moveToBackdropAndDropYellowPixel


  /**
   * Used during INIT ONLY to look for gamepad 1 DPAD selections to move the intake
   */
  private void IfAskedDoIntake() {
    if (gamepad1.dpad_down) {
      // Moves intake inward
      ourActiveIntake.moveInward();
    } else if (gamepad1.dpad_up) {
      // Stops the intake
      ourActiveIntake.moveOutward();
    } else  {
      // Moves intake outward
      ourActiveIntake.stop();
    }
  }
  
}
