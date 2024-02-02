package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
  int MINX_FORSPIKE_RIGHT = 300;  // X pixels on image where left of X is Center Spike, right of X is Right Spike
  final int SPIKE_LEFT = 1;
  final int SPIKE_CENTER = 2;
  final int SPIKE_RIGHT = 3;
  int targetSpike = SPIKE_LEFT; // Default to left spike
  
  DrivetrainMecanumWithSmarts ourDrivetrain = new DrivetrainMecanumWithSmarts();
  Vision ourVision = new Vision();
  Arm ourArm = new Arm();
  ActiveIntakeWithServo ourActiveIntake = new ActiveIntakeWithServo();

  
  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

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

      // Make sure our drivetrain and arm are waiting for a command
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand() && ourArm.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
        // Runs arm based on the state.
        ourArm.runArmIteration();
        telemetry.update();
      }

      // Ready to perform first goal of placing purple pixel on correct spike mark
      requestToMoveAndDropPurplePixel();

      // Ready to perform second goal of scoring yellow pixel above correct AprilTag

      // Ready to perform last goal of parking
      parkFromSpikeTileToBackstageWallTile();
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

  }

  /**
   * Describe this function...
   */
  private void requestToMoveAndDropPurplePixel() {
    double speedForSpikeTasks = 0.3;  // speed to use for all spike tasks
    double distanceToSpikeTile = 22.0;  // Distance (inches) to the middle of the Spike tile
    double initialHeading = 0; // Heading of the robot when it first starts up

    // Yaw to turn clockwise (to right spike) or counterclockwise (to left spike)
    double yawToRightSpike = -50;  // Heading to turn towards the right spike location
    double yawToLeftSpike = 50;  // Heading to turn towards the left spike location

    // Distance to the spike.
    double distanceToSpikeLocation = 7;  // Distance (inches) to each of the spike locations

    // Let's move to the middle of the spike tile
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      ourDrivetrain.driveStraight(speedForSpikeTasks, distanceToSpikeTile, initialHeading);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
      }
      telemetry.update();
    }

    // Turn to left or right spike if necessary
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      if (targetSpike == SPIKE_RIGHT) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        ourDrivetrain.turnToHeading(speedForSpikeTasks, yawToRightSpike);
      } else if (targetSpike == SPIKE_LEFT) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        ourDrivetrain.turnToHeading(speedForSpikeTasks, yawToLeftSpike);
      }
      // Iterates on drivetrain until it has moved to our requested heading
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on our request
        ourDrivetrain.runDrivetrainIteration();
      }

    }

    // Move to targeted spike
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      if (targetSpike == SPIKE_RIGHT) {
        ourDrivetrain.driveStraight(speedForSpikeTasks, distanceToSpikeLocation, yawToRightSpike);
      } else if (targetSpike == SPIKE_LEFT) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        ourDrivetrain.driveStraight(speedForSpikeTasks, distanceToSpikeLocation, yawToLeftSpike);
      } else {
        ourDrivetrain.driveStraight(speedForSpikeTasks, distanceToSpikeLocation, initialHeading);
      }
      // Iterates on drivetrain until it has moved to our requested heading
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
      }
      telemetry.update();
    }

    telemetry.update();

    // ***** EJECT OUR PURPLE PIXEL *****
    if (opModeIsActive()) {
      ourActiveIntake.ejectOneGameObject();
    }

  }

  /**
   * Describe this function...
   */
  private void parkFromSpikeTileToBackstageWallTile() {
    // Turn to face backdrop AprilTags
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
      ourDrivetrain.turnToHeading(0.5, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
      }
    }
    // Move left towards wall
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      // Drives Left or Right.
      ourDrivetrain.driveLeft(0.3, 16, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
      }
      telemetry.update();
    }
    // Move to backstage
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && ourDrivetrain.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      ourDrivetrain.driveStraight(0.3, 40, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !ourDrivetrain.waitingForCommand()) {
        // Runs drivetrain based on the state.
        ourDrivetrain.runDrivetrainIteration();
      }
      telemetry.update();
    }

    sleep(4000);
  }

  /**
   * TODO
   */
  private void requestToMoveToBackdropAndDropYellowPixel() {
    // Returns whether we are waiting for a command.
    // Returns whether we are waiting for a command.
    while (opModeIsActive() && !ourDrivetrain.waitingForCommand() && ourArm.waitingForCommand()) {
      // Runs drivetrain based on the state.
      ourDrivetrain.runDrivetrainIteration();
      // Runs arm based on the state.
      ourArm.runArmIteration();
      telemetry.update();
    }
  }

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
