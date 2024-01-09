package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto_DropThenBackdrop_v1_4 (Blocks to Java)")
public class Auto_DropThenBackdrop_v1_4 extends LinearOpMode {
  
  final String REDALLIANCE = "Red";
  final String BLUEALLIANCE = "Blue";
  String currentAlliance = BLUEALLIANCE;

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
  
  private Servo motor_shoulder;
  
  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    // Initialize our lower pixel ejection
    PixelEjector.init("motor_dropPixels");

    // Initialize Drivetrain
    DrivetrainMecanumWithSmarts.initDriveTrainWithSmarts("left_front_drive", "left_back_drive", "right_front_drive", "right_back_drive", "distance_left_front");

    // Initialize Arm and Poses
    Arm.initArm("motor_shoulder", "motor_elbow", "motor_wrist");

    // TODO: MOVE THIS ACTION TO THE ARM CLASS
    // Moving the shoulder up slightly to allow lower pixels to move freely
    motor_shoulder = hardwareMap.get(Servo.class, "motor_shoulder");
    motor_shoulder.setPosition(0.4);

    // Initialize gripper
    Gripper.init("motor_gripper", 1, 0.2, 0.5, 1);

    // Initialize our vision
    // Initialize vision libraries
    useVision = Vision.initVision("Webcam_front", "Webcam_back");

    telemetry.addData("ALLIANCE", currentAlliance);
    telemetry.addData("LOCATION", currentLocation);
    telemetry.addData("TARGET SPIKE (1=Left, 2=Center, 3=Right)", targetSpike);
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();


    while (opModeInInit() && !gamepad1.back) {
      if (useVision) {
        // Indicates whether the Team Prop is in sight.
        targetSpike = Vision.getTeamPropLocation("Bolt", MINX_FORSPIKE_RIGHT);
      }

      // Using the X,Y,A or B on gamepad 1 to designate location and Alliance
      getLocationPlusAlliance();
      IfAskedDoGripper();
      IfAskedMoveLowerEjector();

      telemetry.addData("ALLIANCE", currentAlliance);
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

    waitForStart();

    if (opModeIsActive()) {
      telemetry.addData("ALLIANCE", currentAlliance);
      telemetry.addData("LOCATION", currentLocation);
      telemetry.addData("USE VISION", useVision);
      telemetry.addData("TARGET SPIKE (1=Left, 2=Center, 3=Right)", targetSpike);
      telemetry.update();
      // Make sure our drivetrain is waiting for a command
      // Returns whether we are waiting for a command.
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand() && Arm.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        // Runs arm based on the state.
        Arm.runArmIteration();
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
    // Distance to be equal distance in the middle of all three spikes
    double distanceToMid = 50.0;  // NOT VALID or UPDATED / USED
    // Yaw to turn clockwise (to right spike) or counterclockwise (to left spike)
    double yawToRightSpike = 0.5;  // NOT VALID or UPDATED / USED
    double yawToLeftSpike = -0.5;  // NOT VALID or UPDATED / USED
    // Distance to the spike.
    double distanceToSpike = 75;  // NOT VALID or UPDATED / USED

    // Let's move to the middle of the spike tile
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, 22, 0);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    // Turn to left or right spike if necessary
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      if (targetSpike == SPIKE_RIGHT) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        DrivetrainMecanumWithSmarts.turnToHeading(0.75, -50);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
          // Runs drivetrain based on the state.
          DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        }
      } else if (targetSpike == SPIKE_LEFT) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        DrivetrainMecanumWithSmarts.turnToHeading(0.75, 50);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
          // Runs drivetrain based on the state.
          DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        }
      }
    }
    // Move to targeted spike
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, 7, 0);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    telemetry.update();
    if (opModeIsActive()) {
      PixelEjector.ejectPixel();
    }
    // Move away spike
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, -7, 0);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    // Turn to left or right if necessary
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      if (targetSpike == SPIKE_RIGHT) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        DrivetrainMecanumWithSmarts.turnToHeading(0.5, 0);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
          // Runs drivetrain based on the state.
          DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        }
      } else if (targetSpike == SPIKE_LEFT) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        DrivetrainMecanumWithSmarts.turnToHeading(0.5, 0);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
          // Runs drivetrain based on the state.
          DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        }
      }
    }
    // Move back away from spikes
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, -7, 0);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void parkFromSpikeTileToBackstageWallTile() {
    // Turn to face backdrop AprilTags
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
      DrivetrainMecanumWithSmarts.turnToHeading(0.5, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
    }
    // Move left towards wall
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Left or Right.
      DrivetrainMecanumWithSmarts.driveLeft(0.3, 16, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    // Move to backstage
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, 40, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    if (opModeIsActive()) {
      PixelEjector.ejectPixel();
    }
    motor_shoulder.setPosition(0.3);
    sleep(4000);
  }

  /**
   * Describe this function...
   */
  private void requestToMoveToBackdropAndDropYellowPixel() {
    // Returns whether we are waiting for a command.
    // Returns whether we are waiting for a command.
    while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand() && Arm.waitingForCommand()) {
      // Runs drivetrain based on the state.
      DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      // Runs arm based on the state.
      Arm.runArmIteration();
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedDoGripper() {
    if (gamepad1.dpad_left) {
      // Opens gripper
      Gripper.GripNone();
    } else if (gamepad1.dpad_down) {
      // Grips one only
      Gripper.GripOne();
    } else if (gamepad1.dpad_right) {
      // Tightens gripper to hold two pixels
      Gripper.GripTwo();
    }
  }

  /**
   * Describe this function...
   */
  public void IfAskedMoveLowerEjector() {
    if (gamepad1.right_bumper) {
      PixelEjector.moveInward();
    } else if (gamepad1.right_trigger != 0) {
      PixelEjector.moveOutward();
    } else {
      PixelEjector.stop();
    }
  }
  
}
