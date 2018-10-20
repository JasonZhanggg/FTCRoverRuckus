

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name = "Sensor: MR Gyro", group = "Sensor")
public class GoldFinder extends LinearOpMode {
  boolean flag = true;
  ElapsedTime timer = new ElapsedTime();
  MyHardwarePushbot robot   = new MyHardwarePushbot();   // Use a Pushbot's hardware
  private GoldAlignDetector detector;

  @Override
  public void runOpMode() {
    robot.init(hardwareMap, this, "autonomous");

    detector = new GoldAlignDetector();
    detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
    detector.useDefaults();

    // Optional Tuning
    detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
    detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
    detector.downscale = 0.4; // How much to downscale the input frames

    detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
    //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
    detector.maxAreaScorer.weight = 0.005;

    detector.ratioScorer.weight = 5;
    detector.ratioScorer.perfectRatio = 1.0;

    detector.enable();
    waitForStart();
    robot.encoderDrive(0.4, -18, -18, 12);
    while (opModeIsActive())  {
      telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
      telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.
      if (!detector.getAligned() && flag) {
        robot.encoderDrive(0.2, 0.3, 0.3, 0.5);

      } else {
        flag = false;
        break;
      }
    }

  }


}
