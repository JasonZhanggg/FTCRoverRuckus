

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
public class MyHardwarePushbot {
    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveBack = null;
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveFront = null;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535);
    public ModernRoboticsI2cRangeSensor rangeSensor;

    HardwareMap hwMap = null;
    LinearOpMode op_mode = null;
    ElapsedTime runtime = new ElapsedTime();
    public MyHardwarePushbot() {

    }


    public void init(HardwareMap ahwMap, LinearOpMode opMode, String runeMode) {
        op_mode = opMode;
        hwMap = ahwMap;
        leftDriveFront = hwMap.get(DcMotor.class, "leftf");
        rightDriveFront = hwMap.get(DcMotor.class, "rightf");
        leftDriveBack = hwMap.get(DcMotor.class, "leftb");
        rightDriveBack = hwMap.get(DcMotor.class, "rightb");
        rangeSensor =   hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        if(runeMode.equals("teleop")) {
            setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if(runeMode.equals("autonomous")){
            setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }

    public void setDriveRunMode(DcMotor.RunMode runmode) {
        leftDriveFront.setMode(runmode);
        leftDriveBack.setMode(runmode);
        rightDriveFront.setMode(runmode);
        rightDriveBack.setMode(runmode);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTargetFront;
        int newLeftTargetBack;
        int newRightTargetBack;
        int newRightTargetFront;

        // Ensure that the opmode is still active
        if (op_mode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRightTargetBack = rightDriveBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightTargetFront = rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTargetBack = leftDriveBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftTargetFront = leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);


            leftDriveFront.setTargetPosition(newLeftTargetFront);
            leftDriveBack.setTargetPosition(newLeftTargetBack);
            rightDriveFront.setTargetPosition(newRightTargetFront);
            rightDriveBack.setTargetPosition(newRightTargetBack);

            // Turn On RUN_TO_POSITION
            setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            setDrivePower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (op_mode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDriveBack.isBusy() && rightDriveBack.isBusy() &&
                            leftDriveFront.isBusy() && rightDriveFront.isBusy())) {

                // Display it for the driver.
                op_mode.telemetry.addData("Path_1", "Running to %7d :%7d: %7d :%7d",
                        newLeftTargetFront, newRightTargetFront, newLeftTargetBack, newRightTargetBack);
                op_mode.telemetry.addData("Path_2", "Running at %7d :%7d: %7d : %7d",
                        leftDriveFront.getCurrentPosition(),
                        rightDriveFront.getCurrentPosition(),
                        leftDriveBack.getCurrentPosition(),
                        rightDriveBack.getCurrentPosition());
                op_mode.telemetry.update();
            }

            // Stop all motion;
            setDrivePower(0);


            // Turn off RUN_TO_POSITION
            setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }


    }
    public void setDrivePower(double power) {
        leftDriveFront.setPower(Math.abs(power));
        leftDriveBack.setPower(Math.abs(power));
        rightDriveFront.setPower(Math.abs(power));
        rightDriveBack.setPower(Math.abs(power));
    }

}


