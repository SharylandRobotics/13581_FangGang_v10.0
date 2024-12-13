package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Encoder", group="Robot")
public class Encoder extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        waitForStart();
        // Step  through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        robot.encoderDrive(robot.DRIVE_SPEED, -18, -18, -18, -18, 5);
        resetRuntime();

        while (getRuntime() < 2) {
            robot.vSlidePosition = robot.VSLIDE_SCORE_SAMPLE_HIGH;
            robot.setvSlideDrivePosition();
        }
        resetRuntime();

        while (getRuntime() < 2) {
            robot.intake2.setPosition(1.0 / 10);
        }
        resetRuntime();

        while (getRuntime() < 1) {
            robot.intake2.setPosition(0);
        }
        resetRuntime();

        while (getRuntime() < 2) {
            robot.vSlidePosition = robot.VSLIDE_START_POSITION;
            robot.setvSlideDrivePosition();
        }
        resetRuntime();

        robot.encoderDrive(robot.DRIVE_SPEED, 12, 12, 12, 12, 12);

        robot.encoderDrive(robot.DRIVE_SPEED, -41.25, 41.25, -41.25, 41.25, 5);

        robot.encoderDrive(robot.DRIVE_SPEED, -25.25, -12.625, 25.25, 12.625, 5);

        while (getRuntime() < 0.5){

            robot.encoderDrive(robot.DRIVE_SPEED, -2.1, -2.1, -2.1, -2.1, 5);

        }
        resetRuntime();

        while (getRuntime() < 1) {

            robot.intake2.setPosition(1.0 / 10);

        }
        resetRuntime();

        while (getRuntime() < 1) {

            robot.leftWrist.setPosition(0.53);

        }
        resetRuntime();

        while (getRuntime() < 2) {

            robot.intake.setPower(robot.INTAKE_COLLECT);

        }
        resetRuntime();

        while (getRuntime() < 1) {

            robot.leftWrist.setPosition(-0.53);

        }
        resetRuntime();

        while (getRuntime() < 1) {

            robot.intake2.setPosition(0);

        }
        resetRuntime();

        while (getRuntime() < 2) {

            robot.intake.setPower(robot.INTAKE_DEPOSIT);

        }
        resetRuntime();

        while (getRuntime() < 1){

            robot.intake.setPower(0.0);

        }
        resetRuntime();

        while (getRuntime() < 3) {

            robot.encoderDrive(robot.DRIVE_SPEED, -24, -12, 24, 12, 5);

        }
        resetRuntime();

        while (getRuntime() < 1){

            robot.encoderDrive(robot.DRIVE_SPEED, -32, -32, -32, -32, 5);

        }
        resetRuntime();

        while(getRuntime()< 2){

            robot.encoderDrive(robot.DRIVE_SPEED, 2, -2, -2, 2, 5);

        }
        resetRuntime();

        while (getRuntime() < 2) {
            robot.vSlidePosition = robot.VSLIDE_SCORE_SAMPLE_HIGH;
            robot.setvSlideDrivePosition();
        }
        resetRuntime();

        while (getRuntime() < 2) {
            robot.intake2.setPosition(1.0 / 10);
        }
        resetRuntime();

        while (getRuntime() < 1) {
            robot.intake2.setPosition(0);
        }
        resetRuntime();

        while (getRuntime() < 2) {
            robot.vSlidePosition = robot.VSLIDE_START_POSITION;
            robot.setvSlideDrivePosition();
        }
        resetRuntime();

        }

    }


