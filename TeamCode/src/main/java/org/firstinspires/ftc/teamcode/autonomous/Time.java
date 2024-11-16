package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Time", group="Robot")
public class Time extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        waitForStart();
        // Step  through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        robot.driveRobot(0.5, 0,0);
        sleep(1000);

        robot.driveRobot(0, 0,0);
        robot.intake.setPower(robot.INTAKE_DEPOSIT);
        sleep(100);

        robot.driveRobot(-0.5, 0,0);
        robot.intake.setPower(robot.INTAKE_OFF);
        sleep(500);

        robot.driveRobot(0, 0.5,0);
        sleep(1500);

        robot.driveRobot(0, 0,0);
        sleep(100);

        robot.driveRobot(0.2, 0,0);
        robot.intake.setPower(robot.INTAKE_COLLECT);
        sleep(200);

        robot.driveRobot(0, 0,0);
        sleep(100);

        robot.driveRobot(-0.2,0,0);
        robot.intake.setPower(robot.INTAKE_OFF);
        sleep(200);

        robot.driveRobot(0,-0.5,0);
        sleep(1500);

        robot.driveRobot(0.4,0, 0);
        sleep(1000);
    }
}
