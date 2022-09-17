
/**
 You need to create a folder called OpModes where you put all of your Auto and TeleOp code because some imports rely on this folder.

 This file's path is: FtcRobotController/TeamCode/java/org.firstinspires.ftc.teamcode/Opmodes/RedAutoCarouselParkSU.java
 **/


package org.firstinspires.ftc.teamcode.Opmodes;

import java.lang.System.*;
import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Helper.Robot;
import org.firstinspires.ftc.teamcode.Helper.RobotVision;



@Autonomous(name="New Red Carousel SU Park", group="Linear Opmode")
public class RedAutoCarouselParkSU extends LinearOpMode {
    //public LinearOpmode opModeObj;
    public long programInitTime = 0;//time the program init
    public long programStartTime = 0;//time the program starts
    public long programCurrentTime = 0;//time the current time in the program
    public long stateStartTime = 0;//time the program state starts
    public long stateEndTime = 0;//time the program state end

    public ElapsedTime test_run_time = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();

    public float maxSpeed = 0.5f;
    public float medSpeed = 0.25f;

    public String objectLabel = null;
    public boolean objectFound = false;

    //ducks
    public double objectAngle;
    public int barCode = 3;
    double timeout_ms=0;
    float offset;
    float angleOne;
    float angleTwo;

    private String strafeDir = "";

    public enum AutoStep {
        detectDuck, movetoAllianceHub, deliver, turnIntake, gotoCarousel, preCarousel, turnCarousel, park, endAuto
    }

    public AutoStep autoStep = AutoStep.detectDuck;

    Robot robot = new Robot();
    RobotVision rVision = new RobotVision();

    private boolean autoMode = false;
    private double ARM_HOLDING_POWER = 0.3;
    private double ARM_LIFTING_POWER = 0.4;
    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Instance of Robot is initialied
         * Instance of Robot is RobotVision
         * TensorFlow Object detection initialized
         */
        robot.init(hardwareMap);
        rVision.initVuforia(hardwareMap);
        rVision.initTfod(hardwareMap);

        /**
         * This code is run during the init phase, and when opMode is not active
         * i.e. When "INIT" Button is pressed on the Driver Station App
         */
        while (!opModeIsActive() && !isStopRequested()) {
            sleep(100);

            List<Recognition> updatedRecognitions = rVision.tfod.getUpdatedRecognitions();
            // inference is slow. In different iterations of the loop, the recognition may not return
            if (updatedRecognitions != null) {

//                objectAngle = 0;
//                objectLabel = "";
                /**
                 * Is object has not be recognized then runthrough all the 4 labels
                 * i.e. Duck, Ball, Cube & Marker and our Vortex
                 */
                for (Recognition recognition : updatedRecognitions) {
                    objectLabel = recognition.getLabel();
                    /**
                     * Object Angle with the Camera is used to identified the barcode
                     */
                    objectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                    /**
                     * If the object has been identified then exit the detection loop
                     * Set the barcode patern number which must match with level of alliance hub
                     */
                    objectFound = (objectLabel == "Ball" || objectLabel == "Duck");
                    if (objectFound && objectAngle < -1.0) {
                        barCode = 1;
                        break;
                    } else if (objectFound && objectAngle > 2) {
                        barCode = 2;
                        break;

                    } else {
                        barCode = 3;
                        break;
                    }
                }
            }
            else {
                barCode = 3;
            }

            telemetry.addData("Barcode", barCode);
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Object Label", "%s", objectLabel);
            telemetry.addData("Object Angle", objectAngle);
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();

        }


        waitForStart();

        programStartTime = SystemClock.uptimeMillis();

        /**
         * Opmode Active when "Play" button is pressed Driver Station App
         * The steps required for Autonomous is automated using AutoStep
         */

        autoStep = AutoStep.detectDuck;

        while (opModeIsActive()) {

            /**
             * For Red Carousel, following steps are to be performed
             * autoStep = AutoStep.detectDuck;
             * If objectFound == true, duck or Team Element (Vortex) found
             * - 1: Move towards Alliance Hib to deliver the Freight
             * - 2: Move towards Carousel
             * - 3: Goto Carousel and rotate Red Carousel
             * - 4: Part to shared hub
             */
            switch (autoStep) {

                /**
                 * Code con be added here is objectFound is True
                 */

                //detect
                case detectDuck:
                    telemetry.addData("Barcode", barCode);
                    autoStep = AutoStep.movetoAllianceHub;
                    break;


                case movetoAllianceHub:

                    /**
                     * Purpose : Driving robot forward towards alliance hub
                     * Control : Using encoder & timer
                     * Power : Mimimum power to avoid slippage
                     */
                    runtime.reset();
                    timeout_ms = 5000;
                    robot.startDriveToPosition(0.5, 8);
                    while (opModeIsActive() &&
                            (runtime.milliseconds() < timeout_ms) &&
                            (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {

                    }
                    robot.stopDriveMotors();
                    sleep(100);

                    runtime.reset();
                    timeout_ms = 5000;
                    /**
                     needs negitive power because it is still backwards, and needs to move right.
                     */
                    robot.startStrafeToPosition(0.5, 15);
                    while (opModeIsActive() &&
                            (runtime.milliseconds() < timeout_ms) &&
                            (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {


                    }

                    robot.stopDriveMotors();
                    autoStep = AutoStep.deliver;
                    break;

                case deliver:

                    robot.turnRobot(330);
                    robot.robotStop();
                    robot.alignStraight(robot.getRobotAngle(),  35, 0.3);
                    robot.robotStop();

                    // TODO Code needed to lift Arm height defined by Barcode



                    robot.moveArmToTarget(3, 0.7);
                    robot.arm.setPower(0);


                    /**
                     * lifts the arm according to the barcode position
                     * we lift the arm, go forwards, deliver, and then go back
                     */
                    robot.movePulleyToTarget(barCode, 0.8);
                    sleep(100);
//                  robot.arm.setPower(ARM_HOLDING_POWER);

                    runtime.reset();
                    timeout_ms = 5000;
                    robot.startDriveToPosition(0.2, 45);
                    while (opModeIsActive() &&
                            (runtime.milliseconds() < timeout_ms) &&
                            (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {
                    }
//                    robot.arm.setPower(ARM_HOLDING_POWER);
                    robot.stopDriveMotors();
                    sleep(100);

                    autoStep = AutoStep.turnIntake;

                    /**
                     * opens and closes box cover to deliver
                     */
                    // TODO Open Box cover
                    break;



                case turnIntake:
                    // robot.turnIntake(100, -0.5);
                    // Outtake power should be <-0.4 - otherwise freight is thrown too far.
                    robot.sIntake.setPower(-0.2);
                    sleep(2000);
                    robot.sIntake.setPower(0);

                    robot.moveArmToTarget(1, 1);
                    sleep(100);

                    autoStep = AutoStep.preCarousel;
                    break;

                case preCarousel:
                    //robot.preCarouselStep(20);

                    /**
                     * moves away from hub
                     */

                    runtime.reset();
                    timeout_ms = 5000;
                    robot.startDriveToPosition(0.5, -45);
                    while (opModeIsActive() &&
                            (runtime.milliseconds() < timeout_ms) &&
                            (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {
                    }
                    robot.stopDriveMotors();
                    sleep(100);

                    /**
                     * lifts the arm according to the barcode position
                     * we lift the arm, go forwards, deliver, and then go back
                     */

                    robot.turnRobot(215);
                    robot.robotStop();

                    robot.movePulleyToTarget(1, 0.8);

                    sleep(100);
//                  robot.arm.setPower(ARM_HOLDING_POWER);





                    runtime.reset();
                    timeout_ms = 3000;

                    /**
                     * turns towards carousel
                     */

                    autoStep = AutoStep.gotoCarousel;
                    break;



                /**
                 *Control: Uses encoder and timer
                 * Purpose: Drive to carousel
                 * Power: Low power to avoid slippage
                 */
                case gotoCarousel:
//                    runtime.reset();
//                    timeout_ms = 3000;
//                    robot.startDriveToPosition(0.5, 15);
//                    while (opModeIsActive() &&
//                            (runtime.milliseconds() < timeout_ms) &&
//                            (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {
//                    }
//                    robot.stopDriveMotors();
//                    sleep(100);
                    runtime.reset();
                    robot.startStrafeToPosition(0.5, 63);
                    while (opModeIsActive() &&
                            (runtime.milliseconds() < timeout_ms) &&
                            (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {
                    }
                    robot.stopDriveMotors();
                    sleep(100);
                    autoStep = AutoStep.turnCarousel;
//                    autoStep = AutoStep.park;
                    break;


                /**
                 *Control: Uses timer
                 * Purpose: Delivers duck
                 * Power: med power so that duck does not go flying off carousel
                 */
                case turnCarousel:

                    robot.turn_carousel(1, -1, 3000);
                    autoStep = AutoStep.park;
                    break;

                case park:

                    runtime.reset();
                    robot.startDriveToPosition(0.8, -50);
                    timeout_ms = 5000;
                    while (opModeIsActive() &&
                            (runtime.milliseconds() < timeout_ms) &&
                            (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {
                    }
                    robot.stopDriveMotors();
                    runtime.reset();
                    timeout_ms = 5000;
                    robot.startStrafeToPosition(0.5, 5);
                    while (opModeIsActive() &&
                            (runtime.milliseconds() < timeout_ms) &&
                            (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {
                    }

                    robot.stopDriveMotors();
                    autoStep = AutoStep.endAuto;
                    break;

                case endAuto:   // End Auto here
                    //autonomousStep = AutonomousStep.moveForward;
                    break;

            }
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.update();

        }


        programCurrentTime = SystemClock.uptimeMillis();

        //robot.moveUpdate();
        telemetry.addData("ProgramTime", "%d", (programCurrentTime - programStartTime));
        telemetry.addData("Autonomous Step", "%s ", autoStep);
        telemetry.addData("Move Step", "%s ", robot.moveStep);
        telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
        telemetry.update();

    }
}
