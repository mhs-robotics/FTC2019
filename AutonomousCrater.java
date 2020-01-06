/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="ShortRobotAutonomousCrater", group="Linear Opmode")
public class RobotAutonomousCrater extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor midMotor = null;
    private DcMotor liftMotor = null;
    private DcMotor armMotor = null;
    private DcMotor sweepMotor = null;
    private DcMotor spoolMotor = null;
    private Servo markerServo;
    public int position = 0;
    public ElapsedTime mRunTime = new ElapsedTime();
    
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "ATvtsBv/////AAABmZo9ZPun/03slIp0OwyihLob4TRGzSgZ/iXriP9XpMwhFzru7yyuUgMacCSFHMggIjDPVUB3iqEUWmnCG1CQv8yuo6fx1/Goa2CSP0lbn84vC4R8OwyQJLB5Doa6SQ/n9+tj9NgEMeS3r7Eq466BkG2kOlbqon8RcMRq/U70lX0cCyv5/TSZZxeLh5r2HAB1MiUau0NJymE4VZbKJ+g8nuaNWz437v0xAE/VHEd/tFZtrtir4vuZPTSQN/I01fcdj5bOC3xrt4R/hlvWoz+RfYJ6MFAUah1D91FHUGLOP87PjqjEAXN7K91PQRM/GVD0TFDjgNUnfFjO4AsL+3aPrsf4aQP0snpTmmNnkfnH7RaU";
    
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
     
    private TFObjectDetector tfod;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        midMotor = hardwareMap.dcMotor.get("middle_motor");
        liftMotor = hardwareMap.dcMotor.get("lift_motor");
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        sweepMotor = hardwareMap.dcMotor.get("sweep_motor");
        spoolMotor = hardwareMap.dcMotor.get("spool_motor");
        markerServo = hardwareMap.servo.get("marker_servo");
        /*
         * Setting the drive motor directions
         */

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        midMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        markerServo.setPosition(0);
        
        // reset encoder count kept by motors.
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run to target encoder position and stop with brakes on.
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //This runs all of the different segments of autonomous
        liftDown();
        leaveLandingArea();
        sample();
        testForErrors();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
    
    void testForErrors() {
        if (position == 4) {
            moveCrater();
        }
        else {
            moveToDepot();
        }
    }

    void sample() {
        
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive() && position == 0) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 2){
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            
                            int silverMineralX = (silverMineral1X + silverMineral2X);
                            
                            if(goldMineralX == -1){
                                telemetry.addData("left",0);
                                position = 1;
                                telemetry.addData("position", position);
                            }    
                        
                            else if ((goldMineralX != -1) && ((goldMineralX) < 600)){
                                telemetry.addData("middle",0);
                                position = 2;
                                telemetry.addData("position", position);
                            }
                            
                            else if ((goldMineralX != -1) && ((goldMineralX) > 600)){
                                telemetry.addData("right",0);
                                position = 3;
                                telemetry.addData("position", position);
                            }
                            
                            else {
                                position = 4;
                                telemetry.addData("***",0);
                            }
                            telemetry.addData("Gold Mineral Position", goldMineralX);
                            telemetry.addData("silverMineral1X ", silverMineral1X);
                            telemetry.addData("silverMineral2X", silverMineral2X);
                        }
                        else if (mRunTime.time() >= 4) {
                           position = 4; 
                        }
                        telemetry.update();
                    }
                }
            }
            tfod.shutdown();
        telemetry.addData("Running", "Scan");
    }
    
    //this lifts the robot down
    void liftDown() {
        liftMotor.setTargetPosition(30000);
        liftMotor.setPower(1);
        while (liftMotor.isBusy()) {
            
        }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //this gets the robot off the hook and moves to the sampling
    void leaveLandingArea() {
        //moves it backwards off of the hook
        mRunTime.reset();
        leftMotor.setPower(0.3);
        rightMotor.setPower(-0.3);
        while (mRunTime.time() <= 1) {
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0); 

        //moves is sideways to the sampling
        mRunTime.reset();
        midMotor.setPower(.4);
        while (mRunTime.time() <= 0.5) {
            
        }
        midMotor.setPower(0);
        
        //move forward
        mRunTime.reset();
        leftMotor.setPower(-1);
        rightMotor.setPower(1);
        while (mRunTime.time() <= 0.5) {
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    //This samples the minerals, then moves into position to drive forward and knock off the chosen mineral

    //this knocks off the chosen mineral and moves into the depot
    void moveToDepot() {
        if(position == 1) {
            //strafe away from lander to avoid crashing with it
            mRunTime.reset();
            midMotor.setPower(0.5);
            while (mRunTime.time() <= 1) {
            
            }
            midMotor.setPower(0);
            
            //move back
            mRunTime.reset();
            leftMotor.setPower(0.8);
            rightMotor.setPower(-0.8);
            while (mRunTime.time() <= 1.7) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //turn
        mRunTime.reset();
            leftMotor.setPower(0.8);
            rightMotor.setPower(0.8);
            while (mRunTime.time() <= 0.65) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //move forward into crater
        mRunTime.reset();
            leftMotor.setPower(-1);
            rightMotor.setPower(1);
            while (mRunTime.time() <= 1.1) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        }
        
        else if(position == 2) {
            //move back
            mRunTime.reset();
            leftMotor.setPower(0.8);
            rightMotor.setPower(-0.8);
            while (mRunTime.time() <= 0.6) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //turn
        mRunTime.reset();
            leftMotor.setPower(0.8);
            rightMotor.setPower(0.8);
            while (mRunTime.time() <= 0.65) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //move forward to crater 
        mRunTime.reset();
            leftMotor.setPower(-0.8);
            rightMotor.setPower(0.8);
            while (mRunTime.time() <= 2) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
            
        }
        
        else if(position == 3) {
            //strafe away from lander to avoid crashing with it
            mRunTime.reset();
            midMotor.setPower(0.5);
            while (mRunTime.time() <= 1) {
            
            }
            midMotor.setPower(0);
            
            //move forward
            mRunTime.reset();
            leftMotor.setPower(-0.8);
            rightMotor.setPower(0.8);
            while (mRunTime.time() <= 0.5) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //turn
        mRunTime.reset();
            leftMotor.setPower(0.8);
            rightMotor.setPower(0.8);
            while (mRunTime.time() <= 0.8) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //move forward 
        mRunTime.reset();
            leftMotor.setPower(-1);
            rightMotor.setPower(1);
            while (mRunTime.time() <= 1.7) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
            
        }
        else {
            telemetry.addData("Error", 0);
        }
        telemetry.addData("Running", "Move To Depot");
    }
    
    void moveCrater() {
        //strafes right
        mRunTime.reset();
            midMotor.setPower(0.5);
            while (mRunTime.time() <= 1) {
            
            }
        midMotor.setPower(0);
        //forward
            mRunTime.reset();
            leftMotor.setPower(-0.8);
            rightMotor.setPower(0.8);
            while (mRunTime.time() <= 1.4) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        
        //turn left
        mRunTime.reset();
            leftMotor.setPower(0.8);
            rightMotor.setPower(0.8);
            while (mRunTime.time() <= 1) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //move forward to crater
        mRunTime.reset();
            leftMotor.setPower(-0.8);
            rightMotor.setPower(0.8);
            while (mRunTime.time() <= 2.3) {
            
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        }
    
    /**
     * Initialize the Vuforia localization engine.
     */
    
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.65;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}


