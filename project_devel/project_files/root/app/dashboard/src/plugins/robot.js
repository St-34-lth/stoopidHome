//my code starts here 

import { ref, reactive, inject } from 'vue'; // Importing Vue's reactive and ref utilities, and inject to get dependencies from the parent components.

class RobotController {
    constructor(options) {
        // Initialize properties
        this.props = options.props; // Storing the passed props.
        
//         this.userEmitter = inject("userEmitter"); // Injecting the userEmitter for emitting commands to the robot.

//         // Robot state
//         this.currentHdg = inject('robotHeading'); // Injecting the robot's heading (current direction).
//         this.currentSpeed = inject('robotSpeed'); // Injecting the robot's current speed.
    }

    // Function to handle robot commands based on button press events
    robotCommand(event) {
        console.log(event); // Log the event (for debugging purposes).
        let button = event.target.id; // Get the button ID from the event target.
        let robotCmd = {}; // Initialize an empty object to hold the robot command.

        // Use a switch statement to assign appropriate movement commands based on the button pressed.
        switch (button) {
            case ('forward'):
                robotCmd.dirA = '0'; // Set the robot's left wheel direction.
                robotCmd.pwmA = '1'; // Set the robot's left wheel power.
                robotCmd.pwmB = '1'; // Set the robot's right wheel power.
                robotCmd.dirB = '0'; // Set the robot's right wheel direction.
                break;
            case ('backward'):
                robotCmd.dirA = '1';
                robotCmd.pwmA = '1';
                robotCmd.pwmB = '1';
                robotCmd.dirB = '1';
                break;
            case ('ArrowLeft'):
                robotCmd.dirA = '1';
                robotCmd.pwmA = '1';
                robotCmd.pwmB = '1';
                robotCmd.dirB = '0';
                break;
            case ('ArrowRight'):
                robotCmd.dirA = '0';
                robotCmd.pwmA = '1';
                robotCmd.pwmB = '1';
                robotCmd.dirB = '1';
                break;
            case (' '): // If space bar is pressed, stop the robot by setting power to zero.
                robotCmd.pwmA = '0';
                robotCmd.pwmB = '0';
                break;
        }

        // Emit the robot command via the userEmitter.
//         this.userEmitter.robotCmd({
//             hostname: "user", // Specify the user as the sender.
//             dest: 'robot1', // The destination robot (robot1).
//             cmd: robotCmd // The command to be sent to the robot.
//         });
    }

    // Function to stop the robot by sending a stop command.
    stopRobot() {
        const stopCmd = {
            hostname: 'user', // The sender.
            dest: 'robot1', // The destination robot.
            cmd: {
                dirA: '0', // Set the left wheel direction to stop.
                pwmA: '0', // Set the left wheel power to zero.
                dirB: '0', // Set the right wheel direction to stop.
                pwmB: '0', // Set the right wheel power to zero.
            }
        };

        console.log("Sending stop command"); // Log the stop command (for debugging purposes).
        this.userEmitter.robotCmd(stopCmd); // Emit the stop command via the userEmitter.
    }

    // Function to move the robot at a specific heading (direction) for a set duration.
    moveRobotAtHdg(targetHdg, duration) {
        console.log(`Moving robot to maintain heading: ${targetHdg}`); // Log the target heading.
        duration = (duration && duration > 0) ? duration * 1000 : 1000; // Set the duration in milliseconds (default is 1 second).

        const robotCmd = {
            hostname: 'user', // The sender.
            dest: 'robot1', // The destination robot.
            cmd: {
                dirA: '0', // Default direction for left wheel.
                pwmA: '1', // Power for left wheel.
                dirB: '0', // Default direction for right wheel.
                pwmB: '1', // Power for right wheel.
            }
        };

        let interval = 1000; // Set the interval for checking the heading (every 1 second).
        let timePassed = 0; // Track the time passed.

        // Set up an interval to check and adjust the robot's heading.
        let id = setInterval(() => {
            timePassed += interval; // Increment the time passed.
            let currentHdg = this.currentHdg.data; // Get the current heading of the robot.

            console.log(`Current heading: ${currentHdg}, Target heading: ${targetHdg}`); // Log current and target headings.

            // Adjust the robot's heading based on the difference between the current and target headings.
            this.adjustHeading(currentHdg, targetHdg, robotCmd);
            this.userEmitter.robotCmd(robotCmd); // Emit the adjusted robot command.

            // Check if the total time has reached the duration. If so, stop the robot.
            if (timePassed >= duration) {
                clearInterval(id); // Clear the interval to stop checking.
                this.stopRobot(); // Stop the robot.
            }
        }, interval);
    }

    // Function to adjust the robot's heading based on the difference between the current and target heading.
    adjustHeading(currentHdg, targetHdg, robotCmd) {
        const headingDifference = targetHdg - currentHdg; // Calculate the heading difference.

        // If the heading difference is positive, adjust the robot to the right.
        if (headingDifference > 0) {
            console.log('Adjusting to the right');
            robotCmd.cmd.dirA = '1'; // Set left wheel direction to turn right.
            robotCmd.cmd.dirB = '1'; // Set right wheel direction to turn right.
            robotCmd.cmd.pwA = '1'; // Set power for left wheel.
            robotCmd.cmd.pwB = '1'; // Set power for right wheel.

        // If the heading difference is negative, adjust the robot to the left.
        } else if (headingDifference < 0) {
            console.log('Adjusting to the left');
            robotCmd.cmd.dirA = '0'; // Set left wheel direction to turn left.
            robotCmd.cmd.dirB = '0'; // Set right wheel direction to turn left.
            robotCmd.cmd.pwA = '1'; // Set power for left wheel.
            robotCmd.cmd.pwB = '1'; // Set power for right wheel.

        // If the heading is correct, move the robot straight forward.
        } else {
            console.log('Heading is correct, moving forward');
            robotCmd.cmd.dirA = '1'; // Set left wheel to move forward.
            robotCmd.cmd.dirB = '0'; // Set right wheel to move forward.
            robotCmd.cmd.pwA = '1'; // Set power for left wheel.
            robotCmd.cmd.pwB = '1'; // Set power for right wheel.
        }
    }
}

export { RobotController }; // Export the RobotController class for use in other parts of the application.


// my code ends here 