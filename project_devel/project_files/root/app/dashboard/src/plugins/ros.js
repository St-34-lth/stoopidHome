// <!-- my code begins here -->
import { ref, reactive } from 'vue'; // Importing ref and reactive from Vue.js for reactive data handling.
import ROSLIB from 'roslib'; // Importing ROSLIB to interface with ROS over WebSocket.

const url = 'ws://localhost:9090'; // URL of the ROS WebSocket server.
const ros = new ROSLIB.Ros({
  url: url, // Creating a ROS connection using the WebSocket URL.
});

const robotData = reactive({data:{}}); // Reactive object to store data from the /imu_data topic.
const connectionStatus = ref('connecting'); // Reactive reference to store the current connection status.
const topics = ref([]); // Reactive reference to store the list of available ROS topics.
const robotHeading = reactive({data:{}}); // Reactive object to store data from the /robot_heading topic.
const createdTopics = reactive({}); // Reactive object to store created topic subscriptions.
const robotSpeed = reactive({data:{}}); // Reactive object to store data from the /speed topic.

// URLs for image streams (raw and annotated). These point to a server that streams the images from topics.
const rawImgStreamURL = ref('http://localhost:9091/stream?topic=/camera/image_raw&width=640&height=480');
const annotatedImgStreamURL = ref('http://localhost:9091/stream?topic=/camera/annotated_image&width=640&height=480');

// Function to create a ROS topic subscription dynamically.
function createTopic(topic, msgType) {
    try {
        // Create a new ROSLIB topic and store it in the createdTopics object.
        createdTopics[topic] = new ROSLIB.Topic({
            ros: ros, // The ROS connection.
            name: topic, // The name of the ROS topic.
            messageType: msgType, // The type of the messages published on this topic.
        });
    } catch (error) {
        console.log(error); // Log any errors during topic creation.
    }
}

// Event listener for when the ROS connection is established.
ros.on("connection", async function() {
    console.log("Connected to ROS WebSocket.");
    connectionStatus.value = "connected"; // Update connection status to "connected".

    // Fetch available topics from the ROS master.
    ros.getTopics((publishedTopics) => {
        topics.value = publishedTopics.topics; // Store the list of topic names.
        let msgtypes = publishedTopics.types; // Get the corresponding message types for each topic.

        // Loop through the available topics and subscribe to the necessary ones.
        topics.value.forEach((topic, index) => {
            // Check if the topic is already subscribed (to avoid duplicate subscriptions).
            if (!(topic in createdTopics)) {
                let msgtype = msgtypes[index]; // Get the message type for this topic.

                // Create and store the topic subscription using the helper function.
                createTopic(topic, msgtype);

                // Subscribe to the /imu_data topic and handle the data.
                if (topic === "/imu_data") {
                    createdTopics[topic].subscribe((msg) => {
                        // Parse the message data as JSON and store it in robotData.
                        robotData.data = JSON.parse(msg.data); // Assuming msg.data is a JSON string.
                    });
                }
                // Subscribe to the /robot_heading topic and store the received float data.
                else if (topic === "/robot_heading") {
                    createdTopics[topic].subscribe((msg) => {
                        robotHeading.data = msg.data; // Store the robot heading (float) data.
                    });
                }
                // Subscribe to the /speed topic and store the speed data.
                else if (topic === "/speed") {
                    createdTopics[topic].subscribe((msg) => {
                        robotSpeed.data = msg.data; // Store the robot speed (float) data.
                    });
                }
                // Handle subscription to the /date topic.
                else if (topic === "/date") {
                    // Subscribe to /date topic and log the received date.
                    createdTopics[topic].subscribe((msg) => {
                        console.log("Date received:", msg.data); // Assuming it's a string message.
                    });
                }

                // Additional topic handling logic can be added here as needed.
            }
        });
    });
});

// Event listener for when there's an error with the ROS connection.
ros.on('error', (error) => {
  console.log('Error connecting to websocket server: ', error); // Log the error.
  connectionStatus.value = 'error'; // Update connection status to 'error'.
});

// Event listener for when the connection to the ROS server is closed.
ros.on('close', () => {
  console.log('Connection to websocket server closed.'); // Log that the connection has closed.
  connectionStatus.value = 'closed'; // Update connection status to 'closed'.
});

// Create a listener for the raw image data from a ROS topic.
// This will allow us to receive compressed images from the /camera/image/compressed topic.
var rawImageListener = new ROSLIB.Topic({
    ros: ros, // The ROS connection.
    name: '/camera/image/compressed', // The name of the topic we're subscribing to.
    messageType: 'sensor_msgs/CompressedImage', // The type of message expected from this topic.
});

// Export the necessary variables and objects to be used in other components or scripts.
export {ros, connectionStatus, topics, createdTopics, robotData, robotHeading, robotSpeed};


// <!-- my code ends here -->