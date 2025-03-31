// my code begins here 
import { reactive, inject } from "vue"; // Importing Vue's reactive API for state management and inject for dependency injection.
import { io } from "socket.io-client"; // Importing socket.io-client to establish a WebSocket connection.

const port = 80; // The port on which the server is running.
const URL = `http://localhost:${port}`; // The WebSocket server URL.

// // Reactive object to track connected clients' hostnames and IDs.
// const clients = reactive({
//     hostnames: [], // List of connected hostnames.
//     ids: [] // List of client IDs.
// });

// Reactive state object to manage connection state, events, and server messages.
// const state = reactive({
//   connected: false, // Connection status to the WebSocket server.
//   nodeEvents: new Map(), // Map to store events related to nodes (devices with "node" prefix).
//   robotEvents: new Map(), // Map to store events related to robots.
//   serverEvents: [], // List of server events received.
// });

// Function to check if a hostname starts with "node".
// const checkName = (value) => {
//     return value.startsWith("node"); // Returns true if the hostname starts with "node".
// };

// // Establish WebSocket connection to the server using socket.io-client.
// const socket = io(URL);

// // Event listener for when the client successfully connects to the WebSocket server.
// socket.on("connect", () => {
//   state.connected = true; // Update the connection status.
//   socket.emit('connection_data', { hostname: 'user' }); // Emit an event to notify the server that the user has connected.
//   console.log(state); // Log the state for debugging.
// });

// // Event listener for when the client disconnects from the WebSocket server.
// socket.on("disconnect", () => {
//   state.connected = false; // Update the connection status to disconnected.
// });

// // Event listener for receiving a message from the server.
// socket.on("message", async (msg) => {
//     console.log("private msg rcvd:"); // Log message received.
//     console.log(msg); // Log the message content.
// });

// // Event listener for receiving forwarded data from other nodes/robots via the server.
// socket.on('forwarded_data', (msg) => {
//     let hostname = msg.hostname; // Extract the hostname of the sender.
//     let dest = msg.dest; // Extract the destination.
//     let data = msg.data; // Extract the data sent by the sender.

//     try {
//         // If the hostname is in the node events map (i.e., it's a node), process the data.
//         if (state.nodeEvents.has(hostname)) {
//             let hostname = data.hostname; // Get hostname from data.
//             let events = state.nodeEvents.get(hostname); // Get the events for this hostname.
//             events.push(data); // Add new data to the events list.
            
//             // If the events list exceeds 1000 items, trim the list to keep the last 500 events.
//             if (events.length > 1000) {
//                 events = events.slice(500, 1000); // Keep only the last 500 events.
//                 state.nodeEvents.set(hostname, events); // Update the events map for this hostname.
//             }
//         }
//         // If the hostname is in the robot events map, process robot-related data.
//         else if (state.robotEvents.has(hostname)) {
//             let events = state.robotEvents.get(hostname); // Get the events for this hostname.
//             events.push(data); // Add new data to the events list.
//             state.robotEvents.set(hostname, events); // Update the events map for this hostname.
//         }
//     } catch (e) {
//         console.log(e); // Log any errors that occur.
//     }
// });

// // Event listener for receiving the network configuration from the server (e.g., list of clients).
// socket.on("network_configuration", async (data) => {
//     clients.hostnames = await data; // Update the list of hostnames from the server.

//     clients.hostnames.forEach((value) => {
//         // If the hostname is not already in nodeEvents and starts with "node", add it.
//         if (!state.nodeEvents.has(state.nodeEvents, value) && checkName(value)) {
//             state.nodeEvents.set(value, []); // Initialize an empty list for the node's events.
//         }
//         // If the hostname is not already in robotEvents, add it.
//         else if (!state.robotEvents.has(state.robotEvents, value)) {
//             state.robotEvents.set(value, []); // Initialize an empty list for the robot's events.
//         }
//     });
// });

// // `userEmitter` object for sending commands to nodes/robots from the client.
// const userEmitter = {
//     socket: null, // Placeholder for the socket connection.

//     // Initialize the userEmitter with the socket connection.
//     initialize(socket) {
//         this.socket = socket; // Store the socket connection.
//     },
    
//     // Function to send commands to nodes.
//     nodeCmd: function (data) {
//         if (this.socket) {
//             this.socket.emit("user_cmd", data); // Emit a user command to the server.
//             console.log("nodecmd emitted from user"); // Log the event for debugging.
//         } else {
//             console.error("no socket initialised"); // Error if the socket is not initialized.
//         }
//     },

//     // Function to send commands to robots.
//     robotCmd: function (data) {
//         if (this.socket) {
//             this.socket.emit("user_cmd", data); // Emit a user command to the server.
//         } else {
//             console.error("no socket initialized"); // Error if the socket is not initialized.
//         }
//     }
// };

// // Initialize the userEmitter with the socket connection.
// userEmitter.initialize(socket);    

// // Export the userEmitter, clients, state, and socket so they can be used in other components or files.
// export { userEmitter, clients, state, socket };

// my code ends here
