<!-- my code begins here -->
<script setup>

import {inject,ref,watch,onBeforeUnmount,onMounted } from 'vue'
import { onKeyStroke } from '@vueuse/core'
import bootstrap from "bootstrap/dist/js/bootstrap.js"
import ROSLIB from 'roslib';
import mqtt from 'mqtt';
const ros = inject("ros")
const props = defineProps(['robotName','robotId'])
const controllerTemplate = inject("robotController")
const status = ref(''); // Status message for UI feedback
/* Create a robot controller instance */
const robot = new controllerTemplate({props})
// MQTT client instance
let client = null;

// Publish a message to the MQTT broker
onMounted(() => {
  // Connect to MQTT broker (WebSocket)
  client = mqtt.connect('mqtt://192.168.1.241', {
    port: 9090,
    username: "user",
    password: "user",
  })

  client.on('connect', () => {
    status.value = `Connected to MQTT broker for ${props.robotName}`
    console.log('MQTT connection established:', props.robotName)
  })

  client.on('error', (err) => {
    status.value = `Connection error for ${props.robotName}`
    console.error('MQTT connection error:', err, props.robotName)
  })

  client.on('close', () => {
    status.value = `MQTT connection closed for ${props.robotName}`
    console.log('MQTT connection closed:', props.robotName)
  })
})

onBeforeUnmount(() => {
  // Clean up
  if (client) {
    client.end()
    console.log('MQTT connection closed:', props.robotName)
  }
})

// Listen for arrow keys + space to teleop
onKeyStroke(['ArrowDown','ArrowUp','ArrowLeft','ArrowRight',' '], (e) => {
  e.preventDefault()

  // Default: STOP
  let robotCmd = {
    cmd:"MV_CMD",
    id: props.robotId, // set this so the robot also knows which ID it is
    dirA: '0',
    pwmA: '0',
    dirB: '0',
    pwmB: '0'
  }

  switch(e.key) {
    case 'ArrowUp':    // forward
      robotCmd.dirA = '0'; robotCmd.pwmA = '1'
      robotCmd.dirB = '0'; robotCmd.pwmB = '1'
      break
    case 'ArrowDown':  // backward
      robotCmd.dirA = '1'; robotCmd.pwmA = '1'
      robotCmd.dirB = '1'; robotCmd.pwmB = '1'
      break
    case 'ArrowLeft':  // turn left
      robotCmd.dirA = '1'; robotCmd.pwmA = '1'
      robotCmd.dirB = '0'; robotCmd.pwmB = '1'
      break
    case 'ArrowRight': // turn right
      robotCmd.dirA = '0'; robotCmd.pwmA = '1'
      robotCmd.dirB = '1'; robotCmd.pwmB = '1'
      break
    case ' ':          // stop
      // already set to 0’s
      break
  }

  publishCmd(robotCmd)
})


function sendCmd(action) {
  let robotCmd = {
    cmd:"MV_CMD",
    id: props.robotId,
    dirA: '0', pwmA: '0',
    dirB: '0', pwmB: '0'
  }

  switch(action) {
    case 'forward':
      robotCmd.dirA = '0'; robotCmd.pwmA = '1'
      robotCmd.dirB = '0'; robotCmd.pwmB = '1'
      break
    case 'backward':
      robotCmd.dirA = '1'; robotCmd.pwmA = '1'
      robotCmd.dirB = '1'; robotCmd.pwmB = '1'
      break
    case 'left':
      robotCmd.dirA = '1'; robotCmd.pwmA = '1'
      robotCmd.dirB = '0'; robotCmd.pwmB = '1'
      break
    case 'right':
      robotCmd.dirA = '0'; robotCmd.pwmA = '1'
      robotCmd.dirB = '1'; robotCmd.pwmB = '1'
      break
    case 'stop':
    default:
      // stops (all 0)
      break
  }

  publishCmd(robotCmd)
}

/** Publish our JSON robotCmd to the correct topic. */
function publishCmd(cmdObj) {
  if (!client || !client.connected) {
    status.value = `Not connected (robot: ${props.robotName})`
    console.warn('Cannot publish, MQTT disconnected:', props.robotName)
    return
  }

  // Use the robotId from props to build the topic dynamically
  let topic = `painlessMesh/to/${props.robotId}`
  let msg = JSON.stringify(cmdObj)

  client.publish(topic, msg, (err) => {
    if (err) {
      status.value = `Failed to publish to ${topic}`
      console.error('Publish error:', err, ' for robot:', props.robotName)
    } else {
      status.value = `Cmd published to ${props.robotName} (${topic})`
      console.log(`Published cmd to ${props.robotName} =>`, msg)
    }
  })
}

</script>


<template>
  <div class="controller">
    <h5>{{ robotName }} Status: {{ status }}</h5>


    <div class="row mb-2">
      <div class="col text-center">
        <button class="btn btn-primary" @click="sendCmd('forward')">
          Forward
        </button>
      </div>
    </div>

    <div class="row mb-2">
      <div class="col text-end">
        <button class="btn btn-primary" @click="sendCmd('left')">
          Left
        </button>
      </div>
      <div class="col text-start">
        <button class="btn btn-primary" @click="sendCmd('right')">
          Right
        </button>
      </div>
    </div>

    <div class="row mb-2">
      <div class="col text-center">
        <button class="btn btn-primary" @click="sendCmd('backward')">
          Backward
        </button>
      </div>
    </div>

    <div class="row">
      <div class="col text-center">
        <button class="btn btn-danger" @click="sendCmd('stop')">
          Stop
        </button>
      </div>
    </div>
  </div>
</template>

<style scoped>

        body {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            background-color: #f8f9fa;
        }
        .controller {
            text-align: center;
            
        }
        .btn-robot {
            /* width: 100%; */
             justify-content: center;
            align-items: center;
            padding: 15px;
            font-size: 18px;
        }
        .control-row {
            margin-bottom: 10px;
        }
        .control-row .col {
            padding: 0 5px;
        }
    </style>
<!-- my code ends here -->