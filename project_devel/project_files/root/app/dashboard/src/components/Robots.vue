<!-- my code begins here -->


<script setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import mqtt from 'mqtt'
import RobotNavigate from './RobotNavigate.vue'

// Tabs
const activeTab = ref('robot1') // Start on "robot1" tab


const topic = ref('test/topic')
const message = ref('')
const status = ref('')
let client = null

onMounted(() => {
  client = mqtt.connect('mqtt://192.168.1.241', {
    port:9090,
    username:"user",
    password:"user"
  })
  client.on('connect', () => {
    status.value = 'Connected to MQTT broker'
    console.log('MQTT connection established')
  })
  client.on('error', (err) => {
    status.value = 'Connection error'
    console.error('MQTT connection error:', err)
  })
  client.on('close', () => {
    status.value = 'Connection closed'
    console.log('MQTT connection closed')
  })
})

onBeforeUnmount(() => {
  if (client) {
    client.end()
    console.log('MQTT connection closed')
  }
})
</script>

<template>
  <div>
    <!-- TABS NAV -->
    <ul class="nav nav-tabs">
      <li class="nav-item">
        <button 
          class="nav-link"
          :class="{ active: activeTab === 'robot1' }"
          @click="activeTab = 'robot1'"
        >
          Robot 1
        </button>
      </li>
      <li class="nav-item">
        <button 
          class="nav-link"
          :class="{ active: activeTab === 'robot2' }"
          @click="activeTab = 'robot2'"
        >
          Robot 2
        </button>
      </li>
    </ul>

    <div v-if="activeTab === 'robot1'">
      <RobotNavigate robotName="robot1" :robotId="2382067720" />
    </div>
    <div v-else-if="activeTab === 'robot2'">
      <RobotNavigate robotName="robot2" :robotId="2811215833" />
    </div>
  </div>
</template>
<style scoped>
.nav-link.active {
  background-color: #0d6efd;
  color: #fff;
}
</style>


