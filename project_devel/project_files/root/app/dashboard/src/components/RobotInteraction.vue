<!-- my code begins here -->
<script setup>

import {inject,reactive,watch,onMounted } from 'vue'

var axiosInstance = inject("axios")
var socket = inject("socket")
var userEmitter = inject("userEmitter")
var state = inject("state")
const incomingData = inject("robotData")
const props = defineProps(['robotId','robotHostname'])
const robotData = reactive({data:{}})


watch(
  () => state.robotEvents, 
  (newState) => {
    if (newState.size > 0 && newState.has(props.robotHostname))
    {
      const eventsArray = newState.get(props.robotHostname); 
      if (eventsArray.length > 0) 
      {
        robotData.data = eventsArray[eventsArray.length - 1];

      }
    }
   
  },

  { deep: true, immediate: true }
)

</script>


<template>
<div class="db-monitor">
    <h1>Sound detection</h1>
    <div v-if="robotData.data">
      <p>Current dB Level: {{ robotData.data.db }} dB</p>
      <progress :value="robotData.data.db" min="0" max="1024"></progress>
      <div :class="{'noise-detected': robotData.data.flag == 1, 'no-noise': robotData.data.flag == 0}">
        Noise Detected: {{ robotData.data.flag == 1 ? 'Yes' : 'No' }}
      </div>
    </div>

  
    <div v-else>
      <p>Waiting for microphone access...</p>
    </div>


  </div>
</template>


<style scoped>
.db-monitor {
  text-align: center;
  margin-top: 50px;
}

progress {
  width: 100%;
  height: 20px;
  margin-top: 10px;
}

.noise-detected {
  color: red;
  font-weight: bold;
}

.no-noise {
  color: green;
  font-weight: bold;
}
</style>
<!-- my code ends here -->