<!-- my code begins here -->
<script setup>

import { ros } from '@/plugins/ros';
import {inject,reactive,watch,onMounted } from 'vue'

var axiosInstance = inject("axios")
var socket = inject("socket")
var userEmitter = inject("userEmitter")
var state = inject("state")
const incomingData = inject("robotData")
const props = defineProps(['robotId','robotHostname'])
const robotData = reactive({data:{}})


watch(
  () => incomingData, // Watch the reactive incomingData object
  (newState) => {
    if (newState && typeof newState === 'object' && 'data' in newState) {
      // Ensure newState contains 'data'
      Object.keys(newState.data).forEach((key) => {
        // Using Vue's reactivity system to update robotData.data
        robotData.data[key] = newState.data[key];
      });
    }
    // console.log(robotData); // Log the updated robotData for debugging
  },
  { deep: true, immediate: true }
);

// console.log(props)
// Fetch initial data when component is mounted

</script>


<template>
  <div class="card">
    <div class="card-header">
      <h5>Robot Data: {{ robotHostname }}</h5>
    </div>
    <div class="card-body">
      <div class="mb-2">
        <h6>Orientation</h6>
        <p>qw: {{ robotData.data?.qw }} | qx: {{ robotData.data?.qx }} | qy: {{ robotData.data?.qy }} | qz: {{ robotData.data?.qz }}</p>
      </div>
      
      <div class="mb-2">
        <h6>Gravity</h6>
        <p>g: {{ robotData.data?.g }}</p>
      </div>
      
      <div class="mb-2">
        <h6>Acceleration World</h6>
        <p>x: {{ robotData.data?.aaWorld?.x }} | y: {{ robotData.data?.aaWorld?.y }} | z: {{ robotData.data?.aaWorld?.z }}</p>
      </div>
      
      <div class="mb-2">
        <h6>Gyro World</h6>
        <p>x: {{ robotData.data?.ggWorld?.x }} | y: {{ robotData.data?.ggWorld?.y }} | z: {{ robotData.data?.ggWorld?.z }}</p>
      </div>
      
      <div class="mb-2">
        <h6>Angles</h6>
        <p>Yaw: {{ robotData.data?.yaw }} | Pitch: {{ robotData.data?.pitch }} | Roll: {{ robotData.data?.roll }}</p>
      </div>
    </div>
  </div>
</template>
<!-- my code ends here -->