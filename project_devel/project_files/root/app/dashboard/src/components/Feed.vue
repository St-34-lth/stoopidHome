<!-- my code begins here -->
<template>
<div class="video-container">
    <h3>Live Camera Feed</h3>
    <img :src="annotatedImgStreamURL" autoplay controls width="640" height="480"></img>
  </div>
  <!-- <div class="video-container">
    <h3>Raw Camera Feed</h3>
    <img :src="rawImgStreamURL" autoplay controls width="640" height="480"></img>
  </div> -->
</template>

<script setup>
import { onMounted, ref, reactive,inject,watch } from 'vue';
import ROSLIB from 'roslib';
// const topics = inject("createdTopics")
// import { inject, ref, onMounted, watch } from 'vue';
  
const topics = inject("createdTopics")

// const rawImgStreamURL = ref('http://localhost:8080/stream?topic=/camera/image_raw&width=640&height=480&quality=90');
const annotatedImgStreamURL = ref('http://192.168.1.241:9091/stream?topic=/camera/annotated_image&width=640&height=480');
const segImgStreamURL = ref('http://192.168.1.241:9091/stream?topic=/camera/segmented_image&width=640&height=480');

// Helper function to convert ROS Image message to base64 image


onMounted(() => {
      
      

})

watch(
  () => topics, // Watch robotEvents map
  (newTopics) => {

          
          Object.keys(newTopics).forEach((key) => {
                    topics[key] = newTopics[key]
          })

      }
        ,
  { deep: true, immediate: true }
);

</script>

<style scoped>
.video-container {
  display: flex;
  justify-content: center;
  align-items: center;
  flex-direction: column;
  margin: 20px;
}

video {
  border: 2px solid #ccc;
  border-radius: 10px;
}
</style>
<!-- my code ends here -->
