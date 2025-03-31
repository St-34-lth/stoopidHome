// <!-- my code begins here -->
import './assets/main.css'
import 'bootstrap/dist/css/bootstrap.css'
import { createApp } from 'vue'

import {ros} from './plugins/ros'
// import axiosPlugin from './plugins/axios'
import App from './App.vue'
import router from './router'
// import {state , socket} from './plugins/socket'
import bootstrap from "bootstrap/dist/js/bootstrap.js"
// import VueDatePicker from '@vuepic/vue-datepicker';
import {RobotController} from './plugins/robot'
const app = createApp(App)
app.provide("robotController",RobotController)

app.use(ros)
app.provide("ros",ros)
app.use(router)
app.use(bootstrap)


app.mount('#app')
// <!-- my code ends here -->