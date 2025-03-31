import {default as axios} from 'axios';
const port = 80
const url = `http://localhost:${port}`

const caller = axios.create({
  baseURL: url,
  timeout: 1000,
});

export default {
  install: (app) => {
    const axiosInstance = caller
    const pathUrl = url 
    app.provide('axios',axiosInstance)
    app.provide('url',pathUrl )
  },
};
