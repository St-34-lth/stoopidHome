This repo is used to store the source code of the stoopidHome system. This is still a work in progress. 

The system uses docker services for each of its components. A main compose file is used to stage all services. 

If you wish to build them (ros and iot), use the included docker build files.
These may be still unstable however so if you are interested in running the services, visit the below docker hub or use the compose to pull the images
automagically. You will need to download this repo and run the compose.yaml either way.

https://hub.docker.com/repository/docker/st34lths0/iot_dev/general

use the below images:

ros_final
iot_dev_final
ha_final
mqtt_final
