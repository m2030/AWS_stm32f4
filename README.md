# AWS_stm32f4

this show how to connect your stm32f4 discovery board to amazon cloud 
change common-> AWS->Src->aws_subscribe_publish_sensor_values.c const  char CA_CREDIATION_AWS[] in the top of the file to your accrediation 
change variable common-> AWS->Src->aws_subscribe_publish_sensor_values.c const  char CLIENT_CERTIFICATION[]= in the top of the file to your certification 
change variable common-> AWS->Src->aws_subscribe_publish_sensor_values.c  const  char CLIENT_PRIVATE_KEY[]= in the top of the file to your key 
change define  AWS_IOT_MQTT_HOST in the common-> AWS->Inc->aws_iot_config.h into your host name 
change define  AWS_IOT_MQTT_CLIENT_ID    in the common-> AWS->Inc->aws_iot_config.h into your client id 
change define  AWS_IOT_MY_THING_NAME     in the common-> AWS->Inc->aws_iot_config.h into your things name
change define  AWS_IOT_ROOT_CA_FILENAME     in the common-> AWS->Inc->aws_iot_config.h into yours
change define  AWS_IOT_CERTIFICATE_FILENAME     in the common-> AWS->Inc->aws_iot_config.h into yours
change define  AWS_IOT_PRIVATE_KEY_FILENAME     in the common-> AWS->Inc->aws_iot_config.h into yours
