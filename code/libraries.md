# Links:

- [i2cdevlib - I2Cdev y MPU 6050 DMP](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
- [timoxd7 - FastRCReader](https://github.com/timoxd7/FastRCReader)

# ¿Como modificar la libería para cambiar la frecuencia del PWM?

Por defecto, la librería Servo emplea 50Hz - 20000 us. Si queremos cambiar el periodo porque nuestro ESC es capaz de aceptar una mayor frecuencia, podemos modificar la libería Servo.h.

Encuentra el fichero Servo.h de la librería Servo del editor o edita **\editor\arduino-1.8.19-windows\arduino-1.8.19\portable\sketchbook\libraries\Servo_200Hz\src**. Librería Servo_200Hz ya modificada para funcionar según los valores de la imagen de abajo.

El parametro REFRESH INTERVAL indica en microsegundos (us) el periodo del PWM. 

![](.\imgs\Servo_Change_Hz.png)