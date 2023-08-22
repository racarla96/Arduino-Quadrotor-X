# Arduino-Quadrotor-X



Version portable del editor con las librerias necesarias para la compilación y ejecución del código.



## Conclusiones extraídas sobre el uso de la MPU6050 con Arduino Nano (atmega 328p) [08/05/2023]

Al utilizar el DMP con el Arduino el sensor es capaz de proporcionar los datos a una velocidad de 200 Hz pero con problemas de cortes en los datos, se para la comunicación de datos inexplicablemente, durante unos segundos o minutos.

He intentado buscar la solución en algunos foros, pero no está claro, (21/08/2023 - parece ser que el protocolo de I2C implementado por por la DMP o la MCU no es totalmente compatible con el estandar I2C y a veces no contesta... según algún foro leído), y tampoco queda sea por falta de capacidad del MCU. Con una teensy 4.0 parece que si que es capaz, pero tampoco está claro que sea cambiar el MCU por uno de mayor capacidad sea la solución.

Los datos de los ángulos se consiguen con éxito y funciona bastante bien el sensor. Queda la problemática de distinguir entre los datos del YawPitchRoll (Gravedad) y Euler; utiliza el vector gravedad y en el otro no. A 100 Hz en Arduino Nano parece que es capaz y sin problemas de funcionamiento.

Y con respecto a la cuestión de averiguar si la velocidad angular se puede obtener paralelamente sin usar el DMP a una mayor frecuencia saliendo directamente de los registros sin venir del paquete del DMP, si es posible obtener a diferente frecuencia hasta 400Hz para que si es capaz, con algún fallo temporal. Así, se podría plantear un control a mayor frecuencia con respecto a los 100 Hz. Entonces, por ser conservadores ejecutaremos el control a 200Hz. 

El máximo vendría marcado por los 490Hz que podrían soportar los ESCs, pero no me parece un número muy redondo, lo cuál dejarlo en 200Hz y trabajar con múltiplos de 100Hz me parece más cómodo y no supondría una ventaja considerable. Además, que queda parte del código de control por implementar lo cúal introducirá mayor carga computacional.
