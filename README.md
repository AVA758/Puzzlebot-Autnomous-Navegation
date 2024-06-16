# Puzzlebot-Autnomous-Navegation

El proyecto consiste en un robot diferencial “Puzzle Bot” que  debe mantenerse en una pista contenida por 2 líneas negras y a manera de guía una línea negra central; el “Puzzle Bot” debe ser capaz además de  reconocer señales de tráfico y colores de semáforos mediante algoritmos de visión computacional y redes neuronales. Este robot diferencial está compuesto por una cámara, una Jetson Nano y la tarjeta Hacker Board de Manchester Robotics. 

Se hace uso de ROS2 para la comunicación entre nodos para el control del robot y para el procesamiento de imágenes con OpenCV y YOLOv8. Las pruebas se realizaron en una pista planteada, configurando el robot para detectar colores y objetos, y utilizando máquinas de estado para gestionar su comportamiento.

Los algoritmos empleados permitieron una interacción efectiva con el entorno, logrando que el robot diferencial recorriera la pista con precisión y a la vez detectando señales de tráfico y semáforos con un buen índice de confianza. 

Para utilizar este proyecto, es necesario descargar todos los archivos del repositorio. 
De igual manera, es necesario utilizar Ubuntu en el sistema operativo de Linux, pues el sistema se ejecutva mediante ROS2 (Robot Operating System)

A continuación se da una explicación de cada código.
- final.py: Este es el código principal del sistema, donde se encuentran las suscripicones a los tópicos de las redes neuronales y los algoritmos de movimiento del robot.
- yolov8_semaforo.py: Este código es para tomar la lectura de imagen de la cámara y permitir el reconocimiento del color de estado del semáforo, para después ser publicado en el tópico de 'traffic_color', que será usado por final.py
- semaforos.pt: Es el modelo de PyTorch utilizado para el machine learning de los semáforos de este sistema.
- yolov8_recognition: Este código es para tomar lalectura de imagen de la cámara y permitir el reconocimiento de de las 6 señales de tránsito de este sistema (seguir derecho: 'frente', vuelta a la derecha: 'right', vuelta a la izquierda 'left', alto: 'stop', ceda el paso: 'giveway' y obra en proceso: 'progress'), esto para que la señal correspondiente sea publicada en un String en el tópico de 'signal_order', que será usado en final.py
- actual2.pt: Es el modelo de Pytorch utilizado para el machine learning de las señales de transito de este sistema.

Una vez que estos archivos hayan sido descargados, es necesario crear un paquete utilizando ROS2, donde todos los archivos menos final.py deben encontrarse.

Después, se debe conectar con el Puzzlebot.
De haberse configurado correctamente, se podrá conectar la computadora a la red de la Jetson, dispositivo donde será necesario subir el archivo de final.py (y solo este archivo) para permitir un procesamiento más rápido.
Es importante mantener conexión con la red de la Jetson, de lo contrario, ningún archivo se ejecutará correctamente.

Para conectarse al dispositivo Jetson, en la terminal se ingresa el comando
ssh pi@(ingresar dirección IP de la Jetson)

Se pedirá que ingrese la contraseña.

Una vez dentro, se deberá crear un archivo para ingresar el código de final.py. Esto se logra con los comandos
touch final.py
chmod +x final.py
nano final.py

Después de este último comando, se abrirá un espacio en blanco, aquí coloque mediante copy-paste el contenido de final.py, guarde y cierre el archivo.

Después de eso, dentro de los comando para la jetson (el usuario en la terminal distingue si está dentro de la jetson o no), deberá correr el comadno de microROS para que la jetson en el puzzlebot pueda empezar a ejecutarse. 

De igual forma, en otra terminal, se debe acceder al paquete de ROS2 crado y correr el comando
python3 yolov8_recognition.py

Esto permitirá que, usando el archivo de actual2.pt, se ejecute la red neuronal para el reconocimiento de señales, pues está suscrito a la información que regresa la cámara.

En otra terminal, con el mismo paquete accedido, se debe correr el comando
python3 yolov8_semaforo.py

Esto permitirá que, usando el archivo de semaforos.py, se ejecute la red neuronal para el reconocimiento del estado de los semáforos.

Por esto, es importante que los 4 archivos de la red neuronal se encuentren en el mismo paquete. 

Antes de correr el puzzlebot, se debe asegurar que se encuentre en la pista de este proyecto para que funcione como se ha reportado.

Finalmente, en la terminal dentro de la Jetson, en la localidad donde se guardó el archivo de final.py, se ejecuta el siguiente comando:
python3 final.py

Con los tres codigos ejecutados, usando los modelos de pytorch, el robot Puzzlebot emepzará amoverse como se ha descrito en el reporte. 

Para cerrar las ejecuciones, se presiona ctrl c en todas las terminales. Para salir del apartado de la Jetson, se escribe exit en la terminal. 

Finalmente, para apagar el puzzlebot, se deberá mantener presionado el botón de la batería durante unos 5 segundos para que este se apague y deje de suministrar energía a la hackerboard y a la Jetson. De querer volverlo a prender, se presiona de nuevo el botón. 

Se recomienda correr el sistema en tres computadoras, una para la red de señales de tránsito (con su respectivo modelo de pytorch), otra para la red de semáforos (con su respectivo modelo de pytorch) y otra para el código de navegación. 
