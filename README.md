[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6883153&assignment_repo_type=AssignmentRepo)

[![GitHub Action
Status](https://github.com/Docencia-fmrico/follow-wall-grupo-dia-libre/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/follow-wall-grupo-dia-libre)


# follow_wall

Group Members:

Luis Miguel López Martín |
Juan Camilo Carmona Sanchez |
Guillermo Bernal Ruiz |
Jose Manuel Tostado Felipe


El proposito de este repositorio es usar el robot tiago en el simulador gazebo y a través del uso del sensor laser para distancias y el modulo de velocidad poder adherirse a una pared a cierta distancia y seguirla indefinidamente.

La idea para poder conseguir el objetivo se basa en dos posibles casos:

- El caso inicial donde el robot este donde este, busque una pared a la que poder seguir.
- El segundo caso, que es el caso donde mas tiempo va a estar el robot. Tiene que seguir una pared a cierta distancia y ser capaz de girar correctamente en las esquinas.

El caso inicial funciona bastante sencillo, el robot parta donde parta, avanzara en linea recta hasta encontrar una pared (no distingue pared de objeto, asi que si encuentra un objeto antes que una pared la tratara con esta última). Mientras avanza, lee un rango pequeño del laser de la posicion central, para ir comprobando todo el rato el momento en el que el robot se encuentra a una distancia menor o igual que la distancia minima que le hemos impuesto para que empiece a girar.
El objetivo es que cuando detecte esta pared entre en modo giro, el modo giro trata de girar 90 grados para poder seguir la pared correctamente.
Como metodo de giro de angulos rectos hemos implementado un algoritmo que mira la tendencia que va siguiendo el laser del robot situado a su izquierda, de tal modo que mientras gira la distancia a la izquierda disminuye, por lo tanto sigue girando.


![image](https://user-images.githubusercontent.com/78974537/153905533-fa77956e-795f-4e36-857a-c5a23c536069.png)
![image](https://user-images.githubusercontent.com/78974537/153905553-399291d1-69a0-4aac-91a6-23b59a2e6119.png)
![image](https://user-images.githubusercontent.com/78974537/153905566-31e4d693-f34f-4e82-9459-9b4750143696.png)

Esto lo hacemos mediante medias de las ultima medidas para mirar que tendencia sigue. Ademas, en cada iteración el robot guarda la minima media encontrada, tratandola como el angulo mas recto encontrado.
Cuando el robot pasa el angulo recto esta distancia del laser de la izquierda crece, por lo tanto si es mayor a la distancia minima, gira en sentido contrario en busca de encontrar el minimo.
Entonces puede: vuelver a crecer la distancia o encontrar la menor, en el primer caso hace lo mismo que en el anterior, girar al contrario (con un maximo de recalculo para que no sea un balanceo) y si es menor que el menor medido el robot considera que esta en el angulo mas recto posible.


![image](https://user-images.githubusercontent.com/78974537/153907394-751cb79e-68b6-446f-a024-e53bfa754733.png)


(En este caso gira al contrario para buscar la distancia minima)

De aqui pasariamos al segundo caso; el segundo caso empieza cuando el robot ya ha girado 90 grados hacia a la derecha y empieza la navegacion siguiendo la pared.
Para poder seguir manteniendo una distancia de respeto a medida que avanza el robot, damos por hecho que en el caso anterior ha habido un error de angulo seguro, con lo cual, a medida que avance el robot, este error de direccion se incrementara.
Entonces este seguimiento lo hacemos mediante un control PD, con la idea de mantenerse en la linea que marca la distancia minima hasta encontrarse con una esquina y tener que girar.
El giro puede ser de dos casos: a la izquierda o a la derecha.

Para la izquierda el control PD se encarga de todo con la ayuda de la medida del laser de la izquierda del robot, insertando las velocidades lineales y angulares necesarias, ya que no hay pared al frente.

![image](https://user-images.githubusercontent.com/78974537/153905796-f1df55af-d8ae-405f-8da1-7ec2c11f1e63.png)

Para girar a la derecha hay que seguir la misma idea que el caso inicial, donde va a ser un caso en el que hay una pared al frente del robot, por lo tanto el PD no puede hacer mucho aqui.

![image](https://user-images.githubusercontent.com/78974537/153905817-35d763d1-2f0e-4321-bbac-a7978695d7ce.png)

Pero hay que ignorar la primera vez que cambia la tendencia de la distancia del laser porque seguramente se trate de la esquina, por lo tanto ignoramos las primeras iteraciones del giro, para evitar esto.


![image](https://user-images.githubusercontent.com/78974537/153906804-d9d21521-7fcc-450a-9579-01df374c473e.png)


Y con estos dos casos cubiertos el robot debe navegar correctamente en el entorno que desee siguiendo una pared a cierta distancia.
