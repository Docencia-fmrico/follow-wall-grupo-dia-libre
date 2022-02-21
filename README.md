[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6883153&assignment_repo_type=AssignmentRepo)

[![GitHub Action
Status](https://github.com/Docencia-fmrico/follow-wall-grupo-dia-libre/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/follow-wall-grupo-dia-libre)


# follow_wall

Group Members:

Luis Miguel López Martín |
Juan Camilo Carmona Sanchez |
Guillermo Bernal Ruiz |
Jose Manuel Tostado Felipe

Tras conseguir un funcionamiento relativamente decente en el simulador, ha habido que realizar cambios para conseguir el mismo funcionamiento en el robot real.

Primero que nada, al utilizar un Kobuki con el RPLidar hemos tenido que cambiar la manera de obtener las lecturas del laser. El laser del kobuki tiene 360 grados, por lo que hemos obtenido las siguientes lecturas:

![Nuevo_Laser](https://user-images.githubusercontent.com/78983070/154996376-ccd54ae5-3a5f-4a14-8e41-2305d022eb05.png)

Las lecturas de la izquierda y del centro se usan por el algoritmo utilizado para girar en esquinas cerradas (Obtención de medias) y las lecturas A y B las hemos introducido nuevas ya que hemos tenido que cambiar la manera de usar el PD.

Al utilizar el láser real también nos hemos encontrado un problema, había medidas infinitas y medidas _not a number_. Las medidas infinitas aparecían cuando el robot no era capaz de detectar nada al final del laser, y las _not a number_ aparecían cuando había un objeto dentro del rango de medidas descartadas del láser, aproximadamente a 30cm de radio del RPLidar. 

Para lidiar con ellas, a la hora de barrer una pequeña área en las medidas del laser, hemos decidido descartar ambas medidas utilizando un contador de fallos que luego se restarían a la cantidad de medidas totales tomadas para que la media no se viese afectada. En el caso de que todas las medidas fueran infinito o NaN, tomábamos la media de medidas (que en este caso sería 0/0), como una distancia fija.

Finalmente, hemos cambiado el algoritmo de obtención del error del controlador PD. Para ello, hemos obtenido dos nuevas distancias, a y b, a un ángulo α que será negativo hacia abajo y positivo hacia arriba.

![Nuevo_PD](https://user-images.githubusercontent.com/78983070/154997515-581e8d5c-03a6-40af-ab67-2c5510baeb5c.png)

Al ser el mismo ángulo α, si A y B son iguales, restar A - B tendrá resultado 0. Si A es mayor que B, la resta será positiva y si B es mayor que A, la resta será negativa.

De esta manera obtenemos un error con signo variante que nos permitirá ajustar el controlador PD dependiendo de cómo esté situado el robot respecto a una pared.

En este video vemos el funcionamiento del robot:

https://user-images.githubusercontent.com/78983070/154998547-5b4f9fc8-6c47-414b-b115-3e1a8a6c1db1.mp4

