# Proyecto Final: Robótica y Sistemas Autónomos 2025-01

**Integrantes:**
* Patricio Figueroa
* Marcelo Flores
* Kavon Kermani
* Gabriel Sanzana
* Lucas Zamora

**Software de Simulación:**
* Webots R2023a

## Descripción del Proyecto

El presente proyecto tiene como objetivo el desarrollo e implementación de un robot móvil autónomo de 4 ruedas en el simulador Webots. El sistema integra control cinemático diferencial y una variedad de sensores para la percepción y navegación en un entorno con obstáculos.

## Descripción del Robot y sus Características

El robot móvil autónomo desarrollado para este proyecto es una plataforma de 4 ruedas diseñada para operar en un entorno simulado dentro de Webots R2025a. Su principal objetivo es la navegación autónoma, la detección y evasión de obstáculos, y la planificación de rutas eficientes hacia un objetivo definido.

* Características Físicas y Cinemáticas
  
- Tipo de Robot: Robot móvil de 4 ruedas.

- Control de Movimiento: Utiliza control cinemático diferencial, lo que le permite ajustar las velocidades de las ruedas izquierdas y derechas (motor1, motor2, motor3, motor4 en Webots) para lograr movimientos de avance/retroceso y giros en el lugar o durante el desplazamiento.

- Dimensiones (referenciales del modelo Cuerpo y Rueda en mundo_virtual.wbt)

- Cuerpo: 0.2m x 0.1m x 0.05m (ancho, alto, profundidad).

- Ruedas: Cilindros de 0.04m de radio y 0.02m de altura.

- Sensores para Percepción del Entorno:
El robot está equipado con un conjunto de sensores que le permiten percibir su entorno y construir un mapa local:
   
   * LIDAR 2D: capaz de detectar obstáculos hasta 0.8 metros de distancia.
   * Sensores de Distancia 
   * GPS (Global Positioning System):
   * Unidad Inercial (IMU - Inertial Unit)

El robot es capaz de:
* Detectar obstáculos en tiempo real mediante la combinación de un sensor LIDAR y sensores de distancia.
* Construir un mapa de ocupación local del entorno a partir de los datos del LIDAR.
* Planificar rutas factibles desde un punto de inicio a una meta utilizando el algoritmo de planificación **RRT (Rapidly-exploring Random Trees)**.
* Navegar de manera autónoma hacia el objetivo, replanificando la ruta dinámicamente si el camino actual se encuentra bloqueado.

El desarrollo integra percepción, planificación y control en un entorno dinámico. El robot se prueba en un entorno controlado de **8x8** con obstáculos estáticos, evaluando su desempeño en términos de eficiencia de navegación, tiempo de ejecución y robustez en la evasión de obstáculos.

## Arquitectura del Software

El sistema de control del robot se organiza en tres niveles principales:

### Percepción

* **LIDAR 2D:**
    * Obtiene un escaneo del entorno para mapear los obstáculos.
    * Los datos se utilizan para actualizar un mapa de ocupación que informa al algoritmo de planificación.
* **Sensores de Distancia:**
    * Detectan obstáculos inminentes que podrían no haber sido capturados por el LIDAR en el ciclo anterior.
    * Actúan como una capa de seguridad para la evasión reactiva inmediata.
* **GPS e IMU (Inertial Unit):**
    * Obtienen la posición (coordenadas X, Y) y la orientación (yaw) del robot en el mundo, datos cruciales para la localización y el seguimiento de la ruta.

### Planificación

* **Mapa de Ocupación:**
    * Un mapa 2D del mundo conocido por el robot, donde las celdas se marcan como libres u ocupadas por obstáculos.
* **Algoritmo de Planificación RRT (Rapidly-exploring Random Trees):**
    * Encuentra un camino factible (no necesariamente el más corto) desde la posición actual hasta el objetivo, explorando aleatoriamente el espacio libre de obstáculos.
    * Se ejecuta una vez para encontrar una ruta inicial y se vuelve a invocar (replanificación) solo si la ruta actual se encuentra bloqueada por un nuevo obstáculo detectado.

### Control

* **Control de Navegación:**
    * Sigue la secuencia de puntos (waypoints) generada por el algoritmo RRT.
    * Un controlador proporcional ajusta la velocidad de las ruedas para minimizar el error de ángulo hacia el siguiente punto del camino.
    * Si la ruta se bloquea, detiene el seguimiento y solicita una replanificación al módulo de planificación.

Esta arquitectura modular permite una navegación robusta, combinando la planificación basada en muestreo (RRT) con la percepción continua del entorno.

## Diagramas del Sistema

Aquí se presentan los diagramas que representan el sistema del robot y su interacción con el entorno.

### Diagrama de Alto Nivel

![Diagrama de Alto Nivel del Robot](altoNivelRobot.png)

### Diagrama de Bajo Nivel (Arquitectura Interna o Flujo de Control)

![Diagrama de Bajo Nivel de Arquitectura](bajoNivelRobot.png)

### Diagrama de flujos

![Diagrama de flujos](DiagramDeFlujo.drawio.png)

### Pseudocódigo

```plaintext
INICIO

  // Inicializar Webots y dispositivos
  inicializar_simulador()
  motores ← inicializar_motores(4)      // 4 ruedas
  sensores_distancia ← inicializar_sensores_distancia(2)
  lidar ← inicializar_lidar()
  gps ← inicializar_gps()

  // Inicializar variables
  GRID_SIZE ← definir_tamaño_grid()
  grid ← crear_grid(GRID_SIZE, GRID_SIZE)
  path ← crear_array_path(MAX_PATH_LEN)
  tiempo_inicio ← obtener_tiempo_actual()
  last_metrics_time ← tiempo_inicio

  MIENTRAS simulador_esta_activo() HACER

    // --- Detección de obstáculos con sensores de distancia ---
    ds_detect_near ← FALSO
    PARA sensor EN sensores_distancia HACER
      lectura ← leer_sensor(sensor)
      SI lectura < UMBRAL_DISTANCIA ENTONCES
        ds_detect_near ← VERDADERO
      FIN SI
    FIN PARA

    // --- Obtener posición actual del GPS ---
    robot_x, robot_y ← leer_gps(gps)

    // --- Actualizar mapa con datos del LIDAR ---
    limpiar_grid(grid)
    lidar_detect_near ← FALSO
    escaneo ← obtener_puntos_lidar(lidar)
    PARA punto EN escaneo HACER
      SI es_valida(punto.distancia) Y punto.distancia < RANGO_MAX_LIDAR ENTONCES
        obs_x, obs_y ← calcular_pos_obstaculo(robot_x, robot_y, punto)
        cell_x, cell_y ← convertir_a_grid(obs_x, obs_y)
        SI dentro_del_grid(cell_x, cell_y, GRID_SIZE) ENTONCES
          grid[cell_x][cell_y] ← 1   // Marcar obstáculo
          SI punto.distancia < UMBRAL_CERCANO_LIDAR ENTONCES
            lidar_detect_near ← VERDADERO
          FIN SI
        FIN SI
      FIN SI
    FIN PARA

    // --- Marcar posición actual en el grid ---
    robot_cell_x, robot_cell_y ← convertir_a_grid(robot_x, robot_y)
    grid[robot_cell_x][robot_cell_y] ← 2   // Explorada

    // --- Planificación de ruta con A* ---
    start ← (robot_cell_x, robot_cell_y)
    goal ← (GRID_SIZE-2, GRID_SIZE-2)  // O definir stop_x, stop_y según objetivo
    path_length ← planificar_ruta_astar(grid, start, goal, path, MAX_PATH_LEN)

    // --- Control de movimiento ---
    SI lidar_detect_near O ds_detect_near ENTONCES
      // Girar para evitar obstáculo
      left_speed ← SPEED
      right_speed ← -SPEED
    SINO SI path_length <= 1 ENTONCES
      // Girar para explorar o buscar nueva ruta
      left_speed ← SPEED
      right_speed ← -SPEED
    SINO
      // Avanzar siguiendo el path
      left_speed ← SPEED
      right_speed ← SPEED
    FIN SI

    aplicar_velocidades_motores(motores, left_speed, right_speed)

    // --- Reporte de métricas cada 5 segundos ---
    tiempo_actual ← obtener_tiempo_actual()
    SI tiempo_actual - last_metrics_time >= 5 ENTONCES
      porcentaje_explorado ← calcular_porcentaje_explorado(grid)
      mostrar_métricas(
        tiempo_navegacion = tiempo_actual - tiempo_inicio,
        longitud_path = path_length,
        tiempo_planificacion = medir_tiempo_planificacion(),
        porcentaje_explorado = porcentaje_explorado,
        posicion_gps = (robot_x, robot_y),
        valores_sensores = obtener_valores_sensores(sensores_distancia, lidar),
        mapa_ascii = imprimir_grid_ascii(grid)
      )
      last_metrics_time ← tiempo_actual
    FIN SI

    // --- Condición de parada ---
    SI robot_cell_x == stop_x Y robot_cell_y == stop_y ENTONCES
      detener_motores(motores)
      terminar_programa()
    FIN SI

  FIN MIENTRAS

  limpiar_simulador()
FIN
```

## Demostración

https://www.youtube.com/watch?v=mXffI4usjAo&ab_channel=lRaging

## Resultados

### Métricas de Desempeño

A continuación, se presentan los resultados de una ejecución de prueba, demostrando el desempeño del sistema en un escenario específico.

- Tiempo total de ejecución: 87.81 segundos
- Distancia total recorrida: 20.06 metros
- Distancia en línea recta (inicio a fin): 2.55 metros
- Eficiencia de la ruta (Recorrido / Línea Recta): 7.88
- Número de replanificaciones (llamadas a RRT): 7

### Análisis de Algoritmos

* **Factibilidad y Eficiencia:** El algoritmo RRT demuestra ser muy eficiente para encontrar rápidamente una solución en espacios complejos. No garantiza la ruta más corta (como se observa en la métrica de eficiencia de 7.88), pero su velocidad es clave para la replanificación en tiempo real.
* **Robustez:** La capacidad de invocar RRT nuevamente cuando la ruta se bloquea (`Número de replanificaciones: 7`) dota al robot de una gran robustez. Puede adaptarse a obstáculos no mapeados inicialmente y encontrar rutas alternativas para cumplir su objetivo. La combinación del LIDAR para el mapeo y los sensores de distancia para la seguridad inmediata permite al robot reaccionar eficazmente ante el entorno.

### Reflexión sobre Mejoras

1. Control proporcional más sofisticado (P, PI o PID):
El controlador actual es proporcional (P) y solo minimiza el error angular hacia el waypoint. Implementar un control PI o PID permitiría no solo corregir el error instantáneo, sino también considerar el histórico y la tendencia, logrando movimientos más suaves y precisos, sobre todo en curvas o ante pequeñas oscilaciones. Esto es especialmente relevante cuando el entorno es complejo y se requiere mayor precisión para evitar obstáculos.

2. Optimización y suavizado de la ruta (post-procesamiento de RRT):
El algoritmo RRT que utilizas encuentra rutas factibles pero, por su naturaleza, tiende a generar trayectorias con muchos quiebres y no necesariamente cortas. Actualmente, la eficiencia de ruta reportada es baja (relación Recorrido/Línea recta ≈7.88). Al aplicar técnicas de suavizado de caminos (como Bezier, splines, o atajos entre nodos libres) puedes reducir la distancia total y mejorar la eficiencia energética y temporal del robot, logrando trayectorias más naturales y seguras.

3. Uso de SLAM para entornos desconocidos:
El sistema construye un mapa de ocupación local a partir del LIDAR, pero no implementa algoritmos completos de SLAM. Si el robot debe operar en entornos completamente desconocidos o más grandes, un sistema SLAM permitiría construir y actualizar un mapa global mientras se navega, facilitando la exploración y la robustez ante cambios inesperados en el entorno.

4. Exploración de variantes como RRT*:
El RRT básico que usas es rápido para encontrar rutas, pero no garantiza óptimos. RRT* agrega la optimización incremental, convergiendo hacia rutas más cortas a medida que se agregan muestras. Si bien sacrifica algo de velocidad inicial, a mediano plazo logra rutas mucho mejores, lo que es clave si buscas balancear rapidez en la planificación con calidad de trayectorias.



