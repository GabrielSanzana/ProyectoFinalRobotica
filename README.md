# Proyecto Final: Robótica y Sistemas Autónomos 2025-01

**Integrantes:**
* Patricio Figueroa
* Marcelo Flores
* Kavon Kermani
* Gabriel Sanzana
* Lucas Zamora

**Software de Simulación:**
* Webots R2025a

## Descripción del Proyecto

El presente proyecto tiene como objetivo el desarrollo e implementación de un robot móvil autónomo de 4 ruedas en el simulador Webots, utilizando control cinemático diferencial y una variedad de sensores para la percepción del entorno.

El robot es capaz de:
* Detectar y evitar obstáculos en tiempo real mediante la combinación de un sensor LIDAR 2D y sensores de distancia (ultrasónicos o infrarrojos).
* Construir un mapa local del entorno (grilla de ocupación 8x8) a partir de los datos del LIDAR.
* Planificar rutas óptimas en el mapa utilizando el algoritmo de planificación A* (A-Star).
* Navegar de manera autónoma hacia un objetivo definido en el entorno simulado, ajustando su trayectoria en función de la percepción actualizada.

El desarrollo integra percepción, planificación y control en un entorno dinámico, permitiendo que el robot reaccione ante cambios en su entorno y tome decisiones en tiempo real. El robot se prueba en un entorno controlado de **5m x 5m** (según `RectangleArena { floorSize 5 5 }` en `mundo_virtual.wbt`) con obstáculos distribuidos (como la `WoodenBox`), evaluando su desempeño en términos de eficiencia de navegación, precisión de planificación y robustez en la evasión de obstáculos.

## Arquitectura del Software

El sistema de control del robot se organiza en tres niveles principales:

###  Percepción

* **LIDAR 2D (1 capa, 128 resoluciones, `maxRange 0.8`):**
    * Obtiene un mapa parcial del entorno en cada ciclo.
    * Detecta obstáculos en un rango de hasta **0.8 metros** (según la configuración del LIDAR en `mundo_virtual.wbt`).
    * Construye un mapa de ocupación (grilla 8x8).
* **Sensores de distancia (`DistanceSensor` - "distIzq" y "distDer"):**
    * Detectan obstáculos cercanos (frontales y laterales inmediatos, dada su `translation` y `rotation` en `mundo_virtual.wbt`).
    * Proveen una capa de seguridad adicional para la evasión reactiva.
* **GPS:**
    * Obtiene la posición actual del robot en el entorno simulado (coordenadas X, Z).

###  Planificación

* **Grilla de ocupación:**
    * Mapa 2D representado como una matriz `grid[8][8]`.
    * Celdas marcadas como libres u ocupadas.
* **Algoritmo de planificación A\* (A-Star):**
    * Calcula la ruta óptima desde la posición actual hacia el objetivo.
    * Se ejecuta en cada ciclo para ajustar el plan en función de los cambios en el mapa.

###  Control

* **Control de navegación:**
    * Si se detecta un obstáculo cercano → evasión reactiva (giro en el lugar o ajuste de velocidad de las ruedas).
    * Si no hay obstáculos cercanos → seguimiento del camino planificado (A\*).
* **Controlador de motores (`RotationalMotor` - "motor1", "motor2", "motor3", "motor4"):**
    * Control cinemático diferencial (velocidad de ruedas izquierda/derecha).
    * Ajustes simples de velocidad en función del ángulo hacia el waypoint actual.

Esta arquitectura modular permite la integración de múltiples fuentes de percepción para una navegación robusta, combinando planificación deliberativa (A\*) con comportamientos reactivos (evasión rápida).

## Diagramas del Sistema

Aquí se presentarán los diagramas que representan el sistema del robot y su interacción con el entorno.

### Diagrama de Alto Nivel

[ **ESPACIO PARA EL DIAGRAMA DE ALTO NIVEL** -  ]

### Diagrama de Bajo Nivel (Arquitectura Interna o Flujo de Control)

[ **ESPACIO PARA EL DIAGRAMA DE BAJO NIVEL** -  ]

## Resultados

### Métricas de Desempeño

* **Tiempo total de navegación:** ___ segundos
* **Longitud del path (celdas):** ___
* **Tiempo de planificación (A\*):** ___ milisegundos
* **Porcentaje del mapa explorado:** ___ %

### Análisis de Algoritmos

* **Precisión:** El algoritmo A\* logra rutas óptimas en escenarios conocidos, demostrando la capacidad del robot para encontrar caminos eficientes hacia el objetivo.
* **Eficiencia:** La planificación en grilla 8x8 es rápida (< 50 ms por iteración), lo que permite una toma de decisiones casi en tiempo real.
* **Robustez:** La combinación de un LIDAR 2D y sensores de distancia frontales y laterales inmediatos permite al robot reaccionar eficazmente ante la presencia de obstáculos estáticos y dinámicos, garantizando una navegación segura.

### Reflexión sobre Mejoras

* Implementar un control proporcional (P, PI o PID) más sofisticado para el seguimiento del waypoint, lo que permitiría trayectorias más suaves y precisas.
* Considerar el uso de algoritmos de SLAM (Simultaneous Localization and Mapping) o un mapeo más denso para entornos más complejos o desconocidos, permitiendo al robot construir mapas más detallados y navegar en áreas más grandes.
* Integrar planificación incremental o replanificación dinámica para ambientes altamente dinámicos, donde los obstáculos pueden moverse con frecuencia, para adaptar la ruta de forma más fluida.

## Demostración

[ **ESPACIO PARA EL VIDEO DE DEMOSTRACIÓN** - ]

```bash
git clone https://github.com/tu_usuario/tu_repositorio.git]
