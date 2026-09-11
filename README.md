<!--
EVA505 · README principal.
Los cuadros de fotografía ya se renderizan: cada uno utiliza un SVG local.
Para incorporar una foto, cambia únicamente el atributo src del cuadro.
Las rutas propuestas y la lista de imágenes están en docs/IMAGENES_README.md.
-->

<a id="inicio"></a>

<p align="center">
  <img src="docs/media/eva505-portada.svg" width="100%" alt="EVA505 · WRO Future Engineers 2026 · Mecánica R08, visión y control autónomo">
</p>

<h1 align="center">EVA505 · Future Engineers 2026</h1>

<p align="center">
  <strong>Diseño mecánico, visión artificial y control embebido en un mismo vehículo.</strong><br>
  Equipo EVA505 · UTEC / IEEE RAS UTEC
</p>

<p align="center">
  <a href="#proyecto">Proyecto</a> ·
  <a href="#arquitectura">Arquitectura</a> ·
  <a href="#mecanica">Mecánica</a> ·
  <a href="#electronica">Electrónica</a> ·
  <a href="#software">Software</a> ·
  <a href="#ejecucion">Ejecución</a> ·
  <a href="#validacion">Validación</a> ·
  <a href="#repositorio">Archivos</a>
</p>

<table>
  <tr>
    <td width="33%" align="center"><strong>R08</strong><br><sub>Revisión mecánica · 4 diseños, 5 piezas</sub></td>
    <td width="34%" align="center"><strong>Raspberry Pi 5 + ESP32</strong><br><sub>Visión, navegación y control de actuadores</sub></td>
    <td width="33%" align="center"><strong>Sin encoder</strong><br><sub>Tracción por PWM · dirección Ackermann</sub></td>
  </tr>
</table>

---

<a id="proyecto"></a>

## 01 · El proyecto

**EVA505 es un vehículo autónomo desarrollado para WRO Future Engineers 2026.** Integra un chasis comercial de cuatro ruedas, dirección delantera Ackermann, tracción trasera y soportes diseñados para impresión 3D. La cámara aporta la información visual de la pista; la Raspberry Pi interpreta esa información y el ESP32 ejecuta las órdenes de dirección y tracción.

El repositorio reúne el diseño mecánico, el esquema eléctrico, el firmware, los algoritmos de percepción y navegación, las herramientas de diagnóstico y la evidencia de validación. Su organización permite seguir el recorrido completo desde una pieza CAD hasta una orden enviada al motor.

| Área | Solución de EVA505 |
| :--- | :--- |
| Estructura | Chasis comercial con dos plataformas, portal frontal y rieles de batería R08. |
| Movimiento | Motor DC GA25 para tracción trasera y servo MG996R para dirección. |
| Percepción | Cámara, Picamera2 y OpenCV; análisis de paredes, pilares y marcadores de suelo. |
| Decisión | Control de dirección PD y máquina de estados en Python. |
| Ejecución | ESP32 con PWM, rampas, pulsador START y vigilancia de comandos. |
| Reproducibilidad | Archivos STEP/STL, configuración YAML, pruebas y registros de resultados. |

> **Estado de la revisión:** software validado en PC y firmware compilado. La calibración del vehículo, la validación eléctrica bajo carga y las tandas completas en pista siguen pendientes de registro. El estacionamiento permanece experimental y desactivado por defecto.

### El vehículo en cuatro vistas

<table>
  <tr>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 01 · src sugerido: v-photos/robot-frontal.jpg -->
      <img src="docs/media/foto-pendiente.svg" width="100%" alt="Espacio para la vista frontal de EVA505">
      <br><strong>01 · Vista frontal</strong><br>
      <sub>Cámara, dirección y despeje de las ruedas delanteras.</sub>
    </td>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 02 · src sugerido: v-photos/robot-superior.jpg -->
      <img src="docs/media/foto-pendiente.svg" width="100%" alt="Espacio para la vista superior de EVA505">
      <br><strong>02 · Vista superior</strong><br>
      <sub>Distribución de plataformas, electrónica y batería.</sub>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 03 · src sugerido: v-photos/robot-lateral.jpg -->
      <img src="docs/media/foto-pendiente.svg" width="100%" alt="Espacio para la vista lateral de EVA505">
      <br><strong>03 · Vista lateral</strong><br>
      <sub>Separación entre niveles y posición del portal.</sub>
    </td>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 04 · src sugerido: v-photos/robot-trasera.jpg -->
      <img src="docs/media/foto-pendiente.svg" width="100%" alt="Espacio para la vista trasera de EVA505">
      <br><strong>04 · Vista trasera</strong><br>
      <sub>Tracción, retención de batería y acceso al montaje.</sub>
    </td>
  </tr>
</table>

<a id="arquitectura"></a>

## 02 · Arquitectura del sistema

El procesamiento se divide entre dos placas. La **Raspberry Pi 5** trabaja con imágenes y toma decisiones; el **ESP32** mantiene el control de los actuadores y supervisa la vigencia de las órdenes recibidas.

<p align="center">
  <img src="docs/media/arquitectura.svg" width="100%" alt="Cámara a Raspberry Pi 5; comunicación UART bidireccional con ESP32; ESP32 conectado a START, DRV8871 y servo MG996R">
</p>

| Capa | Responsabilidad | Resultado |
| :--- | :--- | :--- |
| Captura | Obtener un fotograma reciente y controlar exposición y balance de blancos. | Imagen BGR y referencia temporal. |
| Percepción | Localizar límites de pista, pilares y marcadores mediante HSV y geometría de imagen. | Error lateral, candidatos y confianza. |
| Navegación | Seleccionar el comportamiento y calcular dirección y potencia. | Una orden conjunta de movimiento. |
| Comunicación | Enviar comandos y comprobar telemetría, secuencias y CRC. | Intercambio UART supervisado. |
| Actuación | Aplicar PWM, límites, rampas y condiciones de arranque. | Tracción y dirección dentro de los límites configurados. |

La comunicación funciona a **115 200 baudios**. La telemetría informa del PWM aplicado, el ángulo ordenado y el estado del controlador; no representa mediciones de velocidad ni de posición del servo.

<a id="mecanica"></a>

## 03 · Diseño mecánico R08

### Chasis, dirección y tracción

La base comercial conserva su dirección y transmisión. El **GA25** transmite movimiento al tren trasero, mientras el **MG996R** acciona el mecanismo de dirección delantero.

La geometría **Ackermann** hace que la rueda interior gire más que la exterior durante una curva, aproximando sus trayectorias a un centro de giro común. El radio de giro real depende de la distancia entre ejes, la separación de las ruedas y los ángulos alcanzados por el mecanismo; todavía no se publica un valor medido.

Sobre esta base se distribuyen dos plataformas, un portal de cámara y dos rieles para la batería. La división en subconjuntos facilita desmontar componentes y modificar sus soportes conservando las interfaces de fijación.

### Piezas y archivos de fabricación

Las dimensiones exteriores de esta tabla se comprobaron sobre los **STL presentes en el repositorio**. Son cotas de las piezas, no las dimensiones totales del vehículo.

| Pieza | Cantidad | Envolvente nominal, mm | Función | Descargas |
| :--- | :---: | :--- | :--- | :--- |
| Plataforma inferior | 1 | 104 × 84 × 3,2 | Fijación al chasis, electrónica y soporte del nivel superior. | [STL](Mechanics/EVA505_ELECTRONICS_DECK_UNIVERSAL_R08.stl) · [STEP](Mechanics/EVA505_ELECTRONICS_DECK_UNIVERSAL_R08.step) |
| Plataforma superior | 1 | 104 × 84 × 3,2 | Apoyo del case de la Pi y fijación de módulos inferiores. | [STL](Mechanics/EVA505_PI_CASE_DECK_UNIVERSAL_R08.stl) · [STEP](Mechanics/EVA505_PI_CASE_DECK_UNIVERSAL_R08.step) |
| Portal frontal | 1 | ≈ 40,731 × 79 × 94,643 | Soporte elevado e inclinado de la cámara. | [STL](Mechanics/EVA505_FRONT_CAMERA_PORTAL_R08_PROTOTYPE.stl) · [STEP](Mechanics/EVA505_FRONT_CAMERA_PORTAL_R08_PROTOTYPE.step) |
| Riel de batería | 2 | 72 × 14 × 3,2 por unidad | Apoyo y paso de las correas de retención. | [STL](Mechanics/EVA505_BATTERY_STRAP_RAIL_UNIVERSAL_R08_PRINT_2.stl) · [STEP](Mechanics/EVA505_BATTERY_STRAP_RAIL_UNIVERSAL_R08_PRINT_2.step) |

**Cuatro diseños y cinco piezas impresas.** El sufijo `PRINT_2` indica dos unidades del riel. Los STL se interpretan en milímetros y a escala del 100 %.

<table>
  <tr>
    <td width="50%" align="center" valign="top">
      <a href="Mechanics/EVA505_ELECTRONICS_DECK_UNIVERSAL_R08.png">
        <img src="Mechanics/EVA505_ELECTRONICS_DECK_UNIVERSAL_R08.png" width="100%" alt="Plano de la plataforma inferior de electrónica R08">
      </a>
      <br><strong>Plataforma inferior</strong><br><sub>Interfaz con el chasis y distribución de cargas.</sub>
    </td>
    <td width="50%" align="center" valign="top">
      <a href="Mechanics/EVA505_PI_CASE_DECK_UNIVERSAL_R08.png">
        <img src="Mechanics/EVA505_PI_CASE_DECK_UNIVERSAL_R08.png" width="100%" alt="Plano de la plataforma superior para Raspberry Pi R08">
      </a>
      <br><strong>Plataforma superior</strong><br><sub>Case de la Raspberry Pi y segundo nivel.</sub>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center" valign="top">
      <a href="Mechanics/EVA505_FRONT_CAMERA_PORTAL_R08_PROTOTYPE.png">
        <img src="Mechanics/EVA505_FRONT_CAMERA_PORTAL_R08_PROTOTYPE.png" width="100%" alt="Plano del portal frontal de cámara R08">
      </a>
      <br><strong>Portal frontal</strong><br><sub>Dos apoyos y placa de cámara inclinada.</sub>
    </td>
    <td width="50%" align="center" valign="top">
      <a href="Mechanics/EVA505_BATTERY_STRAP_RAIL_UNIVERSAL_R08_PRINT_2.png">
        <img src="Mechanics/EVA505_BATTERY_STRAP_RAIL_UNIVERSAL_R08_PRINT_2.png" width="100%" alt="Plano del riel de batería R08">
      </a>
      <br><strong>Riel de batería</strong><br><sub>Dos copias con retención mediante correas.</sub>
    </td>
  </tr>
</table>

### Interfaces y decisiones de diseño

Las cotas de interfaces que siguen proceden del aporte mecánico R08 del equipo. Los separadores y suplementos indicados como propuesta corresponden al montaje previsto; su ajuste físico todavía debe registrarse.

| Elemento | Cota o solución nominal | Motivo de diseño |
| :--- | :--- | :--- |
| Unión inferior al chasis | Cuatro ranuras de 16 × 10 mm; centros en patrón de 82 × 64 mm. | Admitir pequeñas diferencias de posición de la base comercial. |
| Reparto del apriete | Arandelas con diámetro exterior de al menos 16 mm. | Distribuir la carga sobre las ventanas de fijación. |
| Apilamiento | Patrón de 54 × 44 mm; agujeros de Ø 3,8 mm. | Mantener una interfaz independiente del ajuste al chasis. |
| Separación vertical | Propuesta: cuatro separadores M3 de 35 mm. | Alojar módulos y conectores entre plataformas. |
| Plataforma superior | Contorno desplazado 8 mm hacia atrás; agujeros conservados. | Liberar espacio para el portal sin mover el patrón estructural. |
| Base del portal | Dos fijaciones separadas 65 mm, de Ø 3,8 mm. | Sujetar la cámara con dos apoyos independientes. |
| Interfaz de cámara | Patrón de 21 × 12,5 mm; agujeros de Ø 2,4 mm; inclinación de 15°. | Definir la posición del módulo y orientar la vista hacia la pista. |
| Retención de batería | Dos rieles y una correa por riel. | Permitir su retirada sin desmontar las plataformas. |

El desplazamiento de la plataforma superior ya forma parte de su geometría: el ensamblaje conserva la coincidencia de los agujeros. Los **94,643 mm del portal** describen su altura exterior, no la altura de la lente respecto al suelo.

<details>
<summary><strong>Detalles del montaje y tolerancias</strong></summary>

- **Referencia de ejes:** X longitudinal, frente hacia X negativo; Y transversal y Z vertical.
- **Plataformas:** con separadores de 35 mm y placas de 3,2 mm, la distancia nominal entre sus caras inferiores es 38,2 mm. Los módulos reducen el espacio libre local.
- **Portal:** el aporte R08 describe 3,5 mm nominales de separación con el borde delantero de la plataforma superior. Propone suplementos de 3 mm bajo ambos apoyos cuando la tornillería cercana lo requiera.
- **Batería de referencia:** envolvente objetivo de 90 × 35 × 20 mm, con el lado de 90 mm transversal al vehículo. Esta envolvente no especifica su tensión, capacidad ni química.
- **Rieles:** ranuras de fijación de 10,4 × 4,8 mm, con centros separados 56 mm. La propuesta contempla aproximadamente 1,5 mm de elevación para las correas y 1 mm de material antideslizante.
- **Fabricación:** material, orientación, paredes, relleno y tolerancias de impresión influyen en la rigidez y el ajuste. El repositorio no atribuye una resistencia o masa final a partir del STL.

Las referencias de montaje no sustituyen las medidas del conjunto construido. El despeje de las ruedas, las bieletas, la transmisión y el cableado se evalúa durante todo su recorrido.

</details>

### Del CAD al vehículo

El formato **STL** describe una superficie mediante triángulos y se utiliza en el laminador. **STEP** permite intercambiar geometría para inspección y ensamblaje en CAD. Una conversión desde una malla no recupera automáticamente los croquis ni el historial paramétrico original.

El paquete publicado en `Mechanics/` contiene los cuatro pares STEP/STL y sus planos PNG. Las fuentes generadoras y los informes adicionales citados en el aporte mecánico no están incluidos en esta carpeta.

<table>
  <tr>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 05 · src sugerido: v-photos/direccion-ackermann.jpg -->
      <img src="docs/media/foto-pendiente.svg" width="100%" alt="Espacio para el detalle del mecanismo Ackermann">
      <br><strong>Dirección y recorrido</strong><br><sub>Servo, brazos y bieletas del chasis.</sub>
    </td>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 06 · src sugerido: v-photos/montaje-r08.jpg -->
      <img src="docs/media/foto-pendiente.svg" width="100%" alt="Espacio para el montaje de plataformas, portal y batería">
      <br><strong>Integración R08</strong><br><sub>Separadores, fijación del portal y correas.</sub>
    </td>
  </tr>
</table>

<a id="electronica"></a>

## 04 · Electrónica y alimentación

### Componentes principales

| Componente | Cantidad | Función |
| :--- | :---: | :--- |
| Raspberry Pi 5 | 1 | Captura, visión artificial y navegación. |
| ESP32 DevKit V1 | 1 | Control de actuadores, START y supervisión de órdenes. |
| Cámara | 1 | Observación de la pista; captura mediante Picamera2. |
| DRV8871 | 1 | Etapa de potencia para el motor DC mediante IN1/IN2. |
| Motor GA25 | 1 | Tracción del vehículo. |
| Servo MG996R | 1 | Accionamiento de la dirección. |
| Convertidor XL4005 | 2 | Ramas de alimentación para la Pi y el servo. |
| Pulsador START | 1 | Inicio físico y parada local. |
| Batería | 1 | Fuente de energía del sistema. |
| Fusible, interruptor y distribución | 1 conjunto | Corte general, protección y reparto de alimentación. |

### Distribución de energía

El esquema distribuye la batería desde el conector **XT60**, el fusible **F1 de 10 A** y el interruptor general. Desde la línea conmutada se alimentan el DRV8871 y los dos convertidores XL4005.

| Rama del esquema | Destino | Elementos asociados |
| :--- | :--- | :--- |
| Batería conmutada | Alimentación VM del DRV8871 y entradas de los reguladores. | Distribución WAGO y TVS P6KE12A. |
| `+5V1_PI` | Raspberry Pi y VIN del ESP32. | XL4005 n.º 1 y condensador de 470 µF. |
| `+5V_SERVO` | Alimentación del MG996R. | XL4005 n.º 2, 470 µF y 1000 µF. |
| Motor | Salidas del DRV8871 hacia el GA25. | Condensador de 100 nF entre terminales del motor. |
| GND común | Referencia de potencia y señales. | Conexión compartida entre ambas placas y actuadores. |

Las ramas separan la alimentación del servo y de la computadora, manteniendo una referencia de tierra común. Las tensiones, las caídas bajo carga y la capacidad efectiva del montaje todavía no tienen mediciones publicadas.

### Mapa de conexiones

| Señal | Raspberry Pi / periférico | ESP32 | Configuración |
| :--- | :--- | :--- | :--- |
| UART Pi → ESP32 | Pin físico 8, GPIO14 / TX | GPIO16 / RX2 | 115 200 baudios, señales de 3,3 V. |
| UART ESP32 → Pi | Pin físico 10, GPIO15 / RX | GPIO17 / TX2 | Comunicación cruzada TX → RX. |
| Tierra | Pi pin físico 6 y GND de módulos | GND | Referencia común. |
| Motor IN1 | DRV8871 IN1 | GPIO25 | PWM de 20 kHz, canal 0. |
| Motor IN2 | DRV8871 IN2 | GPIO26 | PWM de 20 kHz, canal 1. |
| Dirección | MG996R SIGNAL | GPIO18 | PWM de 50 Hz, canal 2. |
| START | Pulsador hacia GND | GPIO4 | Entrada con pull-up; activo en LOW. |
| Encoder | No instalado | No utilizado | Sin AS5600, ticks ni RPM. |

**La numeración física de la Raspberry Pi es distinta de su numeración GPIO.** El servo utiliza un temporizador separado de los canales del motor.

<details>
<summary><strong>Esquema eléctrico de referencia</strong></summary>

<p align="center">
  <a href="hardware/schematic-reference.png">
    <img src="hardware/schematic-reference.png" width="100%" alt="Esquema de alimentación, ESP32, Raspberry Pi, DRV8871, servo y START">
  </a>
</p>

**Este esquema conserva el AS5600 en J10 como referencia histórica. El equipo retiró el encoder y esa conexión no forma parte de la versión actual.** El [documento de cableado](hardware/readme.md) recoge la correspondencia operativa con el firmware.

</details>

<table>
  <tr>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 07 · src sugerido: v-photos/electronica-general.jpg -->
      <img src="docs/media/foto-pendiente.svg" width="100%" alt="Espacio para la distribución real de la electrónica">
      <br><strong>Distribución electrónica</strong><br><sub>Placas, reguladores y organización del cableado.</sub>
    </td>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 08 · src sugerido: v-photos/potencia-y-conexiones.jpg -->
      <img src="docs/media/foto-pendiente.svg" width="100%" alt="Espacio para el detalle de las conexiones eléctricas">
      <br><strong>Potencia y conexiones</strong><br><sub>Protecciones, UART y conexiones de actuadores.</sub>
    </td>
  </tr>
</table>

<a id="software"></a>

## 05 · Percepción, navegación y control

### Raspberry Pi: de la imagen a la trayectoria

El programa obtiene fotogramas recientes y comparte una conversión HSV entre los detectores. La pista se estima a partir de límites y zonas transitables de la imagen. Cuando la perspectiva lo requiere, se utiliza también el perfil de contacto entre pared y suelo.

Los pilares se filtran por color y posición, dando prioridad a los candidatos con mayor proximidad aparente. La dirección se calcula mediante un **control PD con derivada filtrada**, compensación de curvatura y límite de cambio angular.

| Función | Implementación actual |
| :--- | :--- |
| Seguimiento de pista | Error lateral normalizado, confianza y evidencia de paredes. |
| Pilares | Detección de rojo y verde; separación respecto a sus bordes en la imagen. |
| Paso de obstáculos | Estrategia configurada: rojo por la derecha y verde por la izquierda del vehículo. |
| Marcadores de sección | Detección de líneas azules o naranjas, confirmación y antirrebote temporal. |
| Vueltas | Estimación a partir de secciones visuales; configuración inicial de 3 vueltas y 4 secciones por vuelta. |
| Final de recorrido | Aproximación temporal tras el último marcador, pendiente de calibración física. |
| Estacionamiento | Estados experimentales, dos marcadores y maniobra temporal explícita; desactivado. |
| Diagnóstico | Registros JSONL de estados, órdenes, tiempos de procesamiento y motivos de parada. |

La estrategia es reactiva sobre la imagen. No incorpora odometría ni un mapa global; el tiempo de aplicación de una orden no se interpreta como una distancia recorrida.

### Máquina de estados

| Estado | Comportamiento |
| :--- | :--- |
| `INITIALIZATION` | Prepara la ejecución y comprueba las condiciones iniciales. |
| `READY` | Espera una nueva pulsación física de START después de ARM. |
| `FOLLOW_TRACK` | Calcula y aplica seguimiento de pista. |
| `OBSTACLE_AVOID` | Ajusta la trayectoria para rodear el pilar seleccionado. |
| `FINISH_APPROACH` | Ejecuta la aproximación final configurada. |
| `PARKING_*` | Busca, alinea y maniobra cuando el estacionamiento está habilitado. |
| `FINISHED` | Finaliza la ejecución y corta la tracción. |
| `SAFE_STOP` | Mantiene la parada tras un fallo detectado. |

### ESP32: actuación y supervisión

El firmware recibe una orden conjunta de potencia y dirección, aplica límites y genera las señales PWM. La rampa del motor suaviza los cambios de potencia y una pausa de inversión evita pasar directamente de un sentido al contrario.

| Mecanismo | Comportamiento implementado |
| :--- | :--- |
| Arranque físico | HELLO y ARM preparan el controlador; una nueva pulsación de START habilita el movimiento. |
| Parada local | Una segunda pulsación corta la tracción. |
| Vigilancia de movimiento | Sin un DRIVE válido durante 300 ms, el firmware corta y registra un fallo. |
| Integridad de mensajes | CRC-16/CCITT-FALSE, sesión y secuencias para detectar corrupción y rechazar mensajes repetidos. |
| Frescura en la Pi | La telemetría debe avanzar y llegar dentro del plazo configurado de 250 ms. |
| Fallos de percepción | Se rechazan fotogramas repetidos o tardíos; la pérdida de pista reduce la potencia a cero. |
| Recuperación | Una conexión recuperada no reinicia automáticamente el vehículo. |

**STOP coloca IN1 e IN2 a cero:** elimina el accionamiento y deja el motor en rueda libre. No equivale a un frenado mecánico instantáneo.

El protocolo y sus campos se documentan en [PROTOCOLO.md](docs/PROTOCOLO.md). Un mensaje PING no prolonga la vigencia de una orden de movimiento.

### Parámetros de partida

| Parámetro | Valor actual | Interpretación |
| :--- | :--- | :--- |
| Imagen | 640 × 480 px | Resolución configurada. |
| Frecuencia de captura | Objetivo de 25 FPS | No es una medición de rendimiento en la Pi. |
| Potencia de crucero | 0,22 | Fracción PWM; no representa m/s. |
| Límite de potencia | ±0,35 | Límite inicial de puesta en marcha. |
| Servo | Centro 90°; límites 60–120° | Valores de mando provisionales, no ángulos medidos de las ruedas. |
| Tiempo máximo de ejecución | 180 s | Límite local del programa. |
| Estacionamiento | Desactivado | `parking.enabled: false`. |

La configuración principal está en [config.yaml](models/raspberry%20pi%205/config/config.yaml); los límites de actuadores están en [config.h](models/esp32/config.h). Ambos forman parte de una misma configuración del vehículo.

<a id="ejecucion"></a>

## 06 · Instalación y ejecución

### Desarrollo en PC

El proyecto requiere **Python 3.10 o posterior**. Los comandos siguientes utilizan Python 3.13 en Windows, desde la raíz del repositorio.

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"

# Comprobar configuración y ejecutar sin abrir hardware.
.\.venv\Scripts\python.exe -m raspberry_pi --check-config
.\.venv\Scripts\python.exe -m raspberry_pi --simulate --frames 150

# Pruebas automatizadas y compilación del firmware.
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\pio.exe run
```

La simulación utiliza imágenes sintéticas y un enlace simulado. Permite comprobar la integración del programa sin accionar el robot.

<details>
<summary><strong>Comprobaciones adicionales de desarrollo</strong></summary>

```powershell
.\.venv\Scripts\ruff.exe check models tests scripts
.\.venv\Scripts\ruff.exe format --check models tests scripts

# Compilar el firmware real contra periféricos simulados en Windows.
.\.venv\Scripts\python.exe -m pip install ziglang==0.16.0
.\.venv\Scripts\python.exe scripts/test_firmware.py
```

En Linux, el último script utiliza `g++` cuando está disponible. La compilación para ESP32 se define en [platformio.ini](platformio.ini), con Espressif32 6.12.0 y Arduino ESP32 2.0.17.

</details>

### Raspberry Pi 5

La instalación utiliza **Raspberry Pi OS de 64 bits** y las bibliotecas de cámara de APT:

```bash
sudo apt update
sudo apt install python3-picamera2 python3-opencv python3-numpy \
  python3-yaml python3-serial python3-venv

cd WRO2026-EVA505
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --no-deps -e .
python -m raspberry_pi --check-config
```

**UART en Pi 5:** el puerto correspondiente a GPIO14/15 debe estar habilitado y verificado. `/dev/serial0` puede corresponder al conector de depuración. Por eso `serial.port` queda sin un valor supuesto.

Con el firmware compatible cargado y el UART del cabezal confirmado:

```bash
# /dev/ttyAMA0 es un ejemplo: usar el dispositivo verificado.
python -m raspberry_pi --port /dev/ttyAMA0 \
  --challenge open --log-jsonl reports/pista-open.jsonl
```

El modo con obstáculos se selecciona con `--challenge obstacle`. El programa prepara cámara y comunicación, arma el controlador y espera **START**.

La [guía de puesta en marcha](docs/PUESTA_EN_MARCHA.md) desarrolla la configuración del UART, la carga de firmware, las pruebas con ruedas suspendidas y la calibración de cámara y actuadores.

### Herramientas incluidas

| Herramienta | Uso |
| :--- | :--- |
| [bench.py](scripts/bench.py) | Pruebas breves de motor y servo, con límites y pulsación START. |
| [vision_probe.py](scripts/vision_probe.py) | Inspección de una imagen y salida anotada. |
| [validate_dataset.py](scripts/validate_dataset.py) | Procesamiento del conjunto de imágenes y generación de resultados. |
| [test_firmware.py](scripts/test_firmware.py) | Ejecución del firmware real con periféricos simulados en el PC. |

<a id="validacion"></a>

## 07 · Validación y resultados

Los siguientes resultados corresponden a la revisión de software del **11 de septiembre de 2026**, documentada en los archivos de `reports/`.

| Comprobación | Resultado registrado | Evidencia |
| :--- | :--- | :--- |
| Python · OpenCV 5.0.0 / NumPy 2.5.3 | **90 pruebas aprobadas** | [Reporte XML](reports/pytest-opencv5.xml) |
| Python · OpenCV 4.12.0 / NumPy 2.2.6 | **90 pruebas aprobadas** | [Reporte XML](reports/pytest-opencv4.xml) |
| Firmware ESP32 · PlatformIO | **Compilación correcta** | [Registro](reports/firmware-build.txt) |
| Firmware con periféricos simulados | **Prueba correcta** | [Registro](reports/firmware-native.txt) |
| Análisis estático y formato | **Comprobaciones correctas** | [Registro](reports/static-check.txt) |
| Simulación del programa | **Inicio, ejecución y cierre comprobados** | [Registro](reports/simulation-check.txt) |
| Conjunto de imágenes | **184/184 procesadas sin excepciones** | [Resultados JSON](reports/vision_validation.json) |

La compilación registrada utiliza **21 864 bytes de RAM estática** y **284 549 bytes de flash**. Estas cifras describen el binario compilado, no el consumo máximo de memoria durante una carrera.

### Evidencia visual

<p align="center">
  <a href="reports/vision_validation.jpg">
    <img src="reports/vision_validation.jpg" width="100%" alt="Muestra de imágenes del dataset con detecciones y anotaciones del sistema de visión">
  </a>
  <br><sub>Muestra real generada por las herramientas de validación del repositorio.</sub>
</p>

| Observación del conjunto de imágenes | Resultado |
| :--- | ---: |
| Imágenes sobre el umbral de confianza de pista | 115 |
| Imágenes por debajo del umbral | 69 |
| Imágenes con al menos un candidato a pilar | 123 |
| Tiempo mediano de procesamiento en el PC de desarrollo | ≈ 3,36 ms |
| Percentil 95 de procesamiento en ese PC | ≈ 6,73 ms |

**La confianza del detector no es una precisión medida.** Estos resultados describen fotos estáticas y tiempos del PC de desarrollo; no demuestran vueltas completadas ni rendimiento en Raspberry Pi 5. Las etiquetas de los dos grupos contienen clases no equivalentes, cajas y polígonos, por lo que no se publica una métrica global de precisión o recall.

<details>
<summary><strong>Alcance de las pruebas y validación física pendiente</strong></summary>

Las pruebas automatizadas cubren corrupción de mensajes, secuencias repetidas, desbordamiento de recepción, expiración de comandos, arranque físico, límites, rampas, inversión, pérdida de cámara, fotogramas tardíos y casos de percepción y navegación.

La evidencia física pendiente comprende alimentación bajo carga, funcionamiento del UART en la Pi, centro y límites del servo, respuesta del motor con distintas cargas de batería, vista de cámara definitiva, recorridos completos y estacionamiento.

La receta de [integración continua](.github/workflows/ci.yml) está incluida. Los resultados de esta tabla proceden de ejecuciones locales; no se presenta una ejecución en GitHub como realizada.

</details>

### Registro audiovisual de pista

<table>
  <tr>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 09 · src sugerido: v-photos/pista-open.jpg · Puede envolverse en un enlace al vídeo. -->
      <img src="docs/media/video-pendiente.svg" width="100%" alt="Espacio para la portada del vídeo del recorrido abierto">
      <br><strong>Recorrido abierto</strong><br><sub>Registro audiovisual pendiente de incorporar.</sub>
    </td>
    <td width="50%" align="center" valign="top">
      <!-- FOTO 10 · src sugerido: v-photos/pista-obstacle.jpg · Puede envolverse en un enlace al vídeo. -->
      <img src="docs/media/video-pendiente.svg" width="100%" alt="Espacio para la portada del vídeo del recorrido con obstáculos">
      <br><strong>Recorrido con obstáculos</strong><br><sub>Registro audiovisual pendiente de incorporar.</sub>
    </td>
  </tr>
</table>

<a id="repositorio"></a>

## 08 · Organización del repositorio

```text
WRO2026-EVA505/
├── README.md                         # Presentación y documentación principal
├── Mechanics/                        # Cuatro diseños R08: STEP, STL y planos PNG
├── hardware/                         # Cableado y esquema de referencia
├── models/
│   ├── esp32/                        # Firmware, motor, servo y protocolo
│   └── raspberry pi 5/               # Aplicación Python y configuración
│       ├── camera/                   # Captura y calibración
│       ├── communication/            # Enlace UART y protocolo
│       ├── config/                   # YAML y validación de parámetros
│       ├── control/                  # Órdenes y controlador de dirección
│       ├── navigation/               # Movimiento y control
│       ├── state machine/            # Estados y transiciones
│       └── vision/                   # Pista, pilares y marcadores
├── scripts/                          # Banco de pruebas y herramientas de visión
├── tests/                            # Pruebas Python y firmware para host
│   └── datastets/obstacles/           # Imágenes y etiquetas; nombre original
├── reports/                          # Evidencia de validación
├── docs/                             # Guías técnicas y recursos del README
│   └── media/                        # Portada, diagrama y cuadros de imagen
├── t-photos/                         # Ubicación prevista para fotografías del equipo
├── v-photos/                         # Ubicación prevista para fotografías del vehículo
├── videos/                           # Ubicación prevista para registros de pista
├── platformio.ini                    # Compilación ESP32
└── pyproject.toml                    # Instalación y herramientas de Python
```

Las carpetas originales con espacios se conservan. La instalación expone el paquete `raspberry_pi` y su máquina de estados, sin modificaciones manuales de `sys.path`.

### Documentación complementaria

| Documento | Contenido |
| :--- | :--- |
| [Puesta en marcha](docs/PUESTA_EN_MARCHA.md) | Instalación, UART, firmware, calibración y pruebas de pista. |
| [Protocolo C1 / T1](docs/PROTOCOLO.md) | Formato de mensajes, CRC, sesiones, comandos y telemetría. |
| [Cableado](hardware/readme.md) | Pines, funcionamiento del DRV8871 y versión sin encoder. |
| [Revisión y mejoras](docs/REVISION_Y_MEJORAS.md) | Cambios de software, resultados y alcance de la validación. |
| [Inventario de fuentes](reports/source_inventory.json) | Trazabilidad de los 35 archivos originales revisados. |
| [Imágenes del README](docs/IMAGENES_README.md) | Ubicación de los cuadros y sustitución de fotografías. |

<a id="equipo"></a>

## 09 · Equipo EVA505

**EVA505 · UTEC / IEEE RAS UTEC**

El proyecto reúne diseño mecánico, integración electrónica y programación de sistemas autónomos. Este repositorio documenta las decisiones de construcción y la evidencia disponible de cada revisión.

<table>
  <tr>
    <td align="center">
      <!-- FOTO 11 · src sugerido: t-photos/equipo-eva505.jpg -->
      <img src="docs/media/equipo-pendiente.svg" width="100%" alt="Espacio para la fotografía del equipo EVA505">
      <br><strong>El equipo detrás de EVA505</strong><br><sub>Diseño · Integración · Programación · Pruebas</sub>
    </td>
  </tr>
</table>

---

<p align="center">
  <strong>EVA505 · Mecánica R08 · Software 0.2.0</strong><br>
  <sub>Documentación técnica del vehículo y su proceso de desarrollo.</sub><br><br>
  <a href="#inicio">Volver al inicio ↑</a>
</p>
