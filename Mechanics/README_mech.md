# EVA-505 | Chasis y diseño mecánico

EVA-505 utiliza un **chasis comercial de cuatro ruedas, dirección Ackermann y tracción trasera**. Sobre esta base se incorporan las piezas impresas de la revisión **R08**, destinadas a sostener la electrónica, posicionar la cámara y sujetar la batería.

El trabajo mecánico se centra en adaptar los soportes al espacio disponible, conservar el movimiento del chasis y facilitar el montaje y mantenimiento del robot.

## Chasis y movimiento

El chasis conserva su estructura y transmisión comerciales. La dirección delantera se acciona mediante un **servomotor MG996R**, que mueve el mecanismo de dirección. La geometría Ackermann permite que la rueda interior gire más que la exterior al tomar una curva, aproximando el movimiento de ambas a un mismo centro de giro.

La tracción se transmite a las ruedas traseras mediante el motor y el conjunto de engranajes del chasis. Los soportes añadidos deben dejar libres las ruedas, las articulaciones de dirección y la transmisión durante todo su recorrido.

Las placas frontal y trasera sirven como superficies de fijación. Se aprovechan sus puntos de montaje y las ranuras de las piezas impresas para ajustar la posición de los soportes.

## Distribución de las piezas

La zona frontal y central utiliza **dos plataformas superpuestas**. La inferior organiza los módulos electrónicos; la superior sostiene el case de la Raspberry Pi 5 y permite fijar módulos auxiliares por debajo. Los separadores mantienen la distancia entre niveles y dejan espacio para acceder a los componentes.

El **portal de cámara** se coloca en el frente para mantener la cámara orientada hacia la pista. Su estructura une dos apoyos laterales y una placa superior inclinada, buscando una posición estable durante el movimiento.

En la parte trasera se colocan **dos rieles longitudinales para la batería**. La batería se orienta transversalmente, con su dimensión de **90 mm de lado a lado del vehículo**, y se sujeta con tiras y material antideslizante.

## Piezas impresas R08

| Pieza | Cantidad | Función mecánica |
|---|---:|---|
| Plataforma inferior de electrónica | 1 | Une la estructura de soportes al chasis y proporciona ranuras para fijar los módulos. |
| Plataforma superior para el case de la Pi | 1 | Sostiene el case y permite montar componentes bajo la plataforma. |
| Portal frontal de cámara | 1 | Define la posición y orientación de la cámara respecto al chasis. |
| Riel de sujeción de batería | 2 | Proporciona apoyos y pasos para las tiras de retención. |

El conjunto comprende **cuatro diseños y cinco piezas impresas**.

### Archivos de las piezas

- `EVA505_ELECTRONICS_DECK_UNIVERSAL_R08.stl`
- `EVA505_PI_CASE_DECK_UNIVERSAL_R08.stl`
- `EVA505_FRONT_CAMERA_PORTAL_R08_PROTOTYPE.stl`
- `EVA505_BATTERY_STRAP_RAIL_UNIVERSAL_R08_PRINT_2.stl` — imprimir dos unidades.

Los archivos **STL** contienen las geometrías para impresión 3D. Las versiones **STEP**, cuando se incorporan al repositorio, permiten revisar las piezas como sólidos en programas CAD como Onshape.

## Criterios de montaje

El montaje propuesto une las plataformas con **cuatro separadores M3 de 35 mm**, distribuidos en un patrón de **54 × 44 mm**. La plataforma superior tiene el contorno desplazado **8 mm hacia atrás**, dejando espacio para los montantes del portal frontal.

La torre utiliza dos fijaciones independientes. La revisión de montaje propone **separadores de 3 mm bajo sus anclajes** para evitar interferencias con las arandelas anchas del chasis. Esta separación debe comprobarse con la tornillería utilizada.

El case debe apoyarse sin descansar sobre las cabezas de los tornillos y conservar sus entradas de ventilación. Bajo los rieles se deja espacio para pasar las tiras de la batería; sobre ellos se coloca material antideslizante. Las fijaciones deben impedir desplazamientos sin deformar las piezas ni la batería.

La comprobación final se realiza sobre el robot físico: dirección y transmisión libres, batería firme, cámara estable y soportes accesibles. Las cotas del CAD sirven como referencia y se contrastan con las piezas impresas y el chasis real.

## Galería del proyecto

<table>
  <tr>
    <td width="50%" align="center" valign="middle">
      <img src="https://github.com/user-attachments/assets/462f1b39-af06-4e20-bd6b-8387dbaa0a9f" width="100%" alt="Fotografía 1 del proyecto EVA-505" />
      <br />
      <sub>Registro del proyecto · 01</sub>
    </td>
    <td width="50%" align="center" valign="middle">
      <img src="https://github.com/user-attachments/assets/43af9fb4-91f4-4026-a1f7-1ac6fcf93177" width="100%" alt="Fotografía 2 del proyecto EVA-505" />
      <br />
      <sub>Registro del proyecto · 02</sub>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <br /><br />
      <strong>Vista superior del chasis</strong>
      <br />
      <sub>Espacio para añadir una fotografía</sub>
      <br /><br />
    </td>
    <td width="50%" align="center">
      <br /><br />
      <strong>Plataformas y separadores</strong>
      <br />
      <sub>Espacio para añadir una fotografía</sub>
      <br /><br />
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <br /><br />
      <strong>Portal frontal de cámara</strong>
      <br />
      <sub>Espacio para añadir una fotografía</sub>
      <br /><br />
    </td>
    <td width="50%" align="center">
      <br /><br />
      <strong>Rieles y sujeción de batería</strong>
      <br />
      <sub>Espacio para añadir una fotografía</sub>
      <br /><br />
    </td>
  </tr>
</table>

<!-- Para añadir fotografías, sustituye el contenido del espacio correspondiente por:
<img src="URL_DE_LA_FOTO" width="100%" alt="Descripción de la fotografía" />
<br />
<sub>Pie de foto</sub>
Para ampliar la galería, duplica una fila <tr> completa con sus dos celdas <td>.
-->
