# SmartTraffic IoT: Sistema de Control Semafórico con Prioridad Peatonal

**Documentación técnica de la maqueta IoT — ESP8266**  
Máquina de estados finitos, solicitudes independientes, prioridad peatonal, telemetría e interfaz animada.

* **Revisión:** 15 de noviembre de 2026  
* **Firmware documentado:** `SmartTraffic.ino` (`SmartTrafficPrioridad`, versión de API `prioridad-1`)

---

## 1. Descripción general y funcionamiento

**SmartTraffic IoT** es una maqueta de control vial desarrollada sobre **ESP8266** que gestiona un semáforo vehicular (Rojo, Amarillo, Verde) y dos cruces peatonales independientes (**Cruce A** y **Cruce B**) ubicados sobre **la misma calle**.

* **Exclusión mutua:** Las solicitudes, turnos y extensiones de A y B son independientes, pero comparten el control vehicular y un sensor ultrasónico `HC-SR04`. Nunca se habilita un verde peatonal junto con el verde vehicular, ni se activan A y B simultáneamente.
* **Temporización no bloqueante:** La Máquina de Estados Finitos (FSM) gestiona todas las fases mediante `millis()`, atendiendo en `loop()` el servidor HTTP, los pulsadores con antirrebote (`35 ms`) y la sincronización horaria NTP. *(Nota: las lecturas del sensor ultrasónico emplean `pulseIn()` con un tiempo máximo de espera de `30 ms`).*

### 1.1. Modos operativos

#### 1.1.1. Modo Diurno (Alternancia y Prioridad de Solicitudes)
* **Ciclo sin solicitudes:** Alterna automáticamente los cruces A y B con **10 s** de verde peatonal por turno, separados por **10 s** de circulación vehicular.
* **Con solicitud (Física o Web):** Concede **15 s** al cruce solicitado. Si el pedido llega mientras ese mismo cruce cursa su ciclo automático de 10 s, amplía el total a 15 s sin reiniciar el contador.
* **Prioridad FIFO:** El pedido pendiente más antiguo tiene prioridad sobre la alternancia automática. Se atiende un solo cruce por turno y luego se devuelve el paso a los vehículos.
* **Aviso de cierre:** Los últimos **5 s** de todo verde peatonal titilan (cada `250 ms`) antes de pasar a rojo fijo.

```text
Verde vehicular -> Amarillo 3 s -> Todo rojo 2 s
  -> Cruce A o Cruce B (10 s / 15 s + ext. 5 s)
  -> Todo rojo 2 s -> Amarillo solo 2 s -> Verde vehicular
```

#### 1.1.2. Reducción del Verde Vehicular por Prioridad Peatonal
El verde vehicular normal dura **10 s**. Si existe una solicitud peatonal pendiente, el tiempo se acorta hasta un mínimo de **6 s** (descuento máximo de `4 s`):
* Si el pedido llega durante el verde vehicular, se conserva el reloj transcurrido y se recorta el límite asegurando al menos **2 s** de aviso previo al amarillo (`candidato = min(10000, max(6000, ceil(t/1000)*1000 + 2000))`).
* La interfaz web muestra sobre el semáforo vehicular el descuento real aplicado (`-N` en rojo).

#### 1.1.3. Extensión Dinámica por Sensor Ultrasónico (`HC-SR04`)
* El sensor compartido se asigna al cruce peatonal activo y realiza una única lectura **1 s antes** de finalizar el tiempo base (a los `9 s` en turnos de `10 s`, o a los `14 s` en turnos de `15 s`).
* Si detecta presencia válida ($0 < \text{distancia} \le 10.0\text{ cm}$), concede **una única extensión de +5 s** con parpadeo verde continuo y muestra el aviso luminoso `+5` en la web.

#### 1.1.4. Modo Nocturno (Amarillo Intermitente y Cruce bajo Demanda)
* **Activación:** Conmuta automáticamente entre las **00:00 hs y las 05:59 hs** mediante hora NTP (`UTC-3`), o de forma manual desde la interfaz web.
* **Reposo (`ESTADO_NOCT_IDLE`):** Mantiene el semáforo vehicular en **amarillo intermitente** (parpadeo cada `500 ms`) y ambos cruces peatonales en rojo fijo. No existen cruces automáticos.
* **Ante solicitud peatonal (A o B):** Atiende los pedidos en orden de llegada (FIFO) ejecutando la transición segura:

```text
Amarillo vehicular intermitente (0.5 s), peatones A/B en rojo
  -> Solicitud A o B (FIFO)
  -> Amarillo vehicular fijo 3 s -> Todo rojo 2 s
  -> Peatón solicitado en verde: 15 s (últimos 5 s titila + 5 s opcionales por sensor)
  -> Todo rojo 2 s -> Retorno a Amarillo intermitente
```

#### 1.1.5. Modo Emergencia (Bloqueo Seguro)
* **Activación:** Desde el botón web **EMERGENCIA** o manteniendo presionado cualquier pulsador físico durante **3 s**.
* **Comportamiento:** Cancela las solicitudes pendientes y rechaza nuevas peticiones con código HTTP `503`.
* **Cierre diferido:** Si no hay peatones cruzando, bloquea inmediatamente todas las luces en rojo (`ESTADO_EMERGENCIA_ACTIVA`). Si A o B está cruzando, activa `emergenciaPendiente` para dejar finalizar la ventana peatonal segura y su despeje en rojo antes del bloqueo total.
* **Salida:** Al desactivar la emergencia, pasa obligatoriamente por **2 s de amarillo vehicular** (`ESTADO_D8_PREP_AUTOS`) antes de retomar el modo diurno o nocturno.

### 1.2. Tabla de tiempos configurados

| Parámetro | Valor | Función |
| :--- | :---: | :--- |
| `T_VERDE_VEHICULAR` | **10 s** | Verde vehicular diurno normal |
| `T_VERDE_PRIORIDAD` | **6 s** | Mínimo verde vehicular cuando hay solicitud peatonal |
| `T_AVISO_SOLICITUD` | **2 s** | Margen mínimo de aviso al recortar un verde en curso |
| `T_BASE_SIN_SOLICITUD` | **10 s** | Verde peatonal automático diurno |
| `T_CON_SOLICITUD` / `TIEMPO_NOCT_CRUCE` | **15 s** | Verde peatonal por solicitud (diurno y nocturno) |
| `T_PARPADEO_FINAL` | **5 s** | Tramo final de verde peatonal intermitente (`250 ms`) |
| `TIEMPO_EXTENSION` | **5 s** | Extensión única por presencia en sensor ($\le 10\text{ cm}$) |
| `T3_ADV_AUTOS` / `TIEMPO_NOCT_AMARILLO` | **3 s** | Amarillo vehicular fijo previo al cierre |
| `T4_DESPEJE` / `T7_DESPEJE` / `TIEMPO_NOCT_DESPEJE` | **2 s** | Despejes de seguridad en todo rojo |
| `T8_PREP_AUTOS` | **2 s** | Amarillo vehicular previo a habilitar el verde |
| Parpadeo nocturno (`NOCT_IDLE`) | **500 ms** | Ciclo de intermitencia del amarillo vehicular en noche |

---

## 2. Retención de eventos y concurrencia

* **Bucle principal (`loop()`):** Atiende `server.handleClient()`, muestrea pulsadores con antirrebote de `35 ms` y verifica el horario NTP sin usar `delay()` bloqueantes de fase.
* **Cola FIFO en RAM:** `pendienteA`, `pendienteB`, `ordenA` y `ordenB` retienen hasta un pedido por cruce y respetan el orden de llegada tanto en modo diurno como nocturno. Las peticiones duplicadas devuelven HTTP `409 Conflict`.
* **Ampliación sin reinicio:** Una solicitud recibida durante un turno automático de `10 s` extiende la duración a `15 s` conservando `tiempoInicioEstado`.
* **Optimización HTTP (`ETag`):** El endpoint `/status` genera una huella `ETag` del estado completo y responde `304 Not Modified` cuando no hay cambios, evitando saturar los sockets del ESP8266 ante consultas cada `250 ms`.

---

## 3. Estados y transiciones de la FSM

| Estado FSM | Salidas Luminosas | Duración | Condición de Transición |
| :--- | :--- | :---: | :--- |
| `ESTADO_D1_CIRC_VERTICAL` | Veh: Verde \| A/B: Rojo | 10 s (hasta 6 s c/pedido) | Fin de tiempo $\rightarrow$ selecciona cruce FIFO o alterno y pasa a `D3`. Cambio a noche $\rightarrow$ `NOCT_IDLE`. |
| `ESTADO_D3_ADV_AUTOS` | Veh: Amarillo \| A/B: Rojo | 3 s | Pasa a `D4`. Emergencia $\rightarrow$ `EMERGENCIA_ACTIVA`. |
| `ESTADO_D4_DESPEJE_1` | Todo Rojo | 2 s | Inicia cruce preparado (`D5` para A o `ESTADO_CRUCE_B` para B). Si cambió a noche $\rightarrow$ `D8`. |
| `ESTADO_D5_CRUCE_HORIZONTAL` | Veh: Rojo \| A: Verde \| B: Rojo | 10 s / 15 s (+5 s) | Fin de tiempo $\rightarrow$ `D7`. Evalúa sensor `1 s` antes del cierre. |
| `ESTADO_CRUCE_B` | Veh: Rojo \| A: Rojo \| B: Verde | 10 s / 15 s (+5 s) | Fin de tiempo $\rightarrow$ `D7`. Evalúa sensor `1 s` antes del cierre. |
| `ESTADO_D7_DESPEJE_2` | Todo Rojo | 2 s | Si `emergenciaPendiente` $\rightarrow$ `EMERGENCIA_ACTIVA`; en otro caso $\rightarrow$ `D8`. |
| `ESTADO_D8_PREP_AUTOS` | Veh: Amarillo \| A/B: Rojo | 2 s | Día $\rightarrow$ `D1`; Noche $\rightarrow$ `NOCT_IDLE`. |
| `ESTADO_NOCT_IDLE` | Veh: Amarillo intermitente \| A/B: Rojo | Indefinido (`500 ms`) | Con pedido pendiente $\rightarrow$ selecciona FIFO y pasa a `NOCT_AMARILLO_IR`. Día $\rightarrow$ `D1`. |
| `ESTADO_NOCT_AMARILLO_IR` | Veh: Amarillo fijo \| A/B: Rojo | 3 s | Pasa a `NOCT_ROJO_IR`. |
| `ESTADO_NOCT_ROJO_IR` | Todo Rojo | 2 s | Pasa a `NOCT_CRUCE_ACTIVO`. |
| `ESTADO_NOCT_CRUCE_ACTIVO` | Veh: Rojo \| A o B: Verde | 15 s (+5 s) | Fin de tiempo $\rightarrow$ `NOCT_ROJO_RETORNO`. Evalúa sensor `1 s` antes del cierre. |
| `ESTADO_NOCT_ROJO_RETORNO` | Todo Rojo | 2 s | Si `emergenciaPendiente` $\rightarrow$ `EMERGENCIA_ACTIVA`; Día $\rightarrow$ `D8`; Noche $\rightarrow$ `NOCT_IDLE`. |
| `ESTADO_EMERGENCIA_ACTIVA` | Todo Rojo | Hasta desactivación | Al salir de emergencia $\rightarrow$ `D8` y luego al modo de destino. |

---

## 4. Esquema de conexión y pines (ESP8266)

Todos los LED utilizan resistencias limitadoras de $220\,\Omega$ a masa (`GND`). Los pulsadores emplean resistencias internas `INPUT_PULLUP` con cierre a `GND`.

| Periférico | Pin Placa | GPIO | Configuración / Función |
| :--- | :---: | :---: | :--- |
| Semáforo Vehicular (Rojo / Amarillo / Verde) | `D6` / `D5` / `D2` | `12` / `14` / `4` | Salidas digitales activas en `HIGH` |
| Semáforo Peatonal A (Rojo / Verde) | `D8` / `D7` | `15` / `13` | Salidas digitales activas en `HIGH` |
| Semáforo Peatonal B (Rojo / Verde) | `D0` / `D1` | `16` / `5` | Salidas digitales activas en `HIGH` |
| Pulsadores Peatonales (A / B) | `D4` / `D3` | `2` / `0` | `INPUT_PULLUP` (pulso corto = cruce; `3 s` = emergencia) |
| Sensor `HC-SR04` (`TRIG`) | `RX` | `3` | Reasignado a `FUNCTION_3` tras `Serial.end()` |
| Sensor `HC-SR04` (`ECHO`) | `TX` | `1` | Reasignado a `FUNCTION_3` (con divisor resistivo `5V -> 3.3V`) |

---

## 5. Interfaz web y API HTTP

La aplicación web está embebida en memoria Flash (`PROGMEM`) y representa en tiempo real los semáforos, contadores, distancia del sensor, consola serie (`SerialDual` con búfer circular de `45` líneas) y una animación sincronizada de tráfico y peatones.

### 5.1. Endpoints principales

| Ruta | Método | Descripción | Códigos HTTP |
| :--- | :---: | :--- | :---: |
| `/` | `GET` | Entrega la interfaz web principal (`INDEX_HTML`) | `200`, `405` |
| `/status` | `GET` | Telemetría JSON (luces, tiempos, sensor, contadores y eventos) con soporte `ETag` | `200`, `304`, `405` |
| `/logs?desde=N` | `GET` | Devuelve los eventos nuevos del Monitor Serie Web desde el índice `N` | `200` |
| `/setModo?m=...` | `GET` / `POST` | Cambia entre `diurno`, `nocturno` o alterna `emergencia` | `200`, `202`, `400`, `405` |
| `/solicitar?c=1\|2` | `GET` / `POST` | Registra o amplía la solicitud peatonal para A (`1`) o B (`2`) | `201`, `400`, `409`, `503` |
| `/testSistema` | `GET` | Ejecuta lectura diagnóstica del `HC-SR04` y encola una solicitud de prueba en A | `202`, `503` |
| `/limpiarPendientes` | `GET` | Borra las solicitudes pendientes en cola (`pendienteA` y `pendienteB`) | `204` |
| `/limpiarHistorial` | `GET` | Reinicia a cero los contadores de cruces, extensiones y emergencias | `204` |
| `/limpiarMonitor` | `GET` | Limpia el historial del Monitor Serie Web | `204` |

*(Nota: Incluye además redirecciones de demostración `/index.html` [301], `/inicio` [302], `/verEstado` [303] y respuestas de control `/admin` [401/403], `/camara` [501] y `/lcd` [501]).*

### 5.2. Conectividad Wi-Fi y respaldo
* Intenta conectarse en modo Station (`WIFI_STA`) a las redes configuradas anunciando el dominio mDNS `http://smarttraffic.local`.
* Si no logra conexión, levanta automáticamente el punto de acceso local **`SmartTraffic-AP`** (`WIFI_AP`) para garantizar el control web sin depender de infraestructura externa.


---

## 6. Enlaces del proyecto

* Documentación y Póster (Overleaf): https://www.overleaf.com/project/6af343cbfe1c54f9bf22b40d

* Carpeta de Archivos y Evidencias (Google Drive): https://drive.google.com/drive/folders/1F8rVRTN8UfQmnTR4Fw7EnuWTwYQIsX-V?usp=drive_link


### 6.2. Resumen de funcionalidades destacadas
1. **Monitor Serie Web (`SerialDual`):** Permite depurar el sistema en tiempo real desde el navegador luego de liberar los pines `TX`/`RX` para el sensor `HC-SR04`.
2. **Automatización horaria NTP (`UTC-3`):** Conmuta automáticamente a amarillo intermitente nocturno entre las `00:00` y las `06:00 hs`.
3. **Resiliencia Multi-WiFi + SoftAP:** Asegura disponibilidad operativa tanto en redes locales como en modo autónomo.
4. **Accesibilidad y seguridad vial:** Cuenta regresiva visible en verdes peatonales, parpadeo de advertencia en los últimos `5 s`, extensión automática por presencia y cierre diferido seguro ante emergencias.
