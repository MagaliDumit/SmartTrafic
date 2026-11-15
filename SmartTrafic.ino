 #include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <time.h>

// Definición de pines D0-D10 por si se compila como "Generic ESP8266 Module"
#ifndef D0
  #define D0  16
  #define D1  5
  #define D2  4
  #define D3  0
  #define D4  2
  #define D5  14
  #define D6  12
  #define D7  13
  #define D8  15
  #define D9  3
  #define D10 1
#endif

/*
  SmartTraffic - ESP8266
  Cruces independientes: alternancia de 10s sin solicitudes / 15s con solicitud A o B (últimos 5s titila verde -> Rojo fijo directo) +
  Extensión +5s por sensor HC-SR04 + Temporizador solo en Verdes Peatonales +
  Horario Nocturno (00:00 a 06:00 hs) + Guía (?)
*/

// =====================================================================
// DEFINICIÓN DE TIPOS Y ESTADOS (ANTES DE FUNCIONES PARA ARDUINO IDE)
// =====================================================================
struct Boton {
  uint8_t       pin;
  uint8_t       cruce;
  bool          lecturaAnterior;
  bool          estadoEstable;
  unsigned long ultimoCambio;
  unsigned long inicioPulsacion;
  bool          pulsacionLarga;
  bool          emergenciaDisparada;
};

enum ModoOperacion { MODO_DIURNO, MODO_NOCTURNO, MODO_EMERGENCIA };
enum EstadoFSM {
  // Secuencia Diurna Sincronizada (6 Estados)
  ESTADO_D1_CIRC_VERTICAL,      // Veh: VERDE | PeaA: ROJO | PeaB: ROJO (10s)
  ESTADO_D3_ADV_AUTOS,          // 2. Veh: AMAR  | PeaA: ROJO | PeaB: ROJO (3s)
  ESTADO_D4_DESPEJE_1,          // 3. Veh: ROJO  | PeaA: ROJO | PeaB: ROJO (2s)
  ESTADO_D5_CRUCE_HORIZONTAL,   // 4. Veh: ROJO  | PeaA: VERDE (10s/15s, últimos 5s titila -> ROJO fijo) | PeaB: ROJO
  ESTADO_D7_DESPEJE_2,          // 5. Veh: ROJO  | PeaA: ROJO | PeaB: ROJO (2s)
  ESTADO_D8_PREP_AUTOS,         // 6. Veh: AMAR | PeaA: ROJO | PeaB: ROJO (2s)

  ESTADO_CRUCE_B,                // Veh rojo, A rojo, B verde

  // Estados auxiliares para Modo Nocturno bajo demanda
  ESTADO_NOCT_IDLE,             // Veh: AMAR PARP | PeaA: ROJO | PeaB: ROJO
  ESTADO_NOCT_AMARILLO_IR,      // Veh: AMAR FIJO | PeaA: ROJO | PeaB: ROJO
  ESTADO_NOCT_ROJO_IR,          // Todo ROJO despeje
  ESTADO_NOCT_CRUCE_ACTIVO,     // Peatón solicitado en VERDE (15s, últimos 5s titila + 5s ext -> ROJO fijo)
  ESTADO_NOCT_ROJO_RETORNO,     // Todo ROJO despeje retorno

  // Estado de Emergencia
  ESTADO_EMERGENCIA_ACTIVA      // Bloqueo de Seguridad (Todo ROJO)
};

// =====================================================================
// BÚFER DE HISTORIAL PARA MONITOR SERIE WEB
// =====================================================================
const uint8_t MAX_LOGS_MONITOR = 45;
String historialMonitor[MAX_LOGS_MONITOR];
unsigned long totalLogsEmitidos = 0;

String formatearTiempoUptime() {
  unsigned long seg = millis() / 1000;
  unsigned int min = seg / 60;
  unsigned int s = seg % 60;
  char buf[12];
  snprintf(buf, sizeof(buf), "[%02u:%02u] ", min, s);
  return String(buf);
}

void registrarEnHistorialWeb(const String& linea) {
  if (linea.length() == 0) return;
  String entrada = formatearTiempoUptime() + linea;
  if (entrada.length() > 110) {
    entrada = entrada.substring(0, 107) + "...";
  }
  uint8_t idx = totalLogsEmitidos % MAX_LOGS_MONITOR;
  historialMonitor[idx] = entrada;
  totalLogsEmitidos++;
}

// =====================================================================
// CONTROL DE SERIAL (UART EN ARRANQUE + HISTORIAL WEB PERMANENTE)
// =====================================================================
class SerialDual {
 private:
  String lineaActual = "";

 public:
  bool uartActivo = true;

  void begin(unsigned long baud) {
    if (uartActivo) ::Serial.begin(baud);
  }

  void end() {
    if (uartActivo) {
      ::Serial.flush();
      delay(50);
      ::Serial.end();
      uartActivo = false;
    }
  }

  template <typename T>
  void print(T val) {
    if (uartActivo) ::Serial.print(val);
    lineaActual += String(val);
  }

  void print(float val, int dec) {
    if (uartActivo) ::Serial.print(val, dec);
    lineaActual += String(val, dec);
  }

  void print(const IPAddress& ip) {
    if (uartActivo) ::Serial.print(ip);
    lineaActual += ip.toString();
  }

  template <typename T>
  void println(T val) {
    if (uartActivo) ::Serial.println(val);
    lineaActual += String(val);
    registrarEnHistorialWeb(lineaActual);
    lineaActual = "";
  }

  void println(float val, int dec) {
    if (uartActivo) ::Serial.println(val, dec);
    lineaActual += String(val, dec);
    registrarEnHistorialWeb(lineaActual);
    lineaActual = "";
  }

  void println(const IPAddress& ip) {
    if (uartActivo) ::Serial.println(ip);
    lineaActual += ip.toString();
    registrarEnHistorialWeb(lineaActual);
    lineaActual = "";
  }

  void println() {
    if (uartActivo) ::Serial.println();
    if (lineaActual.length() > 0) {
      registrarEnHistorialWeb(lineaActual);
      lineaActual = "";
    }
  }
};

SerialDual serialDual;
#define Serial serialDual

// =====================================================================
// CONFIGURACIÓN DE HARDWARE Y RED
// =====================================================================
ESP8266WebServer server(80);
bool mdnsIniciado = false;

#define PIN_PED_A_R      D8   // Rojo Peatonal A
#define PIN_PED_A_G      D7   // Verde Peatonal A
#define PIN_VEH_R        D6   // Rojo Vehicular
#define PIN_BTN_REQ_A    D4   // Botón A (entre D4 y GND, INPUT_PULLUP)
#define PIN_BTN_REQ_B    D3   // Botón B (entre D3 y GND, INPUT_PULLUP)
#define PIN_VEH_Y        D5   // Amarillo Vehicular
#define PIN_VEH_G        D2   // Verde Vehicular
#define PIN_PED_B_G      D1   // Verde Peatonal B
#define PIN_PED_B_R      D0   // Rojo Peatonal B

// HC-SR04 compartido: TRIG en RX (GPIO3) y ECHO en TX (GPIO1) para evitar interferencia USB
#define PIN_TRIG_SENSOR  3    // RX (GPIO3)
#define PIN_ECHO_SENSOR  1    // TX (GPIO1)

const char* NOMBRE_PLACA = "ESP8266 - Generic Module";

const char* ssidPrimaria = "Fibertel WiFi F1b 2.4GHz";
const char* passwordPrimaria = "cramer2981";
const char* ssidSecundaria = "MOVISTAR-WIFI6-8158";
const char* passwordSecundaria = "0800chupala";
const char* AP_SSID = "SmartTraffic-AP";
const char* AP_PASSWORD = "trafico123";
bool conectadoComoAP = false;

// =====================================================================
// VARIABLES GLOBALES DE FSM, HORARIO AUTOMÁTICO Y TIEMPOS
// =====================================================================
ModoOperacion modoActual = MODO_DIURNO;
ModoOperacion modoPrevio  = MODO_DIURNO;
EstadoFSM estadoActual   = ESTADO_D1_CIRC_VERTICAL;

// Control horario automático (00:00 hs a 05:59 hs -> Modo Nocturno)
const int HORA_INICIO_NOCTURNO = 0;   // 00:00 hs
const int HORA_FIN_NOCTURNO    = 6;   // 06:00 hs
bool anulacionManualHorario    = false;
bool ntpInicializado           = false;
bool franjaNocturnaAnterior    = false;
unsigned long tUltChequeoHora  = 0;

bool pendienteA = false;
bool pendienteB = false;
uint8_t crucePreparado = 0; // Turno elegido antes del amarillo.
uint8_t proximoCruceAutomatico = 1; // Alternancia A/B solo cuando no hay solicitudes.
const unsigned long T_VERDE_PRIORIDAD = 6000; // Mínimo de verde con solicitud pendiente.
const unsigned long T_AVISO_SOLICITUD = 2000; // Nunca adelantar un cierre a menos de 2 s por un pedido tardío.
unsigned long duracionVerdeVehicular = 10000;
uint8_t descuentoVehicularSeg = 0;
unsigned long eventoDescuentoVehicular = 0;
unsigned long eventoSolicitudA = 0, eventoSolicitudB = 0;
uint8_t extraSolicitudA = 0, extraSolicitudB = 0;
const unsigned long T_VERDE_VEHICULAR = 10000; // 10 s de circulación entre cruces.
uint8_t cruceActivo = 0; // 1 = Cruce A, 2 = Cruce B; ambos cruzan la misma calle
bool cruceSolicitadoActual = false; // true si el cruce en curso fue solicitado (15s)
unsigned long ordenA = 0;
unsigned long ordenB = 0;
unsigned long contadorSolicitudes = 0;

const unsigned long TIEMPO_ANTIRREBOTE = 35;

Boton btnA = {PIN_BTN_REQ_A, 1, HIGH, HIGH, 0, 0, false, false};
Boton btnB = {PIN_BTN_REQ_B, 2, HIGH, HIGH, 0, 0, false, false};

unsigned long crucesRealizados = 0;
unsigned long extensionesTiempo = 0;
unsigned long emergenciasRegistradas = 0;

unsigned long tiempoInicioEstado = 0;

// Variables heredadas de parpadeo nocturno y parpadeo de verdes peatonales.
bool          parpNoct = false;
unsigned long tUltParpNoct = 0;
bool          parpVerde = true;
unsigned long tUltParpVerde = 0;

// Medición del HC-SR04 y extensión activa
bool medicionHecha = false;
bool extensionActiva = false;
bool emergenciaPendiente = false;

// -1 significa "sin medición en este ciclo" (la web mostrará "--")
float ultimaDistanciaSensor = -1.0;

// Configuración de Tiempos (milisegundos)
const unsigned long T_BASE_SIN_SOLICITUD = 10000; // 10 s sin solicitudes (últimos 5 s titila)
const unsigned long T_CON_SOLICITUD      = 15000; // 15 s cuando se solicita A o B (últimos 5 s titila)
const unsigned long T_PARPADEO_FINAL     = 5000;  // Últimos 5 s dentro del ciclo titila en verde
const unsigned long TIEMPO_EXTENSION     = 5000;  // +5 s adicionales si el sensor detecta presencia <= 10 cm

const unsigned long T3_ADV_AUTOS         = 3000;  // 3 s
const unsigned long T4_DESPEJE           = 2000;  // 2 s
const unsigned long T7_DESPEJE           = 2000;  // 2 s
const unsigned long T8_PREP_AUTOS        = 2000;  // 2 s

// Tiempos auxiliares Modo Nocturno
const unsigned long TIEMPO_NOCT_AMARILLO = 3000;
const unsigned long TIEMPO_NOCT_DESPEJE  = 2000;
const unsigned long TIEMPO_NOCT_CRUCE    = 15000; // 15 s al solicitar A o B en modo nocturno (últimos 5 s titila)

unsigned long duracionCruceActual        = T_BASE_SIN_SOLICITUD;
const float UMBRAL_PRESENCIA_CM          = 10.0;  // 10 cm

// =====================================================================
// GESTIÓN DE HORA NTP Y FRANJA NOCTURNA (00:00 A 06:00 HS)
// =====================================================================
bool horaSincronizada() {
  return time(nullptr) > 1700000000L;
}

bool esHorarioNocturno() {
  if (!horaSincronizada()) return false;
  time_t ahora = time(nullptr);
  struct tm* infoTiempo = localtime(&ahora);
  if (!infoTiempo) return false;
  return (infoTiempo->tm_hour >= HORA_INICIO_NOCTURNO && infoTiempo->tm_hour < HORA_FIN_NOCTURNO);
}

String obtenerHoraFormateada() {
  if (!horaSincronizada()) return "Sin sincronizar";
  time_t ahora = time(nullptr);
  struct tm* infoTiempo = localtime(&ahora);
  char buf[20];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", infoTiempo->tm_hour, infoTiempo->tm_min, infoTiempo->tm_sec);
  return String(buf);
}

// =====================================================================
// FUNCIONES AUXILIARES, CUENTA REGRESIVA VERDE PEATONAL Y LOGS
// =====================================================================
int msASegundos(unsigned long ms) {
  return (ms == 0) ? 0 : (int)((ms + 999UL) / 1000UL);
}

// Indica si el semáforo verde peatonal del cruce indicado está en sus últimos 5 s o extendido
bool estaParpadeandoVerde(uint8_t cruce) {
  if (cruceActivo != cruce) return false;
  if (estadoActual != ESTADO_CRUCE_B &&
      estadoActual != ESTADO_D5_CRUCE_HORIZONTAL &&
      estadoActual != ESTADO_NOCT_CRUCE_ACTIVO) {
    return false;
  }
  if (extensionActiva) return true;
  unsigned long trans = millis() - tiempoInicioEstado;
  unsigned long rem = (duracionCruceActual > trans) ? (duracionCruceActual - trans) : 0;
  return (rem <= T_PARPADEO_FINAL);
}

// Calcula el tiempo restante únicamente para los semáforos verdes peatonales activos
void calcularTiemposVerdesPeatonales(int &tPedA, int &tPedB) {
  unsigned long trans = millis() - tiempoInicioEstado;
  tPedA = 0;
  tPedB = 0;

  switch (estadoActual) {
    case ESTADO_CRUCE_B: {
      unsigned long remD1 = (duracionCruceActual > trans) ? (duracionCruceActual - trans) : 0;
      tPedB = msASegundos(remD1);
      break;
    }

    case ESTADO_D5_CRUCE_HORIZONTAL: {
      unsigned long remD5 = (duracionCruceActual > trans) ? (duracionCruceActual - trans) : 0;
      tPedA = msASegundos(remD5);
      break;
    }

    case ESTADO_NOCT_CRUCE_ACTIVO: {
      unsigned long rem = (duracionCruceActual > trans) ? (duracionCruceActual - trans) : 0;
      if (cruceActivo == 1) tPedA = msASegundos(rem);
      if (cruceActivo == 2) tPedB = msASegundos(rem);
      break;
    }

    default:
      tPedA = 0;
      tPedB = 0;
      break;
  }
}

const char* obtenerNombreEstado(EstadoFSM estado) {
  switch (estado) {
    case ESTADO_D1_CIRC_VERTICAL:    return "D1_CIRC_VERTICAL (Veh:VDE | PeaA:ROJO | PeaB:ROJO)";
    case ESTADO_CRUCE_B: return "CRUCE_B (Veh:ROJO | PeaA:ROJO | PeaB:VDE)";
    case ESTADO_D3_ADV_AUTOS:        return "D3_ADV_AUTOS (Veh:AMAR | PeaA:ROJO | PeaB:ROJO)";
    case ESTADO_D4_DESPEJE_1:        return "D4_DESPEJE_1 (Todo ROJO)";
    case ESTADO_D5_CRUCE_HORIZONTAL: return "D5_CRUCE_HORIZONTAL (Veh:ROJO | PeaA:VDE | PeaB:ROJO)";
    case ESTADO_D7_DESPEJE_2:        return "D7_DESPEJE_2 (Todo ROJO)";
    case ESTADO_D8_PREP_AUTOS:       return "D8_PREP_AUTOS (Veh:AMAR | PeaA:ROJO | PeaB:ROJO)";
    case ESTADO_NOCT_IDLE:           return "NOCT_IDLE (Amarillo Intermitente)";
    case ESTADO_NOCT_AMARILLO_IR:    return "NOCT_AMARILLO_IR";
    case ESTADO_NOCT_ROJO_IR:        return "NOCT_ROJO_IR";
    case ESTADO_NOCT_CRUCE_ACTIVO:   return "NOCT_CRUCE_ACTIVO";
    case ESTADO_NOCT_ROJO_RETORNO:   return "NOCT_ROJO_RETORNO";
    case ESTADO_EMERGENCIA_ACTIVA:   return "EMERGENCIA_ACTIVA (Bloqueo Todo ROJO)";
    default:                         return "DESCONOCIDO";
  }
}

void cambiarEstadoFSM(EstadoFSM nuevoEstado) {
  Serial.print(F("[FSM] "));
  Serial.print(obtenerNombreEstado(estadoActual));
  Serial.print(F(" -> "));
  Serial.println(obtenerNombreEstado(nuevoEstado));

  estadoActual = nuevoEstado;
  tiempoInicioEstado = millis();
  parpVerde = true;
  tUltParpVerde = tiempoInicioEstado;
  if (nuevoEstado == ESTADO_NOCT_IDLE) {
    parpNoct = true;
    tUltParpNoct = tiempoInicioEstado;
    actualizarLuces(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW);
  }
}

void ajustarVerdePorSolicitudes() {
  if (modoActual != MODO_DIURNO || estadoActual != ESTADO_D1_CIRC_VERTICAL || (!pendienteA && !pendienteB)) return;
  unsigned long transcurrido = millis() - tiempoInicioEstado;
  unsigned long limiteConAviso = ((transcurrido + 999UL) / 1000UL) * 1000UL + T_AVISO_SOLICITUD;
  unsigned long candidato = limiteConAviso > T_VERDE_PRIORIDAD ? limiteConAviso : T_VERDE_PRIORIDAD;
  if (candidato > T_VERDE_VEHICULAR) candidato = T_VERDE_VEHICULAR;
  if (candidato < duracionVerdeVehicular) {
    duracionVerdeVehicular = candidato;
    descuentoVehicularSeg = (T_VERDE_VEHICULAR - candidato) / 1000UL;
    eventoDescuentoVehicular++;
    Serial.print(F("[PRIORIDAD] Descuento de verde vehicular: -"));
    Serial.print(descuentoVehicularSeg);
    Serial.println(F("s"));
  }
}

void registrarEventoSolicitud(uint8_t cruce) {
  uint8_t extra = modoActual == MODO_DIURNO ? (T_CON_SOLICITUD - T_BASE_SIN_SOLICITUD) / 1000UL : 0;
  if (cruce == 1) { eventoSolicitudA++; extraSolicitudA = extra; }
  else { eventoSolicitudB++; extraSolicitudB = extra; }
}

void iniciarCruceVerticalD1() {
  duracionVerdeVehicular = T_VERDE_VEHICULAR;
  descuentoVehicularSeg = 0;
  crucePreparado = 0;
  medicionHecha = false;
  extensionActiva = false;
  cruceActivo = 0;
  cruceSolicitadoActual = false;
  ultimaDistanciaSensor = -1.0;
  duracionCruceActual = T_BASE_SIN_SOLICITUD;
  cambiarEstadoFSM(ESTADO_D1_CIRC_VERTICAL);
  ajustarVerdePorSolicitudes();
}

void iniciarCrucePeatonal(uint8_t cruce) {
  medicionHecha = false;
  extensionActiva = false;
  ultimaDistanciaSensor = -1.0;
  cruceActivo = cruce;
  crucePreparado = cruce;
  cruceSolicitadoActual = cruce == 1 ? pendienteA : pendienteB;
  if (cruce == 1) pendienteA = false;
  else pendienteB = false;
  duracionCruceActual = cruceSolicitadoActual ? T_CON_SOLICITUD : T_BASE_SIN_SOLICITUD;
  proximoCruceAutomatico = cruce == 1 ? 2 : 1;
  Serial.print(F("[CRUCE] Turno independiente: "));
  Serial.print(cruce == 1 ? "A" : "B");
  Serial.println(cruceSolicitadoActual ? " | solicitud: 15s" : " | automatico: 10s");
  cambiarEstadoFSM(cruce == 1 ? ESTADO_D5_CRUCE_HORIZONTAL : ESTADO_CRUCE_B);
}

float medirDistanciaCM(int trigPin, int echoPin) {
  if (Serial.uartActivo) return 999.0;

  // Fuerza los pines TX/RX a modo GPIO (FUNCTION_3) desvinculándolos de UART0
  pinMode(trigPin, FUNCTION_3);
  pinMode(echoPin, FUNCTION_3);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  if (digitalRead(echoPin) == HIGH) {
    Serial.println(F("[HTTP 504 Gateway Timeout] Pin ECHO en HIGH fijo (verificar cableado/divisor)"));
    return 999.0;
  }

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 30000);
  if (duracion == 0) {
    Serial.println(F("[HTTP 504 Gateway Timeout] Sensor HC-SR04 sin respuesta (eco = 0, verificar 5V/TRIG/ECHO)"));
    return 999.0;
  }
  return (duracion * 0.0343) / 2.0;
}

void actualizarLuces(bool vehR, bool vehY, bool vehG, bool pedAR, bool pedAG, bool pedBR, bool pedBG) {
  // A y B cruzan la MISMA calzada: ninguna luz peatonal verde admite vehículos.
  if (pedAG || pedBG) { vehR = HIGH; vehY = LOW; vehG = LOW; }
  digitalWrite(PIN_VEH_R, vehR);
  digitalWrite(PIN_VEH_Y, vehY);
  digitalWrite(PIN_VEH_G, vehG);
  digitalWrite(PIN_PED_A_R, pedAR);
  digitalWrite(PIN_PED_A_G, pedAG);
  digitalWrite(PIN_PED_B_R, pedBR);
  digitalWrite(PIN_PED_B_G, pedBG);
}

// Devuelve código HTTP asociado al resultado del registro (201, 409, 503)
int registrarSolicitud(uint8_t cruce, const char* origen) {
  if (modoActual == MODO_EMERGENCIA) {
    Serial.println(F("[HTTP 503 Service Unavailable] Solicitud rechazada: MODO EMERGENCIA"));
    return 503;
  }

  if (cruce == 1) {
    // Si Cruce A está corriendo en ciclo normal de 10s sin solicitud, lo extiende a 15s
    if (estadoActual == ESTADO_D5_CRUCE_HORIZONTAL && !cruceSolicitadoActual) {
      cruceSolicitadoActual = true;
      duracionCruceActual += (T_CON_SOLICITUD - T_BASE_SIN_SOLICITUD);
      Serial.print(F("[SOLICITUD] Cruce A ampliado a 15s desde: "));
      Serial.println(origen);
      registrarEventoSolicitud(cruce);
      return 201;
    }
    if (pendienteA || cruceActivo == 1) {
      Serial.println(F("[HTTP 409 Conflict] Cruce A ya se encuentra activo o pendiente"));
      return 409;
    }
    pendienteA = true;
    ordenA = ++contadorSolicitudes;
    Serial.print(F("[SOLICITUD] Cruce A registrada (15s) desde: "));
    Serial.println(origen);
    registrarEventoSolicitud(cruce);
      return 201;
  }

  if (cruce == 2) {
    // Si Cruce B está corriendo en ciclo normal de 10s sin solicitud, lo extiende a 15s
    if (estadoActual == ESTADO_CRUCE_B && !cruceSolicitadoActual) {
      cruceSolicitadoActual = true;
      duracionCruceActual += (T_CON_SOLICITUD - T_BASE_SIN_SOLICITUD);
      Serial.print(F("[SOLICITUD] Cruce B ampliado a 15s desde: "));
      Serial.println(origen);
      registrarEventoSolicitud(cruce);
      return 201;
    }
    if (pendienteB || cruceActivo == 2) {
      Serial.println(F("[HTTP 409 Conflict] Cruce B ya se encuentra activo o pendiente"));
      return 409;
    }
    pendienteB = true;
    ordenB = ++contadorSolicitudes;
    Serial.print(F("[SOLICITUD] Cruce B registrada (15s) desde: "));
    Serial.println(origen);
    registrarEventoSolicitud(cruce);
      return 201;
  }

  return 400;
}

uint8_t seleccionarCrucePendiente() {
  if (pendienteA && pendienteB) return ordenA <= ordenB ? 1 : 2;
  if (pendienteA) return 1;
  if (pendienteB) return 2;
  return 0;
}

bool esEstadoPeatonalCritico() {
  return (estadoActual == ESTADO_CRUCE_B ||
          estadoActual == ESTADO_D5_CRUCE_HORIZONTAL ||
          estadoActual == ESTADO_NOCT_CRUCE_ACTIVO);
}

void activarEmergencia() {
  if (modoActual == MODO_EMERGENCIA) return;

  modoPrevio = modoActual;
  modoActual = MODO_EMERGENCIA;
  emergenciasRegistradas++;
  pendienteA = false;
  pendienteB = false;
  if (!esEstadoPeatonalCritico()) extensionActiva = false;

  Serial.println(F("[ALERTA] MODO EMERGENCIA ACTIVADO"));

  if (esEstadoPeatonalCritico()) {
    emergenciaPendiente = true;
  } else {
    emergenciaPendiente = false;
    cruceActivo = 0;
    cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
  }
}

void salirDeEmergenciaHacia(ModoOperacion modoDestino, bool esSeleccionManual) {
  if (modoActual != MODO_EMERGENCIA) return;

  modoActual = modoDestino;
  modoPrevio = modoDestino;
  emergenciaPendiente = false;
  extensionActiva = false;

  if (esSeleccionManual) {
    anulacionManualHorario = (modoDestino == MODO_DIURNO && esHorarioNocturno()) ||
                             (modoDestino == MODO_NOCTURNO && !esHorarioNocturno());
  } else {
    anulacionManualHorario = false;
  }

  Serial.print(F("[SISTEMA] MODO EMERGENCIA DESACTIVADO -> "));
  Serial.println(modoDestino == MODO_DIURNO ? F("DIURNO") : F("NOCTURNO"));

  if (estadoActual == ESTADO_EMERGENCIA_ACTIVA) {
    cruceActivo = 0;
    cambiarEstadoFSM(ESTADO_D8_PREP_AUTOS);
  }
}

void salirDeEmergenciaAutomatico() {
  if (modoActual != MODO_EMERGENCIA) return;

  if (esHorarioNocturno()) {
    Serial.println(F("[HORARIO 00:00-06:00] Salida de emergencia -> MODO NOCTURNO automatico"));
    salirDeEmergenciaHacia(MODO_NOCTURNO, false);
  } else {
    salirDeEmergenciaHacia(modoPrevio, false);
  }
}

void alternarEmergencia() {
  if (modoActual == MODO_EMERGENCIA) salirDeEmergenciaAutomatico();
  else activarEmergencia();
}

void gestionarHorarioAutomatico() {
  unsigned long ahoraMs = millis();
  if (ahoraMs - tUltChequeoHora < 1000) return;
  tUltChequeoHora = ahoraMs;

  if (!horaSincronizada()) return;

  bool enFranjaNocturna = esHorarioNocturno();

  if (!ntpInicializado) {
    ntpInicializado = true;
    franjaNocturnaAnterior = enFranjaNocturna;
    Serial.print(F("[NTP] Hora sincronizada: "));
    Serial.println(obtenerHoraFormateada());

    if (enFranjaNocturna && !anulacionManualHorario && modoActual == MODO_DIURNO) {
      modoActual = MODO_NOCTURNO;
      Serial.println(F("[HORARIO 00:00-06:00] Ingreso automatico a MODO NOCTURNO"));
      if (estadoActual == ESTADO_D1_CIRC_VERTICAL) {
        cruceActivo = 0;
        extensionActiva = false;
        ultimaDistanciaSensor = -1.0;
        cambiarEstadoFSM(ESTADO_NOCT_IDLE);
      }
    }
    return;
  }

  if (enFranjaNocturna != franjaNocturnaAnterior) {
    franjaNocturnaAnterior = enFranjaNocturna;
    anulacionManualHorario = false;

    if (enFranjaNocturna) {
      Serial.println(F("[HORARIO 00:00] Inicio horario nocturno automatico"));
      if (modoActual == MODO_DIURNO) {
        modoActual = MODO_NOCTURNO;
        if (estadoActual == ESTADO_D1_CIRC_VERTICAL) {
          cruceActivo = 0;
          extensionActiva = false;
          ultimaDistanciaSensor = -1.0;
          cambiarEstadoFSM(ESTADO_NOCT_IDLE);
        }
      }
    } else {
      Serial.println(F("[HORARIO 06:00] Fin horario nocturno -> MODO DIURNO"));
      if (modoActual == MODO_NOCTURNO) {
        modoActual = MODO_DIURNO;
        if (estadoActual == ESTADO_NOCT_IDLE) {
          iniciarCruceVerticalD1();
        }
      }
    }
  }
}

// =====================================================================
// GESTIÓN DE BOTONES CON FLAGS INDEPENDIENTES
// =====================================================================
void actualizarBoton(struct Boton &b) {
  bool lectura = digitalRead(b.pin);
  unsigned long ahora = millis();

  if (lectura != b.lecturaAnterior) {
    b.lecturaAnterior = lectura;
    b.ultimoCambio = ahora;
  }

  if (lectura != b.estadoEstable && ahora - b.ultimoCambio >= TIEMPO_ANTIRREBOTE) {
    b.estadoEstable = lectura;

    if (b.estadoEstable == LOW) {                 // Pulsado
      b.inicioPulsacion = ahora;
      b.pulsacionLarga = false;
      b.emergenciaDisparada = false;
    } else {                                      // Soltado
      if (!b.pulsacionLarga) {
        registrarSolicitud(b.cruce, b.cruce == 1 ? "Boton Fisico A" : "Boton Fisico B");
      }
      b.inicioPulsacion = 0;
      b.pulsacionLarga = false;
      b.emergenciaDisparada = false;
    }
  }

  // Pulsación larga (3 s): alterna emergencia una sola vez por pulsación
  if (b.estadoEstable == LOW && !b.emergenciaDisparada &&
      ahora - b.inicioPulsacion >= 3000) {
    b.pulsacionLarga = true;
    b.emergenciaDisparada = true;
    alternarEmergencia();
  }
}

// =====================================================================
// INTERFAZ WEB
// =====================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SmartTraffic · Prioridad peatonal</title>
  <style>
    :root{font-family:Inter,Segoe UI,system-ui,Arial,sans-serif;color-scheme:light}
    *{box-sizing:border-box}body{margin:0}button{font:inherit;cursor:pointer}button:focus-visible{outline:3px solid #348fe3;outline-offset:3px}
    .app{--page:#edf4ef;--paper:#fff;--ink:#14352f;--muted:#638078;--line:#d5e3da;--accent:#167853;--soft:#dff0e5;--shadow:0 16px 40px rgba(24,78,58,.08);background:var(--page);color:var(--ink);min-height:100vh;transition:background .8s,color .8s}
    .app.night{--page:#0d1722;--paper:#172636;--ink:#eef4f6;--muted:#9db3c2;--line:#2d4354;--accent:#9dd9d2;--soft:#253d4c;--shadow:0 16px 40px rgba(0,0,0,.19);color-scheme:dark}
    .shell{width:min(1180px,calc(100% - 36px));margin:auto;padding:28px 0 50px}
    header{display:flex;align-items:center;justify-content:space-between;gap:20px;flex-wrap:wrap;margin-bottom:28px}
    .brand{display:flex;align-items:center;gap:13px}.logo{height:44px;width:44px;border-radius:14px;background:var(--ink);display:grid;place-items:center;color:var(--paper)}
    .logo span{width:10px;height:26px;border-radius:8px;background:linear-gradient(#fa7770 0 31%,#f9d66a 32% 65%,#64d99c 66%)}
    .brand strong{display:block;font-size:22px;letter-spacing:-.055em}.brand small{display:block;color:var(--muted);font-size:12px;letter-spacing:.08em;text-transform:uppercase;margin-top:1px}
    .header-actions{display:flex;align-items:center;gap:10px;flex-wrap:wrap}.preview-mark{font-size:12px;color:var(--muted);border:1px solid var(--line);padding:9px 12px;border-radius:999px}
    .mode{display:flex;gap:3px;background:var(--soft);border-radius:999px;padding:4px}.mode button{border:0;border-radius:999px;padding:8px 16px;background:transparent;color:var(--muted);font-weight:700}
    .mode button[aria-pressed="true"]{background:var(--paper);color:var(--ink);box-shadow:0 2px 8px rgba(0,0,0,.1)}
    .headline{display:flex;align-items:end;justify-content:space-between;gap:20px;flex-wrap:wrap;margin-bottom:15px}
    h1{font-size:clamp(28px,4vw,46px);line-height:1.05;letter-spacing:-.065em;margin:0 0 8px}.subtitle{color:var(--muted);margin:0;font-size:14px}
    .live{display:flex;align-items:center;gap:9px;color:var(--accent);font-weight:750;font-size:14px}.live i{width:9px;height:9px;background:currentColor;border-radius:50%}
    .surface{background:var(--paper);border:1px solid var(--line);box-shadow:var(--shadow);border-radius:22px;overflow:hidden}
    .scene{--walk-distance:157px;--approach-distance:58px;position:relative;height:470px;overflow:hidden;background:#bfe4e6;transition:background .8s;isolation:isolate}
    .night .scene{background:#182f4c}.sky{position:absolute;inset:0 0 57%;background:linear-gradient(#8bcad7,#d9edf0);transition:background .8s}
    .night .sky{background:linear-gradient(#12263e,#314c6b)}
    .sun,.moon{position:absolute;width:76px;height:76px;right:10%;top:11%;border-radius:50%;transition:opacity .6s,transform .8s}
    .sun{background:#fff6c6;box-shadow:0 0 0 19px rgba(255,245,186,.18),0 0 70px #fff1b1}.moon{background:#f4f1db;box-shadow:0 0 0 16px rgba(239,244,255,.06),0 0 40px rgba(247,246,221,.2);opacity:0;transform:translateY(20px)}
    .night .sun{opacity:0;transform:translateY(-22px)}.night .moon{opacity:1;transform:translateY(0)}
    .cloud{position:absolute;width:130px;height:26px;left:13%;top:20%;border-radius:30px;background:rgba(255,255,255,.5);box-shadow:28px -16px 0 8px rgba(255,255,255,.43),65px 1px 0 -2px rgba(255,255,255,.45);transition:opacity .7s}.cloud.two{left:52%;top:35%;transform:scale(.65)}.night .cloud{opacity:0}
    .stars{position:absolute;inset:0 0 55%;opacity:0;transition:opacity .7s;background-image:radial-gradient(circle,#e5f3ff 1.2px,transparent 1.7px),radial-gradient(circle,#e5f3ff .9px,transparent 1.5px);background-size:97px 67px,141px 79px;background-position:9px 12px,44px 31px}.night .stars{opacity:.66}
    .buildings{position:absolute;bottom:56%;left:0;right:0;height:66px;display:flex;align-items:end;gap:13px;padding:0 18px;opacity:.48}.buildings i{display:block;background:#7da9ab;border-radius:4px 4px 0 0;width:8%;height:var(--h)}.night .buildings i{background:#284663}
    .grass-top,.grass-bottom{position:absolute;left:0;right:0;background:#80b99c}.grass-top{top:42%;height:20px}.grass-bottom{bottom:16%;height:3%}.night .grass-top,.night .grass-bottom{background:#315c5b}
    .walk-top,.walk-bottom{position:absolute;left:0;right:0;background:#d1dcd3}.walk-top{top:46%;height:27px}.walk-bottom{bottom:0;height:16%;background:repeating-linear-gradient(90deg,#d7dfd8 0 118px,#c9d4cc 118px 121px)}.night .walk-top,.night .walk-bottom{background:#647b84}.night .walk-bottom{background:repeating-linear-gradient(90deg,#647b84 0 118px,#536a74 118px 121px)}
    .road{position:absolute;top:52%;bottom:19%;left:0;right:0;background:#536774;transition:background .8s}.night .road{background:#293d50}
    .road:before{content:"";position:absolute;top:51%;left:0;right:0;height:3px;background:repeating-linear-gradient(90deg,#e8dbb6 0 38px,transparent 38px 72px);opacity:.7}
    .crosswalk{position:absolute;z-index:2;top:51%;height:32%;width:82px;background:repeating-linear-gradient(180deg,rgba(248,250,244,.95) 0 10px,transparent 10px 20px);border-left:2px solid rgba(255,255,255,.36);border-right:2px solid rgba(255,255,255,.36)}
    .crosswalk.a{left:26%}.crosswalk.b{left:71%}.cross-label{position:absolute;z-index:6;top:45%;transform:translateX(-50%);font-size:11px;font-weight:800;letter-spacing:.12em;color:#33584d;background:#f2f7ef;padding:5px 8px;border-radius:5px}.cross-label.a{left:calc(26% + 41px)}.cross-label.b{left:calc(71% + 41px)}
    .stop-mark{position:absolute;z-index:2;top:52%;height:31%;width:4px;background:rgba(250,252,247,.72)}.stop-mark.a{left:calc(26% - 4px)}.stop-mark.b{left:calc(71% - 4px)}
    .night .cross-label{background:#dce8df;color:#233f46}
    .signal{position:absolute;z-index:7;top:43%;left:4%;display:flex;gap:5px;background:#22343b;border:3px solid #d2e0da;border-radius:9px;padding:7px;box-shadow:0 5px 14px rgba(0,0,0,.19)}.signal i{height:16px;width:16px;border-radius:50%;background:#53646b;box-shadow:inset 0 2px 4px #263238}.signal i.on.red{background:#ff6c61;box-shadow:0 0 14px #ff6c61}.signal i.on.yellow{background:#ffd168;box-shadow:0 0 14px #ffd168}.signal i.on.green{background:#65e1a1;box-shadow:0 0 14px #65e1a1}
    .ped-signal{position:absolute;z-index:7;bottom:19.5%;display:flex;gap:4px;background:#22343b;border:2px solid #d2e0da;border-radius:7px;padding:5px}.ped-signal.a{left:calc(26% - 31px)}.ped-signal.b{left:calc(71% - 31px)}.ped-signal i{height:10px;width:10px;border-radius:50%;background:#52646b}.ped-signal i.on.red{background:#ff6c61;box-shadow:0 0 9px #ff6c61}.ped-signal i.on.green{background:#65e1a1;box-shadow:0 0 9px #65e1a1}
    .sensor-visual{position:absolute;z-index:7;bottom:19.5%;width:34px;height:20px;border:2px solid #c9ded7;border-radius:6px;background:#29424a;display:flex;align-items:center;justify-content:center;gap:4px}.sensor-visual.shared{left:calc(50% - 17px)}.sensor-visual i{width:9px;height:9px;border-radius:50%;background:#9dcad0;border:2px solid #173039}.sensor-visual:after{content:"";position:absolute;left:30px;top:-8px;width:18px;height:32px;border:2px solid #90c5b9;border-left:0;border-radius:50%;opacity:.65}.sensor-visual.detected{border-color:#f4c266;box-shadow:0 0 13px #f4c266}.sensor-visual.detected:after{border-color:#f4c266;opacity:1}
    .distance-scale{position:absolute;z-index:6;bottom:1.8%;height:16%;width:58px;color:#315b52;font-size:9px;font-weight:800}.night .distance-scale{color:#d4e6e4}.distance-scale.shared{left:calc(50% + 30px)}.distance-scale:before{content:"";position:absolute;left:5px;top:0;bottom:0;border-left:2px solid currentColor;opacity:.65}.distance-scale span{position:absolute;left:0;top:var(--at);transform:translateY(-50%);display:flex;align-items:center;gap:4px;white-space:nowrap}.distance-scale span:before{content:"";display:block;width:10px;border-top:2px solid currentColor}.distance-scale span:first-child{transform:none}.distance-scale span:last-child{transform:translateY(-100%)}
    .car{position:absolute;z-index:3;top:61%;width:102px;height:32px;background:#f0bd6b;border-radius:13px 20px 10px 10px;border-bottom:5px solid #9c593f;filter:drop-shadow(0 7px 5px rgba(0,0,0,.2))}.car:before{content:"";position:absolute;left:24px;top:-14px;width:50px;height:19px;border-radius:14px 14px 2px 2px;background:#f0bd6b;border:2px solid #894b3b}.car:after{content:"";position:absolute;left:15px;bottom:-11px;width:16px;height:16px;border-radius:50%;background:#182731;box-shadow:59px 0 #182731}.car .glass{position:absolute;left:32px;top:-9px;width:32px;height:11px;border-radius:7px 7px 2px 2px;background:#618d9b;z-index:1;opacity:.9}.car .lamp-front{position:absolute;right:-1px;top:12px;width:5px;height:6px;background:#fff0c2;border-radius:2px;box-shadow:0 0 6px #fff0c2}.car .lamp-rear{position:absolute;left:0;top:12px;width:4px;height:6px;background:#ec6b64;border-radius:2px}
    .car.blue{background:#a2d7df;border-bottom-color:#527c99}.car.blue:before{background:#a2d7df;border-color:#537c93}
    .car.moving{left:-130px;top:57%;will-change:left}.car.moving.second{top:69%}
    .person{position:absolute;z-index:5;top:84%;width:18px;height:46px;opacity:0;filter:drop-shadow(2px 4px 3px rgba(0,0,0,.3))}.person.a{left:calc(26% + 29px)}.person.b{left:calc(71% + 29px)}.person.follower{margin-left:22px}.person.sensor-extra{margin-left:-17px}.person.sensor-extra .body{background:#76b7b4}
    .person .head{display:block;width:12px;height:12px;border-radius:50%;background:#183e3b;margin:auto}.person .body{display:block;background:#e97461;width:13px;height:21px;border-radius:7px 7px 3px 3px;margin:1px auto 0;position:relative}.person .body:after{content:"";position:absolute;top:17px;left:2px;width:3px;height:17px;background:#143e43;transform:rotate(15deg);box-shadow:7px -1px #143e43}
    .person.follower .body{background:#f2c168}.scene.cross-a .person.a:not(.sensor-extra),.scene.cross-b .person.b:not(.sensor-extra){animation:walk 5.1s linear forwards}.scene.cross-a .person.a.follower,.scene.cross-b .person.b.follower{animation-delay:.65s}.scene.extended-a .person.a.sensor-extra,.scene.extended-b .person.b.sensor-extra{animation:walk 5.8s linear forwards}
    @keyframes walk{0%{opacity:0;transform:translateY(0)}8%{opacity:1}88%{opacity:1}100%{opacity:0;transform:translateY(calc(-1 * var(--walk-distance)))}}
    .scene-status{position:absolute;z-index:8;right:20px;top:18px;background:rgba(255,255,255,.86);backdrop-filter:blur(10px);border-radius:12px;padding:10px 13px;display:flex;align-items:center;gap:9px;color:#254740;font-weight:750;font-size:13px}.night .scene-status{background:rgba(19,38,55,.84);color:#eef8f5}
    .scene-status i{width:8px;height:8px;border-radius:50%;background:#40b783}.scene-status i.warn{background:#efa94c}.scene-status i.danger{background:#f27970}
    .scene-timer{position:absolute;z-index:8;left:18px;top:18px;min-width:118px;padding:9px 13px;background:rgba(255,255,255,.86);backdrop-filter:blur(10px);border-radius:13px;color:#254740}.night .scene-timer{background:rgba(19,38,55,.84);color:#eef8f5}.scene-timer strong{display:block;font-size:29px;letter-spacing:-.06em;line-height:1;font-variant-numeric:tabular-nums}.scene-timer small{display:block;font-size:11px;font-weight:650;margin-top:4px}
    .scene-bottom{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:16px 22px;flex-wrap:wrap}.phase{font-size:14px;font-weight:760}.phase small{display:block;color:var(--muted);font-size:12px;font-weight:500;margin-top:3px}.scene-commands{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
    .btn{border:1px solid var(--line);background:var(--paper);color:var(--ink);border-radius:11px;padding:10px 14px;font-weight:760}.btn:hover{filter:brightness(.95)}.btn.primary{background:var(--ink);border-color:var(--ink);color:var(--paper)}.night .btn.primary{background:#d3e9e5;color:#17333f;border-color:#d3e9e5}.btn.emergency{background:#d95852;border-color:#d95852;color:white}.btn:disabled{opacity:.5;cursor:not-allowed}
    .section-head{display:flex;justify-content:space-between;align-items:center;gap:12px;margin:30px 0 14px}h2{font-size:20px;letter-spacing:-.045em;margin:0}.section-head span{font-size:12px;color:var(--muted)}
    .cross-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}.cross-card{background:var(--paper);border:1px solid var(--line);border-radius:17px;padding:19px;display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center;transition:background .8s}.cross-card.active{border-color:#65b789;background:var(--soft)}.cross-card.pending{border-color:#dca65b}
    .cross-card h3{margin:0 0 4px;font-size:17px}.cross-card p{margin:0;color:var(--muted);font-size:13px}.cross-card .time{font-size:23px;font-weight:800;letter-spacing:-.06em;text-align:right}.cross-card .time small{display:block;font-size:12px;color:var(--muted);letter-spacing:0;font-weight:500}
    .cross-card button{grid-column:1/-1;justify-self:start}.cross-card .sensor-reading{grid-column:1/-1;display:flex;gap:7px;align-items:center;color:var(--muted);font-size:12px}.sensor-reading i{width:8px;height:8px;border-radius:50%;background:#91aaa5}.sensor-reading i.detected{background:#e7a844;box-shadow:0 0 7px #e7a844}
    .sensor-grid{display:grid;grid-template-columns:1fr;gap:14px}.sensor-card{background:var(--paper);border:1px solid var(--line);border-radius:17px;padding:18px}.sensor-card label{display:flex;align-items:center;justify-content:space-between;gap:12px;font-size:14px;font-weight:700}.sensor-card label strong{font-variant-numeric:tabular-nums;color:var(--accent)}.sensor-card small{color:var(--muted);font-size:12px}.sensor-card .sensor-result{margin-top:6px;font-size:12px;font-weight:700}
    .lower{display:grid;grid-template-columns:1.2fr .8fr;gap:14px;margin-top:22px}.panel{background:var(--paper);border:1px solid var(--line);border-radius:17px;padding:20px}.panel h2{margin-bottom:12px}
    .log{list-style:none;padding:0;margin:0}.log li{display:flex;gap:13px;padding:9px 0;border-top:1px solid var(--line);font-size:13px}.log li:first-child{border:0}.log time{color:var(--muted);min-width:43px;font-variant-numeric:tabular-nums}.empty{color:var(--muted);font-size:13px}
    .stats{display:flex;gap:16px;margin-bottom:16px}.stat{flex:1}.stat b{display:block;font-size:22px}.stat span{color:var(--muted);font-size:12px}.settings{display:flex;gap:8px;flex-wrap:wrap;border-top:1px solid var(--line);padding-top:16px}
    .diagnostic{border-top:1px solid var(--line);margin-top:16px;padding-top:14px}.diagnostic summary{cursor:pointer;font-size:13px;font-weight:700}.diagnostic dl{display:grid;grid-template-columns:1fr auto;gap:9px;margin:14px 0 0;font-size:12px}.diagnostic dt{color:var(--muted)}.diagnostic dd{margin:0;text-align:right;font-weight:700;font-variant-numeric:tabular-nums}
    .note{color:var(--muted);font-size:12px;line-height:1.5;margin:17px 3px 0}
    @media(max-width:760px){.shell{width:min(100% - 20px,620px);padding-top:16px}.scene{height:420px}.scene-status{right:9px;top:9px;font-size:11px}.crosswalk{width:53px}.cross-label.a{left:calc(26% + 27px)}.cross-label.b{left:calc(71% + 27px)}.person.a{left:calc(26% + 13px)}.person.b{left:calc(71% + 13px)}.signal{left:3%;top:43%}.lower{grid-template-columns:1fr}}
    @media(max-width:560px){.scene{--walk-distance:115px;height:390px}.cross-grid,.sensor-grid{grid-template-columns:1fr}.sun,.moon{width:55px;height:55px}.car,.car.moving.second{transform:scale(.7);transform-origin:left center}.crosswalk.a{left:29%}.crosswalk.b{left:72%}.stop-mark.a{left:calc(29% - 4px)}.stop-mark.b{left:calc(72% - 4px)}.cross-label.a{left:calc(29% + 26px)}.cross-label.b{left:calc(72% + 26px)}.person.a{left:calc(29% + 13px)}.person.b{left:calc(72% + 13px)}.ped-signal.a{left:calc(29% - 29px)}.ped-signal.b{left:calc(72% - 29px)}.sensor-visual.shared{left:calc(50% - 17px)}.distance-scale.shared{left:calc(50% + 28px)}.person{top:82%}.distance-scale{font-size:8px}}
    @media(prefers-reduced-motion:reduce){*,*:before,*:after{animation-duration:.001ms!important;transition-duration:.001ms!important}}
 
    .ped-signal i{display:grid;place-items:center;font:800 12px/1 Arial;font-style:normal;color:#103b25}
    .ped-signal i.green{width:24px;height:24px}.ped-signal i.red{align-self:center}
    .ped-signal .blink-dim{background:#a6c8b2}
    .ped-signal small{position:absolute;top:100%;left:0;white-space:nowrap;background:#e0f8e7;color:#145a37;padding:2px 5px;border-radius:4px;font-weight:800}
    .monitor{max-height:15rem;overflow:auto;background:#17282a;color:#e5f4ed;padding:10px;font:12px/1.5 monospace;overflow-wrap:anywhere}

    /* Semáforos legibles; el contador permanece dentro de la luz verde. */
    .signal{gap:7px;padding:9px;top:40%}.signal i{width:25px;height:25px}
    .ped-signal{gap:6px;padding:7px;bottom:19.5%;border-radius:10px}
    .ped-signal i.red{width:18px;height:18px}.ped-signal i.green{width:38px;height:38px;font-size:20px;font-weight:800}
    .ped-signal small{font-size:12px;top:calc(100% + 4px)}
    .guide{margin-top:18px;padding:18px 20px;background:var(--paper);border:1px solid var(--line);border-radius:16px;font-size:14px;line-height:1.6}
    .guide summary{font-weight:750;cursor:pointer}.guide p{margin:12px 0 0}.schedule{font-size:12px;color:var(--muted);margin:0 0 14px}
    .app.offline .scene{opacity:.65}.app.offline .person{animation-play-state:paused!important}
    @media(max-width:560px){.signal{left:2%;padding:6px;gap:4px}.signal i{width:19px;height:19px}.ped-signal{padding:5px;gap:4px}.ped-signal i.red{width:14px;height:14px}.ped-signal i.green{width:34px;height:34px;font-size:19px}}

    /* Las dos sendas cruzan la misma calle, sin cambios de posición. */
    .scene{--walk-distance:174px}
    .scene .person.a,.scene .person.b{top:calc(84% - 46px)}
    .scene .person.b{left:calc(71% + 29px)}
    @media(max-width:760px){.scene{--walk-distance:155px}.scene .person.b{left:calc(71% + 13px)}}
    @media(max-width:560px){.scene{--walk-distance:144px}.scene .person.b{left:calc(72% + 13px)}}
    /* Semáforo vehicular centrado; extensión legible bajo cada señal peatonal. */
    .scene .signal{left:50%;top:35%;transform:translateX(-50%)}
    .scene .ped-signal small{left:50%;transform:translateX(-50%);top:calc(100% + 6px);width:max-content;max-width:150px;white-space:normal;text-align:center;padding:5px 8px;border:1px solid #167853;box-shadow:0 3px 10px #14352f26;font-size:12px;line-height:1.25;z-index:9}
    .scene .ped-signal small[hidden]{display:none!important}

    /* Variantes de tránsito; solo se sustituyen fuera de la escena. */
    .car,.car.blue{width:var(--vehicle-width,102px);background:var(--car-paint,#f0bd6b);border-bottom-color:var(--car-trim,#9c593f)}
    .car:before,.car.blue:before{background:var(--car-paint,#f0bd6b);border-color:var(--car-trim,#9c593f)}
    .car.hatchback{border-radius:7px 13px 8px 8px}
    .car.hatchback:before{left:15px;top:-18px;width:51px;height:23px;border-radius:8px 10px 2px 2px}
    .car.hatchback:after{left:12px;box-shadow:52px 0 #182731}
    .car.hatchback .glass{left:22px;top:-12px;width:34px;height:14px;border-radius:4px 6px 1px 1px}
    .car.moto{height:14px;margin-top:16px;border-bottom-width:2px;border-radius:12px 7px 8px 6px}
    .car.moto:before{left:23px;top:-25px;width:12px;height:20px;border:0;border-radius:7px 7px 3px 3px;background:#29434b;transform:rotate(-12deg);box-shadow:2px -8px 0 -1px #e4b692}
    .car.moto:after{left:5px;bottom:-10px;width:18px;height:18px;border:3px solid #182731;background:#8fa5ab;box-shadow:40px 0 0 -3px #8fa5ab,40px 0 #182731}
    .car.moto .glass{left:45px;top:-9px;width:13px;height:3px;border-radius:1px;background:#182731;transform:rotate(-18deg)}
    .car.moto .lamp-front{top:0;right:0;height:5px}.car.moto .lamp-rear{top:3px;height:4px}

    /* Señales de pie; ambos semáforos peatonales son botones completos. */
    .scene .signal{flex-direction:column;align-items:center;gap:5px;padding:6px;top:calc(52% - 142px);border-width:2px;border-radius:12px}
    .scene .signal i{display:grid;place-items:center;width:28px;height:28px;font:800 16px/1 Arial;font-style:normal;color:#103b25}
    .scene .signal i.green{width:32px;height:32px}
    .scene .signal:after{content:"";position:absolute;left:calc(50% - 3px);top:100%;height:23px;width:6px;background:#455b61;z-index:-1}
    .scene .ped-signal{flex-direction:column;align-items:center;gap:4px;padding:4px;top:calc(81% + 2px);bottom:auto;left:calc(26% - 46px);border-radius:9px;cursor:pointer}
    .scene .ped-signal.b{left:calc(71% - 46px)}
    .scene .ped-signal i.red{width:22px;height:22px}.scene .ped-signal i.green{width:32px;height:32px;font-size:20px}
    .scene .ped-signal:after{content:"";position:absolute;top:100%;left:calc(50% - 2px);width:4px;height:14px;background:#455b61;z-index:-1}
    .scene .ped-signal:focus-visible{outline:3px solid #348fe3;outline-offset:3px}
    .scene .ped-signal:disabled{cursor:default}
    .scene .delta,.scene .ped-signal small.delta{position:absolute;top:auto;bottom:calc(100% + 8px);left:50%;transform:translateX(-50%);width:max-content;max-width:none;margin:0;padding:0;border:0!important;border-radius:0;background:none!important;box-shadow:none!important;font:900 26px/1 Arial;white-space:nowrap;pointer-events:none;opacity:0;z-index:12}
    .scene .delta.positive{color:#dcffeb;-webkit-text-stroke:1px #16dc7a;text-shadow:0 0 4px #18e786,0 0 12px #18e786}
    .scene .delta.negative{color:#ffe2de;-webkit-text-stroke:1px #ff403f;text-shadow:0 0 4px #ff403f,0 0 12px #ff403f}
    .scene .delta.flash{animation:delta-fade 2.8s ease-out forwards}
    .night-blink{animation:night-yellow 1s steps(1,end) infinite}
    @keyframes night-yellow{50%{opacity:.15;box-shadow:none}}
    @keyframes delta-fade{0%{opacity:0;transform:translate(-50%,5px)}12%,65%{opacity:1}100%{opacity:0;transform:translate(-50%,-15px)}}
    @media(max-width:560px){.scene .signal{top:calc(52% - 126px);gap:4px;padding:5px}.scene .signal i{width:24px;height:24px}.scene .signal i.green{width:28px;height:28px}.scene .signal:after{height:22px}.scene .ped-signal.a{left:calc(29% - 42px)}.scene .ped-signal.b{left:calc(72% - 42px)}.scene .ped-signal i.red{width:18px;height:18px}.scene .ped-signal i.green{width:28px;height:28px;font-size:18px}.scene .ped-signal:after{height:7px}.distance-scale.shared{left:calc(50% - 54px)}}

    /* Orden visual solicitado: verde arriba, rojo abajo. Las señales lógicas no se intercambian. */
    .scene .signal,.scene .ped-signal{flex-direction:column-reverse}
    /* Emergencia destacada sin alterar el tamaño de la escena ni tapar sus controles. */
    .app.emergency .surface{border-color:#e3323c;box-shadow:0 0 0 2px #e3323c55,0 8px 28px #b3183033}
    .scene.emergency:after{content:"";position:absolute;inset:0;z-index:11;pointer-events:none;border:4px solid #f13c48;box-shadow:inset 0 0 22px #f13c4844}
    .scene.emergency-pending:after{border-style:dashed}
    .app.emergency #modeLabel,.app.emergency #phaseTitle{color:#c51e30;font-weight:900}
    .app.night.emergency #modeLabel,.app.night.emergency #phaseTitle{color:#ff8c94}
    .scene.emergency .scene-status{background:#fff0f0;color:#a51224;border:1px solid #f13c4866}
    .scene.emergency .scene-status i{background:#e32d40;box-shadow:0 0 8px #e32d4066}
  </style>
</head>
<body>
<div class="app offline" id="app">
  <div class="shell">
    <header>
      <div class="brand"><div class="logo" aria-hidden="true"><span></span></div><div><strong>SmartTraffic</strong><small>Intersección inteligente</small></div></div>
      <div class="header-actions"><span class="preview-mark" id="connection">Conectando con ESP…</span><div class="mode" role="group" aria-label="Modo de operación"><button id="day" aria-pressed="true">☀ Día</button><button id="night" aria-pressed="false">☾ Noche</button></div></div>
    </header>
    <div class="headline"><div><h1>La calle, en tiempo real.</h1><p class="subtitle">Cruces A y B independientes · prioridad a las solicitudes.</p></div><div class="live"><i></i><span id="modeLabel">Modo diurno</span></div></div>
    <p class="schedule">Horario nocturno automático: 00:00 a 06:00, cuando la placa tiene la hora sincronizada. Los cambios de modo respetan la transición del controlador.</p>
    <section class="surface" aria-label="Intersección animada">
      <div class="scene" id="scene" data-stage="GREEN">
        <div class="sky"><div class="sun"></div><div class="moon"></div><div class="cloud"></div><div class="cloud two"></div><div class="stars"></div></div>
        <div class="buildings" aria-hidden="true"><i style="--h:28px"></i><i style="--h:58px"></i><i style="--h:41px"></i><i style="--h:52px"></i><i style="--h:30px"></i><i style="--h:64px"></i><i style="--h:39px"></i><i style="--h:51px"></i><i style="--h:31px"></i><i style="--h:60px"></i></div>
        <div class="grass-top"></div><div class="grass-bottom"></div><div class="walk-top"></div><div class="walk-bottom"></div><div class="road"></div>
        <div class="crosswalk a"></div><div class="crosswalk b"></div><div class="stop-mark a"></div><div class="stop-mark b"></div><div class="cross-label a">CRUCE A</div><div class="cross-label b">CRUCE B</div>
        <div class="signal" aria-label="Semáforo vehicular"><small id="vehicleDelta" class="delta negative" role="status" hidden></small><i class="red" id="r"></i><i class="yellow" id="y"></i><i class="green" id="g"></i></div>
        <button type="button" id="signalA" class="ped-signal a" aria-label="Semáforo peatonal A: solicitar cruce"><i class="red" id="paR"></i><i class="green" id="paG"></i><small id="extraA" class="delta positive" role="status" hidden></small></button><button type="button" id="signalB" class="ped-signal b" aria-label="Semáforo peatonal B: solicitar cruce"><i class="red" id="pbR"></i><i class="green" id="pbG"></i><small id="extraB" class="delta positive" role="status" hidden></small></button>
        <div class="sensor-visual shared" id="sensorVisual" aria-label="Sensor ultrasónico compartido"><i></i><i></i></div>
        <div class="distance-scale shared" aria-label="Escala de distancia del sensor compartido"><span style="--at:0%">0 m</span><span style="--at:25%">0,5</span><span style="--at:50%">1,0</span><span style="--at:75%">1,5</span><span style="--at:100%">2 m</span></div>
        <div id="car00" class="car moving" style="visibility:hidden"><span class="glass"></span><span class="lamp-front"></span><span class="lamp-rear"></span></div>
        <div id="car01" class="car moving" style="visibility:hidden"><span class="glass"></span><span class="lamp-front"></span><span class="lamp-rear"></span></div>
        <div id="car02" class="car moving" style="visibility:hidden"><span class="glass"></span><span class="lamp-front"></span><span class="lamp-rear"></span></div>
        <div id="car03" class="car moving" style="visibility:hidden"><span class="glass"></span><span class="lamp-front"></span><span class="lamp-rear"></span></div>
        <div id="car10" class="car moving second" style="visibility:hidden"><span class="glass"></span><span class="lamp-front"></span><span class="lamp-rear"></span></div>
        <div id="car11" class="car moving second" style="visibility:hidden"><span class="glass"></span><span class="lamp-front"></span><span class="lamp-rear"></span></div>
        <div id="car12" class="car moving second" style="visibility:hidden"><span class="glass"></span><span class="lamp-front"></span><span class="lamp-rear"></span></div>
        <div id="car13" class="car moving second" style="visibility:hidden"><span class="glass"></span><span class="lamp-front"></span><span class="lamp-rear"></span></div>
        <div class="person a"><span class="head"></span><span class="body"></span></div><div class="person a follower"><span class="head"></span><span class="body"></span></div><div class="person a sensor-extra"><span class="head"></span><span class="body"></span></div>
        <div class="person b"><span class="head"></span><span class="body"></span></div><div class="person b follower"><span class="head"></span><span class="body"></span></div><div class="person b sensor-extra"><span class="head"></span><span class="body"></span></div>
        <div class="scene-timer" aria-live="off"><strong id="sceneTime">05 s</strong><small id="sceneTimeLabel">Verde mínimo</small></div><div class="scene-status"><i id="statusDot"></i><span id="sceneState">Vehículos en verde</span></div>
      </div>
      <div class="scene-bottom"><div class="phase"><span id="phaseTitle">Esperando al controlador</span><small id="phaseDetail">Los datos se actualizan desde la maqueta.</small></div><div class="scene-commands"><button class="btn emergency" id="emergency">Activar emergencia</button></div></div>
    </section>
    <div class="section-head"><h2>Cruces peatonales</h2><span>Solicitudes con prioridad por orden de llegada · autos entre cruces</span></div>
    <div class="cross-grid">
      <article class="cross-card" id="cardA"><div><h3>Cruce A</h3><p id="descA">Esperando solicitud</p></div><div class="time" id="timeA">—<small>restante</small></div><div class="sensor-reading"><i id="sensorDotA"></i><span id="sensorReadA">Sensor compartido · disponible</span></div><button class="btn" id="requestA">Solicitar cruce A</button></article>
      <article class="cross-card" id="cardB"><div><h3>Cruce B</h3><p id="descB">Esperando solicitud</p></div><div class="time" id="timeB">—<small>restante</small></div><div class="sensor-reading"><i id="sensorDotB"></i><span id="sensorReadB">Sensor compartido · disponible</span></div><button class="btn" id="requestB">Solicitar cruce B</button></article>
    </div>
    <div class="section-head"><h2>Sensor de presencia compartido</h2><span>La placa asigna el sensor al cruce activo</span></div>
    <div class="sensor-grid">
      <article class="sensor-card"><label><span>Distancia medida</span><strong id="distanceValue">Sin eco</strong></label><small id="echo">Esperando lectura ultrasónica</small><div class="sensor-result" id="sensorResult">Sin datos del sensor</div></article>
    </div>
    <div class="lower">
      <section class="panel"><h2>Consola del ESP <small id="logCount"></small></h2><div id="events" class="monitor" aria-label="Registro del ESP"></div><button class="btn" id="clearLog">Limpiar consola</button></section>
      <section class="panel"><h2>Información y pruebas</h2><div class="stats"><div class="stat"><b id="countCross">0</b><span>Cruces</span></div><div class="stat"><b id="countExtension">0</b><span>Extensiones</span></div><div class="stat"><b id="countEmergency">0</b><span>Emergencias</span></div></div><div class="settings"><button class="btn" id="clear">Limpiar pendientes</button> <button class="btn" id="test">Prueba del sistema</button> <button class="btn" id="clearHistory">Reiniciar contadores</button></div><p id="commandMessage" class="note" role="status"></p></section>
    </div>
    <details class="guide"><summary>Guía de tiempos y funcionamiento</summary>
      <p><strong>Día:</strong> los vehículos tienen 10 s de verde entre cruces sin solicitudes. Un pedido pendiente puede reducirlo hasta 6 s, manteniendo al menos 2 s de aviso si llega durante el verde; el reloj no se reinicia y el aviso rojo muestra el descuento real. El turno siguiente atiende primero la solicitud pendiente más antigua; si no hay solicitudes, alterna A y B con 10 s peatonales. Solo cruza uno por turno y después vuelven a circular los autos. Una solicitud otorga 15 s; los últimos 5 s parpadea el verde peatonal. Si el turno ya está en amarillo o despeje, se completa antes de atender otros pedidos.</p>
      <p><strong>Luces vehiculares:</strong> verde (10 s normal, hasta 6 s con prioridad), amarillo (3 s), todo rojo (2 s), cruce solicitado o automático, todo rojo (2 s), amarillo solo (2 s) y verde. Rojo y amarillo nunca se encienden juntos.</p>
      <p><strong>Sensor:</strong> la placa mide 1 s antes de terminar el verde. Si detecta presencia a 10 cm o menos, agrega una única extensión de 5 s, indicada debajo del semáforo.</p>
      <p><strong>Noche:</strong> amarillo vehicular intermitente cada 0,5 s en reposo, con ambos peatones en rojo. Solo se atienden solicitudes: amarillo fijo (3 s), todo rojo (2 s), cruce peatonal solicitado (15 s, más la posible extensión) y todo rojo (2 s), antes de volver al amarillo intermitente. Los pedidos pendientes se atienden por orden de llegada.</p>
      <p><strong>Emergencia:</strong> se solicita desde este panel o manteniendo un pulsador físico 3 s. Si hay un cruce activo, el controlador completa la ventana segura antes del bloqueo en rojo. Durante el bloqueo rechaza nuevas solicitudes.</p>
      <p><strong>Contadores:</strong> los valores los informa la placa; no son cantidades de personas ni de vehículos detectados. El movimiento de autos y peatones es ilustrativo y sigue las luces del controlador.</p>
    </details>
    <p class="note">El panel reproduce las órdenes y la lectura informada por el controlador. La comprobación final de LED, cableado y sensor sigue siendo física.</p>
  </div>
</div>
<script>
(() => {
  const $ = id => document.getElementById(id);
  const scene = $('scene');
  let model = null;
  let connected = false;
  let lastStatusAt = 0;
  let previousFrame = 0;

  const vehicleVariants=[
    {kind:'sedan',width:102,paint:'#e6b66b',trim:'#946342',label:'Auto ocre'},
    {kind:'hatchback',width:90,paint:'#9fcdd7',trim:'#52788d',label:'Auto compacto celeste'},
    {kind:'moto',width:68,paint:'#d77b68',trim:'#844b43',label:'Moto terracota'},
    {kind:'sedan',width:102,paint:'#e0e5dd',trim:'#7c918c',label:'Auto claro'},
    {kind:'hatchback',width:90,paint:'#94b9a6',trim:'#507b69',label:'Auto compacto verde'},
    {kind:'moto',width:68,paint:'#91b9c8',trim:'#536e8b',label:'Moto azul'},
    {kind:'sedan',width:102,paint:'#cd8f89',trim:'#805c64',label:'Auto rosado'}
  ];
  let vehicleDeck=[];
  function carWidth(car){return Math.round((car?.width||102)*(window.innerWidth<=560?.7:1));}
  function configureVehicle(car){
    if(vehicleDeck.length===0){
      vehicleDeck=vehicleVariants.map((_,index)=>index);
      for(let i=vehicleDeck.length-1;i>0;i--){const j=Math.floor(Math.random()*(i+1));[vehicleDeck[i],vehicleDeck[j]]=[vehicleDeck[j],vehicleDeck[i]];}
    }
    let choice=vehicleDeck.pop();
    if(choice===car.variantIndex&&vehicleDeck.length>0){const alternative=vehicleDeck.pop();vehicleDeck.push(choice);choice=alternative;}
    const variant=vehicleVariants[choice];
    car.variantIndex=choice;car.width=variant.width;
    car.cruise=140+Math.random()*105;
    for(const kind of ['sedan','hatchback','moto'])car.el.classList.toggle(kind,variant.kind===kind);
    car.el.dataset.kind=variant.kind;
    car.el.style.setProperty('--vehicle-width',variant.width+'px');
    car.el.style.setProperty('--car-paint',variant.paint);
    car.el.style.setProperty('--car-trim',variant.trim);
    car.el.setAttribute('aria-label',variant.label);
  }
  const cars=[];
  const lanes=[{lane:0,nextSpawn:.2},{lane:1,nextSpawn:.9}];
  for(let lane=0;lane<2;lane++)for(let slot=0;slot<4;slot++){
    cars.push({el:$('car'+lane+slot),x:-200,v:0,cruise:0,lane,variantIndex:-1,width:102,active:false,ready:false});
  }
  function spawnVehicle(lane){
    const car=cars.find(car=>car.lane===lane&&!car.active);
    if(!car)return false;
    const width=scene.clientWidth;
    const gap=window.innerWidth<=560?16:24;
    const entryFront=-30;
    const traffic=cars.filter(other=>other.active&&other.lane===lane);
    if(traffic.some(other=>other.x<entryFront+gap))return false;
    if(!car.ready){configureVehicle(car);car.ready=true;}
    // Durante el cierre de B se reserva espacio entre sendas; no entra otro auto si la fila no cabe.
    if(model&&model.stopCross===2&&(!model.vehGreen||model.crossA||model.crossB)){
      const zebraWidth=window.innerWidth<=760?53:82;
      const farA=stopLine('A',width)+4+zebraWidth+4;
      const beforeB=stopLine('B',width);
      const farB=beforeB+4+zebraWidth+4;
      const pending=traffic.filter(other=>other.x<farB);
      const needed=pending.reduce((sum,other)=>sum+carWidth(other)+gap,carWidth(car));
      if(needed>beforeB-farA)return false;
    }
    car.x=-carWidth(car)-30;car.v=0;car.active=true;car.ready=false;
    car.el.style.left=car.x+'px';car.el.style.visibility='visible';
    return true;
  }
  function stopLine(side,width){const mobile=window.innerWidth<=560;return width*(side==='B'?(mobile?.72:.71):(mobile?.29:.26))-4;}
  function advanceTraffic(dt){
    const width=scene.clientWidth;
    const motionScale=Math.max(1,width/400);
    const phase=model.fase;
    const pedestrian=model.crossA||model.crossB;
    const free=!pedestrian&&!model.vehRed&&(model.vehGreen||phase==='NIGHT_BLINK');
    const preparing=phase==='DAY_YELLOW'||phase==='NIGHT_YELLOW';
    const side=model.stopCross===2?'B':model.stopCross===1?'A':null;
    const line=side?stopLine(side,width):null;
    const crossingWidth=window.innerWidth<=760?53:82;
    const farEdge=line===null?null:line+4+crossingWidth+4;
    const gap=window.innerWidth<=560?16:24;
    for(const lane of lanes){
      lane.nextSpawn-=dt;
      if(lane.nextSpawn<=0&&spawnVehicle(lane.lane))lane.nextSpawn=1.1+Math.random()*1.2;
      const traffic=cars.filter(car=>car.active&&car.lane===lane.lane).sort((a,b)=>b.x-a.x);
      let leader=null;
      for(const car of traffic){
        const size=carWidth(car);
        const cruise=car.cruise*motionScale;
        const stopX=line===null?null:line-size;
        if(pedestrian&&line!==null&&car.x+size>line&&car.x<farEdge){car.x=stopX;car.v=0;}
        const cleared=farEdge!==null&&car.x>=farEdge;
        let target=0,limit=Infinity;
        if(free||cleared)target=cruise;
        else if(stopX!==null&&car.x<=stopX){
          limit=stopX;
          target=Math.min(cruise,Math.sqrt(840*motionScale*Math.max(0,limit-car.x)));
        }else if(preparing&&!pedestrian)target=cruise;
        let plannedStop=limit;
        if(leader){
          const followLimit=leader.x-gap-size;
          const available=Math.max(0,followLimit-car.x);
          limit=Math.min(limit,followLimit);
          // Seguir la velocidad del líder: la distancia sola no obliga a frenar en cada fotograma.
          const followSpeed=Math.max(0,leader.v+1.6*(available-leader.v*.25));
          target=Math.min(target,followSpeed);
          if(!free&&!cleared){
            plannedStop=Math.min(plannedStop,leader.stopTarget-gap-size);
            // Reservar sitio según dónde terminará la fila, no por la posición instantánea del líder.
            for(const crossing of ['A','B']){
              const crossingStop=stopLine(crossing,width);
              const crossingExit=crossingStop+4+crossingWidth+4;
              if(car.x+size<=crossingStop+.001&&plannedStop<crossingExit){
                const waitX=crossingStop-size;
                plannedStop=Math.min(plannedStop,waitX);
                limit=Math.min(limit,waitX);
                target=Math.min(target,Math.sqrt(840*motionScale*Math.max(0,waitX-car.x)));
              }
            }
          }
        }
        car.stopTarget=plannedStop;
        const acceleration=(target>car.v?270:360)*motionScale;
        car.v+=Math.max(-acceleration*dt,Math.min(acceleration*dt,target-car.v));
        if(!free&&!cleared&&limit===Infinity&&!preparing)car.v=0;
        car.x=Math.min(limit,car.x+car.v*dt);
        if(limit-car.x<.2&&target<1)car.v=0;
        if(car.x>width+140){car.active=false;car.v=0;car.el.style.visibility='hidden';continue;}
        car.el.style.left=car.x.toFixed(1)+'px';
        leader=car;
      }
    }
  }
  function animateCars(frame){
    const elapsed=previousFrame?Math.max(0,Math.min((frame-previousFrame)/1000,.25)):0;
    previousFrame=frame;
    if(!connected||performance.now()-lastStatusAt>3000||!model||model.emergency){
      for(const car of cars)car.v=0;
    }else{
      // Pasos breves y estables: el tránsito no se vuelve más lento por dibujar menos fotogramas.
      let remaining=elapsed;
      while(remaining>0){const dt=Math.min(remaining,1/60);advanceTraffic(dt);remaining-=dt;}
    }
    requestAnimationFrame(animateCars);
  }

  let noticeState=null;
  function flashDelta(id,seconds,positive=true){
    if(seconds<=0)return;
    const el=$(id);el.hidden=false;el.textContent=(positive?'+':'−')+seconds;
    el.classList.remove('flash');void el.offsetWidth;el.classList.add('flash');
    el.setAttribute('aria-label',seconds+' segundos '+(positive?'añadidos':'descontados'));
  }
  function showTimeNotices(data){
    const now={a:Number(data.solicitud_event_a)||0,b:Number(data.solicitud_event_b)||0,ext:Number(data.cnt_ext)||0,discount:Number(data.descuento_evento)||0};
    if(noticeState){
      if(now.a>noticeState.a)flashDelta('extraA',Number(data.solicitud_extra_a)||0);
      if(now.b>noticeState.b)flashDelta('extraB',Number(data.solicitud_extra_b)||0);
      if(now.ext>noticeState.ext&&(data.ext_cruce===1||data.ext_cruce===2))flashDelta(data.ext_cruce===1?'extraA':'extraB',Number(data.extra_seg)||0);
      if(now.discount>noticeState.discount)flashDelta('vehicleDelta',Number(data.descuento_veh_seg)||0,false);
    }
    noticeState=now;
  }

  let cursor = 0;
  let commandBusy = false;
  let pollRequest = Promise.resolve();
  function lamp(id, on, seconds=0, blink=false){
    const el=$(id);
    el.classList.toggle('on',Boolean(on));
    el.classList.toggle('blink-dim',!on&&blink&&seconds>0);
    el.textContent=(on||blink)&&seconds>0?seconds:'';
  }
  function render(data){
    const night=data.modo==='nocturno';
    const emergency=data.modo==='emergencia';
    const activeA=data.t_ped_a>0, activeB=data.t_ped_b>0;
    const allRed=Boolean(data.veh_r&&data.ped_a_r&&data.ped_b_r&&!data.veh_y&&!data.veh_g&&!data.ped_a_g&&!data.ped_b_g);
    $('app').classList.toggle('emergency',emergency);
    scene.classList.toggle('emergency',emergency);
    scene.classList.toggle('emergency-pending',emergency&&!allRed);
    // A y B son cruces de la misma calzada.
    const state=data.estado_fsm||'';
    const phase=state.startsWith('NOCT_IDLE')?'NIGHT_BLINK':state.startsWith('D8_PREP_AUTOS')?'RETURN_YELLOW':state.startsWith('NOCT_AMARILLO_IR')?'NIGHT_YELLOW':activeA||activeB?'CROSSING':emergency?'EMERGENCY':data.veh_g?'DAY_GREEN':data.veh_y?'DAY_YELLOW':'ALL_RED';
    const prepared=phase==='NIGHT_BLINK'?0:Number(data.cruce_preparado)||Number(data.cruce_activo)||0;
    const stopCross=activeA?1:activeB?2:prepared;
    model={fase:phase,vehRed:Boolean(data.veh_r),vehGreen:Boolean(data.veh_g),crossA:activeA,crossB:activeB,emergency,stopCross};
    $('app').classList.toggle('night',night);
    // La senda B permanece fija, también durante las fases en rojo.
    scene.classList.toggle('cross-a',activeA);
    scene.classList.toggle('cross-b',activeB);
    scene.classList.toggle('extended-a',activeA&&data.ext_cruce===1);
    scene.classList.toggle('extended-b',activeB&&data.ext_cruce===2);
    scene.dataset.stage=phase;
    $('connection').textContent=data.placa||'ESP conectado';
    $('modeLabel').textContent=emergency?(allRed?'EMERGENCIA ACTIVA':'EMERGENCIA SOLICITADA'):'Modo '+data.modo;
    $('day').setAttribute('aria-pressed',String(data.modo==='diurno'));
    $('night').setAttribute('aria-pressed',String(night));
    $('emergency').textContent=emergency?'Desactivar emergencia':'Activar emergencia';
    const activeName=activeA?'A':'B';
    const preparedName=prepared===1?'A':prepared===2?'B':null;
    $('sceneState').textContent=emergency?(activeA||activeB?'Emergencia solicitada · termina cruce '+activeName:allRed?'Emergencia activa · todo rojo':'Emergencia solicitada · cerrando señales'):activeA?'Cruce A habilitado':activeB?'Cruce B habilitado':data.veh_g?(night?'Noche · vehículos en verde':'Vehículos en verde'):phase==='RETURN_YELLOW'?'Amarillo · preparando vehículos':preparedName?'Preparando cruce '+preparedName:phase==='NIGHT_BLINK'?'Noche · amarillo intermitente':'Despeje · todo rojo';
    $('phaseTitle').textContent=$('sceneState').textContent;
    $('phaseDetail').textContent='Pendientes: '+([data.pend_a?'A':'',data.pend_b?'B':''].filter(Boolean).join(', ')||'ninguno');
    const pedestrianTime=data.t_ped_a||data.t_ped_b;
    const vehicleTime=Number(data.t_vehicular)||0;
    $('sceneTime').textContent=pedestrianTime>0?pedestrianTime+' s':data.veh_g?(vehicleTime+' s'):'—';
    $('sceneTimeLabel').textContent=pedestrianTime>0?'Tiempo peatonal':data.veh_g?'Verde vehicular':phase==='NIGHT_BLINK'?'Amarillo intermitente':'Tiempo peatonal';
    lamp('r',data.veh_r); lamp('y',phase==='NIGHT_BLINK'||data.veh_y); $('y').classList.toggle('night-blink',phase==='NIGHT_BLINK'); lamp('g',data.veh_g,vehicleTime);
    lamp('paR',data.ped_a_r); lamp('paG',data.ped_a_g,data.t_ped_a,data.blink_a);
    lamp('pbR',data.ped_b_r); lamp('pbG',data.ped_b_g,data.t_ped_b,data.blink_b);
    showTimeNotices(data);
    const distance=Number(data.dist_sensor);
    const distanceText=distance>=0?distance.toFixed(1)+' cm':'—';
    const assigned=data.cruce_activo===1?'A':data.cruce_activo===2?'B':null;
    const presence=distance>0&&distance<=10;
    $('distanceValue').textContent=distanceText;
    $('echo').textContent=distance<0?'Sin medición en este ciclo':'Lectura informada por el ESP · umbral del firmware: 10 cm';
    $('sensorVisual').classList.toggle('detected',presence);
    $('sensorResult').textContent=assigned?'Sensor asignado al cruce '+assigned:'Sin cruce asignado';
    for(const [side,seconds,pending] of [['A',data.t_ped_a,data.pend_a],['B',data.t_ped_b,data.pend_b]]){
      $('card'+side).classList.toggle('active',seconds>0);
      $('card'+side).classList.toggle('pending',Boolean(pending));
      $('desc'+side).textContent=seconds>0?'Cruce habilitado':pending?'Solicitud pendiente':'Esperando solicitud';
      $('time'+side).firstChild.textContent=seconds>0?seconds+' s':'—';
      $('sensorDot'+side).classList.toggle('detected',assigned===side&&presence);
      $('sensorRead'+side).textContent=assigned===side?'Sensor compartido · '+distanceText:'Sensor compartido';
    }
    $('countCross').textContent=data.cnt_cruces;
    $('countExtension').textContent=data.cnt_ext;
    $('countEmergency').textContent=data.cnt_emerg;
    $('requestA').disabled=emergency;
    $('requestB').disabled=emergency;
    $('signalA').disabled=emergency;
    $('signalB').disabled=emergency;
  }
  async function refresh(){
    try{
      const response=await fetch('/status');
      if(response.status===304) return;
      if(!response.ok) throw new Error(response.status);
      render(await response.json());
      connected=true;
      lastStatusAt=performance.now();
      $('app').classList.remove('offline');
    }catch(error){connected=false; $('app').classList.add('offline'); $('connection').textContent='Sin conexión · última lectura';}
  }
  async function logs(){
    const response=await fetch('/logs?desde='+cursor,{cache:'no-store'});
    if(!response.ok) return;
    const data=await response.json();
    const consoleEl=$('events');
    if(data.total<cursor){cursor=0;consoleEl.replaceChildren();return;}
    const follow=consoleEl.scrollHeight-consoleEl.scrollTop-consoleEl.clientHeight<35;
    for(const line of data.logs){const row=document.createElement('div');row.textContent=line;consoleEl.append(row);}
    while(consoleEl.childElementCount>150) consoleEl.firstElementChild.remove();
    cursor=data.total;
    $('logCount').textContent=cursor+' eventos';
    if(follow) consoleEl.scrollTop=consoleEl.scrollHeight;
  }
  async function command(url){
    if(commandBusy) return;
    commandBusy=true;
    try{
      await pollRequest.catch(()=>{});
      const response=await fetch(url,{headers:{'X-Client-Info':'100-continue'}});
      $('commandMessage').textContent=(await response.text())||String(response.status);
      if(response.ok&&url==='/limpiarMonitor'){cursor=0;$('events').replaceChildren();}
      await refresh(); await logs();
    }catch(error){$('commandMessage').textContent='No se pudo contactar al ESP';}
    finally{commandBusy=false;}
  }
  const routes={day:'/setModo?m=diurno',night:'/setModo?m=nocturno',emergency:'/setModo?m=emergencia',requestA:'/solicitar?c=1',requestB:'/solicitar?c=2',signalA:'/solicitar?c=1',signalB:'/solicitar?c=2',clear:'/limpiarPendientes',test:'/testSistema',clearHistory:'/limpiarHistorial',clearLog:'/limpiarMonitor'};
  for(const [id,url] of Object.entries(routes)) $(id).onclick=()=>command(url);
  let pollLogs=true;
  async function poll(){
    if(!commandBusy){
      pollRequest=(async()=>{await refresh(); if(pollLogs&&connected) await logs();})();
      try{await pollRequest;}
      catch(error){$('logCount').textContent='Consola sin respuesta';}
      finally{pollRequest=Promise.resolve(); pollLogs=!pollLogs;}
    }
    setTimeout(poll,250);
  }
  poll();
  requestAnimationFrame(animateCars);
})();
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// UTILIDADES HTTP (1xx, 2xx, 3xx, 4xx, 5xx)
// =====================================================================
String escaparJSON(const String& texto) {
  String salida = "";
  salida.reserve(texto.length() + 8);
  for (size_t i = 0; i < texto.length(); i++) {
    char c = texto[i];
    if (c == '\"') salida += "\\\"";
    else if (c == '\\') salida += "\\\\";
    else if (c == '\n') salida += "\\n";
    else if (c == '\r') {}
    else salida += c;
  }
  return salida;
}

void verificarInformativos1xx() {
  if (server.hasHeader("Expect") && server.header("Expect").indexOf("100-continue") >= 0) {
    WiFiClient cliente = server.client();
    if (cliente && cliente.connected()) {
      cliente.print(F("HTTP/1.1 100 Continue\r\n\r\n"));
    }
    Serial.println(F("[HTTP 100 Continue] Encabezados recibidos, cliente procede"));
  }
}

void enviarRespuestaHTTP(int codigo, const char* tipo, const String& cuerpo, const char* descripcion, bool loguear = true) {
  if (loguear) {
    Serial.print(F("[HTTP "));
    Serial.print(codigo);
    Serial.print(F(" "));
    Serial.print(descripcion);
    Serial.print(F("] "));
    Serial.println(server.uri());
  }
  server.send(codigo, tipo, cuerpo);
}

void enviarRedireccionHTTP(int codigo, const char* destino, const char* descripcion) {
  Serial.print(F("[HTTP "));
  Serial.print(codigo);
  Serial.print(F(" "));
  Serial.print(descripcion);
  Serial.print(F("] "));
  Serial.print(server.uri());
  Serial.print(F(" -> "));
  Serial.println(destino);

  server.sendHeader("Location", destino, true);
  server.send(codigo, "text/plain", descripcion);
}

// =====================================================================
// RUTAS SERVIDOR WEB Y EVENTOS
// =====================================================================
void handleRoot() {
  if (server.method() != HTTP_GET) {
    server.sendHeader("Allow", "GET");
    enviarRespuestaHTTP(405, "text/plain", "405 Method Not Allowed", "Method Not Allowed");
    return;
  }
  Serial.println(F("[HTTP 200 OK] Interfaz Web principal cargada (/)"));
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  if (server.method() != HTTP_GET) {
    server.sendHeader("Allow", "GET");
    enviarRespuestaHTTP(405, "text/plain", "405 Method Not Allowed", "Method Not Allowed");
    return;
  }

  bool vehR = digitalRead(PIN_VEH_R);
  bool vehY = digitalRead(PIN_VEH_Y);
  bool vehG = digitalRead(PIN_VEH_G);

  bool pedAR = digitalRead(PIN_PED_A_R);
  bool pedAG = digitalRead(PIN_PED_A_G);
  bool pedBR = digitalRead(PIN_PED_B_R);
  bool pedBG = digitalRead(PIN_PED_B_G);

  int tPedA = 0, tPedB = 0;
  calcularTiemposVerdesPeatonales(tPedA, tPedB);

  uint8_t blinkA = estaParpadeandoVerde(1) ? 1 : 0;
  uint8_t blinkB = estaParpadeandoVerde(2) ? 1 : 0;
  uint8_t extCruce = extensionActiva ? cruceActivo : 0;

  unsigned long transcurridoVerde = millis() - tiempoInicioEstado;
  int tVehicular = 0;
  if (estadoActual == ESTADO_D1_CIRC_VERTICAL) {
    tVehicular = msASegundos(transcurridoVerde < duracionVerdeVehicular ? duracionVerdeVehicular - transcurridoVerde : 0);

  }
  float distSensor = ultimaDistanciaSensor;
  String modoStr = (modoActual == MODO_DIURNO) ? "diurno" :
                   (modoActual == MODO_NOCTURNO) ? "nocturno" : "emergencia";

  String etag = "\"" + modoStr + "-" + String(estadoActual) + "-" +
                String(vehR) + String(vehY) + String(vehG) +
                String(pedAR) + String(pedAG) + String(pedBR) + String(pedBG) + "-" +
                String(tPedA) + "-" + String(tPedB) + "-" + String(extCruce) + "-" +
                String(pendienteA) + String(pendienteB) + "-" + String(crucePreparado) + "-" + String(tVehicular) + "-" + String(eventoSolicitudA) + "-" + String(eventoSolicitudB) + "-" + String(eventoDescuentoVehicular) + "-" +
                String(crucesRealizados) + "-" + String(extensionesTiempo) + "-" +
                String(emergenciasRegistradas) + "-" + String((int)(distSensor * 10)) + "\"";

  if (server.hasHeader("If-None-Match") && server.header("If-None-Match") == etag) {
    server.send(304, "text/plain", "");
    return;
  }

  String json = "{";
  json += "\"placa\":\"" + String(NOMBRE_PLACA) + "\",";
  json += "\"modo\":\"" + modoStr + "\",";
  json += "\"version\":\"prioridad-1\",";
  json += "\"estado_fsm\":\"" + String(obtenerNombreEstado(estadoActual)) + "\",";
  json += "\"cruce_preparado\":" + String(crucePreparado) + ",";
  json += "\"t_vehicular\":" + String(tVehicular) + ",";
  json += "\"descuento_veh_seg\":" + String(descuentoVehicularSeg) + ",";
  json += "\"descuento_evento\":" + String(eventoDescuentoVehicular) + ",";
  json += "\"solicitud_event_a\":" + String(eventoSolicitudA) + ",";
  json += "\"solicitud_event_b\":" + String(eventoSolicitudB) + ",";
  json += "\"solicitud_extra_a\":" + String(extraSolicitudA) + ",";
  json += "\"solicitud_extra_b\":" + String(extraSolicitudB) + ",";
  json += "\"extra_seg\":" + String(extensionActiva ? TIEMPO_EXTENSION / 1000 : 0) + ",";
  json += "\"veh_r\":" + String(vehR) + ",";
  json += "\"veh_y\":" + String(vehY) + ",";
  json += "\"veh_g\":" + String(vehG) + ",";
  json += "\"ped_a_r\":" + String(pedAR) + ",";
  json += "\"ped_a_g\":" + String(pedAG) + ",";
  json += "\"ped_b_r\":" + String(pedBR) + ",";
  json += "\"ped_b_g\":" + String(pedBG) + ",";
  json += "\"t_ped_a\":" + String(tPedA) + ",";
  json += "\"t_ped_b\":" + String(tPedB) + ",";
  json += "\"blink_a\":" + String(blinkA) + ",";
  json += "\"blink_b\":" + String(blinkB) + ",";
  json += "\"ext_cruce\":" + String(extCruce) + ",";
  json += "\"dist_sensor\":" + String(distSensor) + ",";
  json += "\"cruce_activo\":" + String(cruceActivo) + ",";
  json += "\"pend_a\":" + String(pendienteA) + ",";
  json += "\"pend_b\":" + String(pendienteB) + ",";
  json += "\"cnt_cruces\":" + String(crucesRealizados) + ",";
  json += "\"cnt_ext\":" + String(extensionesTiempo) + ",";
  json += "\"cnt_emerg\":" + String(emergenciasRegistradas);
  json += "}";

  server.sendHeader("ETag", etag);
  server.send(200, "application/json", json);
}

void handleLogs() {
  unsigned long desde = 0;
  if (server.hasArg("desde")) {
    desde = server.arg("desde").toInt();
  }

  unsigned long inicioDisponible = (totalLogsEmitidos > MAX_LOGS_MONITOR)
                                     ? (totalLogsEmitidos - MAX_LOGS_MONITOR)
                                     : 0;
  if (desde < inicioDisponible) {
    desde = inicioDisponible;
  }

  String json = "{\"total\":" + String(totalLogsEmitidos) + ",\"logs\":[";
  bool primero = true;
  for (unsigned long i = desde; i < totalLogsEmitidos; i++) {
    if (!primero) json += ",";
    primero = false;
    json += "\"" + escaparJSON(historialMonitor[i % MAX_LOGS_MONITOR]) + "\"";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

void handleSetModo() {
  verificarInformativos1xx();

  if (server.method() != HTTP_GET && server.method() != HTTP_POST) {
    server.sendHeader("Allow", "GET, POST");
    enviarRespuestaHTTP(405, "text/plain", "405 Method Not Allowed", "Method Not Allowed");
    return;
  }

  if (!server.hasArg("m")) {
    enviarRespuestaHTTP(400, "text/plain", "400 Bad Request: falta parametro 'm'", "Bad Request");
    return;
  }

  String m = server.arg("m");
  if (m != "diurno" && m != "nocturno" && m != "emergencia") {
    enviarRespuestaHTTP(400, "text/plain", "400 Bad Request: modo invalido", "Bad Request");
    return;
  }

  Serial.println(F("[HTTP 102 Processing] Aplicando cambio de modo operativo..."));

  if (m == "diurno") {
    if (modoActual == MODO_EMERGENCIA) {
      salirDeEmergenciaHacia(MODO_DIURNO, true);
    } else if (modoActual == MODO_NOCTURNO) {
      modoActual = MODO_DIURNO;
      anulacionManualHorario = esHorarioNocturno();
      if (estadoActual == ESTADO_NOCT_IDLE) {
        iniciarCruceVerticalD1();
      }
    }
    enviarRespuestaHTTP(200, "text/plain", "200 OK: Modo Diurno", "OK");
  } else if (m == "nocturno") {
    if (modoActual == MODO_EMERGENCIA) {
      salirDeEmergenciaHacia(MODO_NOCTURNO, true);
    } else if (modoActual == MODO_DIURNO) {
      modoActual = MODO_NOCTURNO;
      anulacionManualHorario = !esHorarioNocturno();
      if (estadoActual == ESTADO_D1_CIRC_VERTICAL) {
        cruceActivo = 0;
        extensionActiva = false;
        ultimaDistanciaSensor = -1.0;
        cambiarEstadoFSM(ESTADO_NOCT_IDLE);
      }
    }
    enviarRespuestaHTTP(200, "text/plain", "200 OK: Modo Nocturno", "OK");
  } else if (m == "emergencia") {
    if (modoActual == MODO_EMERGENCIA) {
      salirDeEmergenciaAutomatico();
      enviarRespuestaHTTP(200, "text/plain", "200 OK: Emergencia desactivada", "OK");
    } else {
      activarEmergencia();
      enviarRespuestaHTTP(202, "text/plain", "202 Accepted: Emergencia activada", "Accepted");
    }
  }
}

void handleSolicitar() {
  verificarInformativos1xx();

  if (server.method() != HTTP_GET && server.method() != HTTP_POST) {
    server.sendHeader("Allow", "GET, POST");
    enviarRespuestaHTTP(405, "text/plain", "405 Method Not Allowed", "Method Not Allowed");
    return;
  }

  if (!server.hasArg("c")) {
    enviarRespuestaHTTP(400, "text/plain", "400 Bad Request: falta parametro 'c'", "Bad Request");
    return;
  }

  uint8_t c = server.arg("c").toInt();
  if (c != 1 && c != 2) {
    enviarRespuestaHTTP(400, "text/plain", "400 Bad Request: cruce debe ser 1 o 2", "Bad Request");
    return;
  }

  int resultado = registrarSolicitud(c, "Interfaz Web");
  if (resultado == 201) {
    enviarRespuestaHTTP(201, "text/plain", "201 Created: Solicitud registrada", "Created");
  } else if (resultado == 409) {
    server.send(409, "text/plain", "409 Conflict: Cruce ya pendiente o activo");
  } else if (resultado == 503) {
    server.send(503, "text/plain", "503 Service Unavailable: Modo Emergencia");
  } else {
    enviarRespuestaHTTP(500, "text/plain", "500 Internal Server Error", "Internal Server Error");
  }
}

void handleTestSistema() {
  if (modoActual == MODO_EMERGENCIA) {
    enviarRespuestaHTTP(503, "text/plain", "503 Service Unavailable: En Emergencia", "Service Unavailable");
    return;
  }

  Serial.println(F("[HTTP 102 Processing] Ejecutando prueba diagnostica del sistema..."));
  float distPrueba = medirDistanciaCM(PIN_TRIG_SENSOR, PIN_ECHO_SENSOR);
  ultimaDistanciaSensor = distPrueba;

  Serial.print(F("[TEST] Lectura HC-SR04 = "));
  Serial.print(distPrueba, 1);
  Serial.println(F(" cm | Solicitud de prueba encolada"));

  if (!pendienteA && cruceActivo != 1) {
    pendienteA = true;
    ordenA = ++contadorSolicitudes;
  }
  enviarRespuestaHTTP(202, "text/plain", "202 Accepted: Prueba iniciada", "Accepted");
}

void handleLimpiarPendientes() {
  pendienteA = false;
  pendienteB = false;
  enviarRespuestaHTTP(204, "text/plain", "", "No Content");
}

void handleLimpiarHistorial() {
  crucesRealizados = 0;
  extensionesTiempo = 0;
  emergenciasRegistradas = 0;
  enviarRespuestaHTTP(204, "text/plain", "", "No Content");
}

void handleLimpiarMonitor() {
  totalLogsEmitidos = 0;
  Serial.println(F("[HTTP 204 No Content] Monitor serie limpiado"));
  server.send(204, "text/plain", "");
}

void handleAdminProtegido() {
  if (!server.hasHeader("Authorization")) {
    server.sendHeader("WWW-Authenticate", "Basic realm=\"SmartTraffic Admin\"");
    enviarRespuestaHTTP(401, "text/plain", "401 Unauthorized", "Unauthorized");
    return;
  }
  enviarRespuestaHTTP(403, "text/plain", "403 Forbidden: Acceso denegado", "Forbidden");
}

void handleNoImplementado() {
  enviarRespuestaHTTP(501, "text/plain", "501 Not Implemented", "Not Implemented");
}

void handleNotFound() {
  enviarRespuestaHTTP(404, "text/plain", "404 Not Found: Recurso no encontrado", "Not Found");
}

// =====================================================================
// SETUP Y LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("========================================="));
  Serial.println(F("   DIAGNOSTICO DE CONEXION WI-FI ESP8266  "));
  Serial.println(F("========================================="));
  Serial.print(F("[PLACA] "));
  Serial.println(NOMBRE_PLACA);

  // Salidas de semáforos
  pinMode(PIN_PED_A_R, OUTPUT);
  pinMode(PIN_PED_A_G, OUTPUT);
  pinMode(PIN_VEH_R, OUTPUT);
  pinMode(PIN_VEH_Y, OUTPUT);
  pinMode(PIN_VEH_G, OUTPUT);
  pinMode(PIN_PED_B_G, OUTPUT);
  pinMode(PIN_PED_B_R, OUTPUT);

  // Pulsadores en D4 y D3
  pinMode(PIN_BTN_REQ_A, INPUT_PULLUP);
  pinMode(PIN_BTN_REQ_B, INPUT_PULLUP);

  // Estado inicial: vehículos en verde y ambos cruces peatonales en rojo.
  cruceActivo = 0;
  cruceSolicitadoActual = false;
  duracionCruceActual = T_BASE_SIN_SOLICITUD;
  actualizarLuces(LOW, LOW, HIGH, HIGH, LOW, HIGH, LOW);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.hostname("SmartTraffic");

  bool conectado = false;
  const char* redes[] = { ssidPrimaria, ssidSecundaria };
  const char* claves[] = { passwordPrimaria, passwordSecundaria };

  for (uint8_t i = 0; i < 2 && !conectado; i++) {
    Serial.print(F("[WIFI] Intentando conectar a: "));
    Serial.println(redes[i]);

    WiFi.disconnect();
    delay(100);
    WiFi.begin(redes[i], claves[i]);

    unsigned long inicioIntento = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - inicioIntento < 10000) {
      delay(250);
      Serial.print(F("."));
      yield();
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      conectado = true;
    } else {
      Serial.print(F("[WIFI] Fallo de conexion con: "));
      Serial.println(redes[i]);
    }
  }

  Serial.println(F("========================================="));

  if (conectado) {
    conectadoComoAP = false;
    mdnsIniciado = MDNS.begin("smarttraffic");
    if (mdnsIniciado) MDNS.addService("http", "tcp", 80);

    // Sincronización horaria NTP (Argentina UTC-3)
    configTime(-3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

    Serial.println(F("[WIFI] CONEXION ESTABLECIDA EXITOSAMENTE"));
    Serial.print(F("[WIFI] Red Activa: "));
    Serial.println(WiFi.SSID());
    Serial.print(F("[WIFI] Direccion IP Local: "));
    Serial.println(WiFi.localIP());
    if (mdnsIniciado) {
      Serial.println(F("[WIFI] mDNS Activo: http://smarttraffic.local"));
    }
  } else {
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    conectadoComoAP = WiFi.softAP(AP_SSID, AP_PASSWORD);

    if (conectadoComoAP) {
      Serial.println(F("[WIFI] PUNTO DE ACCESO (AP) CREADO"));
      Serial.print(F("[WIFI] SSID AP: "));
      Serial.println(AP_SSID);
      Serial.print(F("[WIFI] Direccion IP AP: "));
      Serial.println(WiFi.softAPIP());
    } else {
      Serial.println(F("[ERROR] No se pudo levantar el modo AP."));
    }
  }
  Serial.println(F("========================================="));

  // Recolectar cabeceras HTTP
  server.collectHeaders("Expect", "If-None-Match", "Authorization");

  // Rutas principales (2xx)
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/logs", handleLogs);
  server.on("/setModo", handleSetModo);
  server.on("/solicitar", handleSolicitar);
  server.on("/testSistema", handleTestSistema);
  server.on("/limpiarPendientes", handleLimpiarPendientes);
  server.on("/limpiarHistorial", handleLimpiarHistorial);
  server.on("/limpiarMonitor", handleLimpiarMonitor);

  // Rutas de redirección (3xx)
  server.on("/index.html", []() { enviarRedireccionHTTP(301, "/", "Moved Permanently"); });
  server.on("/inicio", []() { enviarRedireccionHTTP(302, "/", "Found"); });
  server.on("/verEstado", []() { enviarRedireccionHTTP(303, "/status", "See Other"); });

  // Rutas de control de acceso / no implementadas (4xx / 5xx)
  server.on("/admin", handleAdminProtegido);
  server.on("/camara", handleNoImplementado);
  server.on("/lcd", handleNoImplementado);
  server.onNotFound(handleNotFound);

  server.begin();

  Serial.println(F("[WEB] SERVIDOR HTTP ACTIVO EN PUERTO 80"));
  Serial.println(F("[SISTEMA] UART cerrado; HC-SR04 activo (TRIG=RX/GPIO3, ECHO=TX/GPIO1)"));

  // Finaliza la salida serie física y reasigna TX/RX a modo GPIO (FUNCTION_3)
  Serial.end();

  pinMode(PIN_TRIG_SENSOR, FUNCTION_3);
  pinMode(PIN_ECHO_SENSOR, FUNCTION_3);
  pinMode(PIN_TRIG_SENSOR, OUTPUT);
  digitalWrite(PIN_TRIG_SENSOR, LOW);
  pinMode(PIN_ECHO_SENSOR, INPUT);

  tiempoInicioEstado = millis();
}

void loop() {
  server.handleClient();
  if (mdnsIniciado) MDNS.update();

  actualizarBoton(btnA);
  actualizarBoton(btnB);
  gestionarHorarioAutomatico();

  // Muestrear después de las órdenes que pueden cambiar estado y su instante inicial.
  unsigned long tiempoActual = millis();

  // ===================================================================
  // MÁQUINA DE ESTADOS FINITOS (FSM)
  // ===================================================================
  switch (estadoActual) {

    // -----------------------------------------------------------------
    // SECUENCIA DIURNA SINCRONIZADA
    // -----------------------------------------------------------------
    case ESTADO_D1_CIRC_VERTICAL: {
      ajustarVerdePorSolicitudes();
      cruceActivo = 0;
      actualizarLuces(LOW, LOW, HIGH, HIGH, LOW, HIGH, LOW);
      if (modoActual == MODO_EMERGENCIA) {
        cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
      } else if (modoActual == MODO_NOCTURNO) {
        cambiarEstadoFSM(ESTADO_D3_ADV_AUTOS);
      } else if (tiempoActual - tiempoInicioEstado >= duracionVerdeVehicular) {
        uint8_t solicitado = seleccionarCrucePendiente();
        crucePreparado = solicitado ? solicitado : proximoCruceAutomatico;
        cambiarEstadoFSM(ESTADO_D3_ADV_AUTOS);
      }
      break;
    }

    case ESTADO_CRUCE_B: {
      // Cruce B independiente: autos en rojo durante todo el verde peatonal.
      cruceActivo = 2;
      unsigned long transcurrido = tiempoActual - tiempoInicioEstado;
      unsigned long tiempoBaseD1 = cruceSolicitadoActual ? T_CON_SOLICITUD : T_BASE_SIN_SOLICITUD;

      if (!medicionHecha && transcurrido >= (tiempoBaseD1 - 1000)) {
        medicionHecha = true;
        float dist = medirDistanciaCM(PIN_TRIG_SENSOR, PIN_ECHO_SENSOR);
        ultimaDistanciaSensor = dist;

        Serial.print(F("[HC-SR04] Distancia medida (Cruce B): "));
        Serial.print(dist, 1);
        Serial.println(F(" cm"));

        if (dist > 0 && dist <= UMBRAL_PRESENCIA_CM) {
          extensionesTiempo++;
          extensionActiva = true;
          duracionCruceActual += TIEMPO_EXTENSION;
          Serial.println(F("[ALERTA] Presencia <= 10 cm: +5s de extension en Cruce B"));
        }
      }

      bool luzVerdePeatB = HIGH;
      if (estaParpadeandoVerde(2)) {
        if (tiempoActual - tUltParpVerde >= 250) {
          tUltParpVerde = tiempoActual;
          parpVerde = !parpVerde;
        }
        luzVerdePeatB = parpVerde;
      }
      actualizarLuces(HIGH, LOW, LOW, HIGH, LOW, LOW, luzVerdePeatB);

      if (transcurrido >= duracionCruceActual) {
        crucesRealizados++;
        cruceActivo = 0;
        extensionActiva = false;
        ultimaDistanciaSensor = -1.0;
        cambiarEstadoFSM(ESTADO_D7_DESPEJE_2);
      }
      break;
    }

    case ESTADO_D3_ADV_AUTOS:
      if (modoActual == MODO_EMERGENCIA) {
        cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
        break;
      }
      // 2. Advertencia Autos (3s): Veh Amarillo, Pea A Rojo fijo, Pea B Rojo fijo
      cruceActivo = 0;
      actualizarLuces(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW);

      if (tiempoActual - tiempoInicioEstado >= T3_ADV_AUTOS) {
        cambiarEstadoFSM(ESTADO_D4_DESPEJE_1);
      }
      break;

    case ESTADO_D4_DESPEJE_1:
      if (modoActual == MODO_EMERGENCIA) {
        cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
        break;
      }
      cruceActivo = 0;
      actualizarLuces(HIGH, LOW, LOW, HIGH, LOW, HIGH, LOW);
      if (tiempoActual - tiempoInicioEstado >= T4_DESPEJE) {
        if (modoActual == MODO_NOCTURNO) {
          crucePreparado = 0;
          cambiarEstadoFSM(ESTADO_D8_PREP_AUTOS);
        } else {
          iniciarCrucePeatonal(crucePreparado == 2 ? 2 : 1);
        }
      }
      break;

    case ESTADO_D5_CRUCE_HORIZONTAL: {
      // 4. Cruce Horizontal A: 10s sin solicitud / 15s si se solicitó A (últimos 5s titila -> Rojo fijo directo)
      cruceActivo = 1;
      unsigned long transcurrido = tiempoActual - tiempoInicioEstado;
      unsigned long tiempoBaseD5 = cruceSolicitadoActual ? T_CON_SOLICITUD : T_BASE_SIN_SOLICITUD;

      if (!medicionHecha && transcurrido >= (tiempoBaseD5 - 1000)) {
        medicionHecha = true;
        float dist = medirDistanciaCM(PIN_TRIG_SENSOR, PIN_ECHO_SENSOR);
        ultimaDistanciaSensor = dist;

        Serial.print(F("[HC-SR04] Distancia medida (Cruce A): "));
        Serial.print(dist, 1);
        Serial.println(F(" cm"));

        if (dist > 0 && dist <= UMBRAL_PRESENCIA_CM) {
          extensionesTiempo++;
          extensionActiva = true;
          duracionCruceActual += TIEMPO_EXTENSION;
          Serial.println(F("[ALERTA] Presencia <= 10 cm: +5s de extension en Cruce A"));
        }
      }

      bool luzVerdePeatA = HIGH;
      if (estaParpadeandoVerde(1)) {
        if (tiempoActual - tUltParpVerde >= 250) {
          tUltParpVerde = tiempoActual;
          parpVerde = !parpVerde;
        }
        luzVerdePeatA = parpVerde;
      }
      actualizarLuces(HIGH, LOW, LOW, LOW, luzVerdePeatA, HIGH, LOW);

      if (transcurrido >= duracionCruceActual) {
        crucesRealizados++;
        cruceActivo = 0;
        extensionActiva = false;
        ultimaDistanciaSensor = -1.0;
        cambiarEstadoFSM(ESTADO_D7_DESPEJE_2);
      }
      break;
    }

    case ESTADO_D7_DESPEJE_2:
      // 5. Despeje Todo Rojo (2s): Veh Rojo, Pea A Rojo fijo, Pea B Rojo fijo
      cruceActivo = 0;
      actualizarLuces(HIGH, LOW, LOW, HIGH, LOW, HIGH, LOW);

      if (tiempoActual - tiempoInicioEstado >= T7_DESPEJE) {
        if (modoActual == MODO_EMERGENCIA || emergenciaPendiente) {
          emergenciaPendiente = false;
          cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
        } else if (modoActual == MODO_NOCTURNO) {
          cambiarEstadoFSM(ESTADO_D8_PREP_AUTOS);
        } else {
          cambiarEstadoFSM(ESTADO_D8_PREP_AUTOS);
        }
      }
      break;

    case ESTADO_D8_PREP_AUTOS:
      if (modoActual == MODO_EMERGENCIA) {
        cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
        break;
      }
      // 6. Preparación Autos (2s): Veh Amarillo solo, Pea A Rojo, Pea B Rojo
      cruceActivo = 0;
      actualizarLuces(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW);

      if (tiempoActual - tiempoInicioEstado >= T8_PREP_AUTOS) {
        if (modoActual == MODO_NOCTURNO) {
          cambiarEstadoFSM(ESTADO_NOCT_IDLE);
        } else {
          iniciarCruceVerticalD1();
        }
      }
      break;

    // -----------------------------------------------------------------
    // MODO NOCTURNO (AMARILLO INTERMITENTE CON DEMANDA PEATONAL DE 15s)
    // -----------------------------------------------------------------
    case ESTADO_NOCT_IDLE:
      if (modoActual == MODO_EMERGENCIA) {
        cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
        break;
      }
      if (modoActual == MODO_DIURNO) {
        iniciarCruceVerticalD1();
        break;
      }
      if (tiempoActual - tUltParpNoct >= 500) {
        tUltParpNoct = tiempoActual;
        parpNoct = !parpNoct;
      }
      actualizarLuces(LOW, parpNoct, LOW, HIGH, LOW, HIGH, LOW);
      if (pendienteA || pendienteB) {
        cruceActivo = seleccionarCrucePendiente();
        crucePreparado = cruceActivo;
        if (cruceActivo == 1) pendienteA = false;
        else if (cruceActivo == 2) pendienteB = false;
        cruceSolicitadoActual = true;
        cambiarEstadoFSM(ESTADO_NOCT_AMARILLO_IR);
      }
      break;

    case ESTADO_NOCT_AMARILLO_IR:
      if (modoActual == MODO_EMERGENCIA) {
        cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
        break;
      }
      actualizarLuces(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW);
      if (tiempoActual - tiempoInicioEstado >= TIEMPO_NOCT_AMARILLO) {
        cambiarEstadoFSM(ESTADO_NOCT_ROJO_IR);
      }
      break;

    case ESTADO_NOCT_ROJO_IR:
      if (modoActual == MODO_EMERGENCIA) {
        cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
        break;
      }
      actualizarLuces(HIGH, LOW, LOW, HIGH, LOW, HIGH, LOW);
      if (tiempoActual - tiempoInicioEstado >= TIEMPO_NOCT_DESPEJE) {
        medicionHecha = false;
        extensionActiva = false;
        duracionCruceActual = TIEMPO_NOCT_CRUCE; // 15s por solicitud
        cambiarEstadoFSM(ESTADO_NOCT_CRUCE_ACTIVO);
      }
      break;

    case ESTADO_NOCT_CRUCE_ACTIVO: {
      unsigned long transcurrido = tiempoActual - tiempoInicioEstado;
      if (!medicionHecha && transcurrido >= (TIEMPO_NOCT_CRUCE - 1000)) {
        medicionHecha = true;
        float dist = medirDistanciaCM(PIN_TRIG_SENSOR, PIN_ECHO_SENSOR);
        ultimaDistanciaSensor = dist;
        Serial.print(F("[HC-SR04] Distancia medida: "));
        Serial.print(dist, 1);
        Serial.println(F(" cm"));

        if (dist > 0 && dist <= UMBRAL_PRESENCIA_CM) {
          extensionesTiempo++;
          extensionActiva = true;
          duracionCruceActual += TIEMPO_EXTENSION;
          Serial.println(F("[ALERTA NOCT] Presencia <= 10 cm: +5s de extension (LED verde titilando)"));
        }
      }

      bool luzVerdeActiva = HIGH;
      if (estaParpadeandoVerde(cruceActivo)) {
        if (tiempoActual - tUltParpVerde >= 250) {
          tUltParpVerde = tiempoActual;
          parpVerde = !parpVerde;
        }
        luzVerdeActiva = parpVerde;
      }

      if (cruceActivo == 1) actualizarLuces(HIGH, LOW, LOW, LOW, luzVerdeActiva, HIGH, LOW);
      else if (cruceActivo == 2) actualizarLuces(HIGH, LOW, LOW, HIGH, LOW, LOW, luzVerdeActiva);

      if (transcurrido >= duracionCruceActual) {
        crucesRealizados++;
        extensionActiva = false;
        cambiarEstadoFSM(ESTADO_NOCT_ROJO_RETORNO);
      }
      break;
    }

    case ESTADO_NOCT_ROJO_RETORNO:
      actualizarLuces(HIGH, LOW, LOW, HIGH, LOW, HIGH, LOW);
      if (tiempoActual - tiempoInicioEstado >= TIEMPO_NOCT_DESPEJE) {
        cruceActivo = 0;
        ultimaDistanciaSensor = -1.0;
        if (modoActual == MODO_EMERGENCIA || emergenciaPendiente) {
          emergenciaPendiente = false;
          cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
        } else if (modoActual == MODO_DIURNO) {
          cambiarEstadoFSM(ESTADO_D8_PREP_AUTOS);
        } else {
          cambiarEstadoFSM(ESTADO_NOCT_IDLE);
        }
      }
      break;

    // -----------------------------------------------------------------
    // MODO EMERGENCIA
    // -----------------------------------------------------------------
    case ESTADO_EMERGENCIA_ACTIVA:
      cruceActivo = 0;
      extensionActiva = false;
      actualizarLuces(HIGH, LOW, LOW, HIGH, LOW, HIGH, LOW);

      if (modoActual != MODO_EMERGENCIA) {
        if (modoActual == MODO_DIURNO) {
          cambiarEstadoFSM(ESTADO_D8_PREP_AUTOS);
        } else {
          cambiarEstadoFSM(ESTADO_NOCT_IDLE);
        }
      }
      break;

    default:
      Serial.println(F("[HTTP 500 Internal Server Error] Estado FSM desconocido"));
      cambiarEstadoFSM(ESTADO_EMERGENCIA_ACTIVA);
      break;
  }
}
