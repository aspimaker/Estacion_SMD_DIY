#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <EEPROM.h>
#include <Bounce2.h>

// Configuración OLED
#define OLED_ANCHO 128
#define OLED_ALTO 64
Adafruit_SSD1306 oled(OLED_ANCHO, OLED_ALTO, &Wire, -1);

// Pines
#define SSR_PIN  8
#define NTC_PIN A0
#define ENCODER_CLK_PIN 3
#define ENCODER_DT_PIN 2
#define ENCODER_SW_PIN 4
#define LED_R_PIN  5
#define LED_G_PIN  9
#define LED_B_PIN 10
#define ZC_PIN 11

// Constantes de control
#define HISTERESIS_ON 5    // °C diferencia para encender
#define HISTERESIS_OFF 2   // °C diferencia para apagar
#define TEMP_MAX 300       // Límite térmico absoluto
#define TIMEOUT_MIN 30     // Apagado automático (minutos)
#define DEBOUNCE_TIME 25   // ms para debounce

// Configuración NTC
#define ResistenciaSerieNTC 100000.0
#define NUM_LECTURAS 10
#define BETA 3950.0        // Coeficiente beta del NTC
#define TEMP_NOMINAL 25    // Temperatura nominal (para cálculo NTC)
#define RESISTENCIA_NOMINAL 100000 // Resistencia a temp nominal

// EEPROM
#define EEPROM_ADDR_SETPOINT 0
#define EEPROM_ADDR_PID 10

// Instancias Bounce2
Bounce debouncerEncoderCLK = Bounce();
Bounce debouncerEncoderSW = Bounce();

struct PIDParams {
    double Kp, Ki, Kd;
    byte checksum;
};

// Variables de estado
int setpoint = 150;
bool calentando = false;
unsigned long ultimaActualizacion = 0;
volatile bool crucePorCeroDetectado = false;

// Variables PID
double temperaturaActual = 0, salidaPID = 0, temperaturaObjetivo = 150;
PID controladorPID(&temperaturaActual, &salidaPID, &temperaturaObjetivo, 30, 5, 50, DIRECT);

// Auto-tuning
PID_ATune autotune(&temperaturaActual, &salidaPID);
bool autoTuning = false;
unsigned long tiempoAutoTune = 0;


void guardarPID() {
    PIDParams params = {controladorPID.GetKp(), controladorPID.GetKi(), controladorPID.GetKd(), 0};
    params.checksum = (byte)(params.Kp + params.Ki + params.Kd);
    EEPROM.put(EEPROM_ADDR_PID, params);
}

PIDParams cargarValidarPIDParams() {
    PIDParams params;
    EEPROM.get(EEPROM_ADDR_PID, params);
    
    bool eepromCorrupta = false;
    
    if (isnan(params.Kp) || isnan(params.Ki) || isnan(params.Kd)) {
        eepromCorrupta = true;
    }
    
    if (params.Kp < 1 || params.Kp > 100 ||
        params.Ki < 0 || params.Ki > 10 ||
        params.Kd < 0 || params.Kd > 100) {
        eepromCorrupta = true;
    }
    
    byte checksumCalculado = (byte)(params.Kp + params.Ki + params.Kd);
    if (params.checksum != checksumCalculado) {
        eepromCorrupta = true;
    }
    
    if (eepromCorrupta) {
        params = {15.0, 0.5, 10.0, (byte)(15.0 + 0.5 + 10.0)};
        EEPROM.put(EEPROM_ADDR_PID, params);
    }
    
    return params;
}

void tiraLED_ROJO() { analogWrite(LED_R_PIN, 255); analogWrite(LED_G_PIN, 0); analogWrite(LED_B_PIN, 0); }
void tiraLED_VERDE() { analogWrite(LED_R_PIN, 0); analogWrite(LED_G_PIN, 255); analogWrite(LED_B_PIN, 0); }
void tiraLED_AZUL() { analogWrite(LED_R_PIN, 0); analogWrite(LED_G_PIN, 0); analogWrite(LED_B_PIN, 255); }
void tiraLED_AMARILLO() { analogWrite(LED_R_PIN, 255); analogWrite(LED_G_PIN, 255); analogWrite(LED_B_PIN, 0); }
void tiraLED_BLANCO() { analogWrite(LED_R_PIN, 255); analogWrite(LED_G_PIN, 255); analogWrite(LED_B_PIN, 255); }

void emergenciaTermica(const char* motivo) {
    digitalWrite(SSR_PIN, LOW);
    tiraLED_ROJO();
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println(motivo);
    oled.print(F("Temp: ")); oled.print(temperaturaActual); oled.println(F("°C"));
    oled.println(F("Reiniciar equipo"));
    oled.display();
    
    while(1) {
        digitalWrite(LED_R_PIN, !digitalRead(LED_R_PIN));
        delay(500);
    }
}

void mostrarError(const char* mensaje, bool bloqueante) {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println(mensaje);
    oled.print(F("Temp: ")); oled.print(temperaturaActual); oled.println(F("°C"));
    oled.display();
    
    if (bloqueante) {
        while(1) {
            tiraLED_ROJO();
            delay(500);
            tiraLED_BLANCO();
            delay(500);
        }
    }
}

void iniciarAutoTuning() {
    if(temperaturaActual < 100) {
        mostrarError("Temp <100°C", false);
        return;
    }
    autoTuning = true;
    tiempoAutoTune = millis();

    autotune.Cancel();
    autotune.SetNoiseBand(2.0);
    autotune.SetOutputStep(100);
    autotune.SetLookbackSec(10);

    tiraLED_AMARILLO();
}

void finalizarAutoTuning() {
    autoTuning = false;
    controladorPID.SetTunings(autotune.GetKp(), autotune.GetKi(), autotune.GetKd());
    guardarPID();

    tiraLED_ROJO();
}

void abortarAutoTuning() {
    autoTuning = false;
    emergenciaTermica("FALLO AUTO-TUNE");
}

void ejecutarControlSSR() {
  static unsigned long ultimoCiclo = 0;
  unsigned long ahora = millis();

  if (ahora - ultimoCiclo < 10) return; // Protección contra rebotes
  ultimoCiclo = ahora;
  
  if (salidaPID > 10 && calentando && !autoTuning) {
      unsigned int tiempoON = constrain(map(salidaPID, 0, 255, 1000, 8000), 1000, 8000);
      digitalWrite(SSR_PIN, HIGH);
      delayMicroseconds(tiempoON);
      digitalWrite(SSR_PIN, LOW);
  }

  crucePorCeroDetectado = false;
}

float leerTemperaturaNTC(int pin) {
    int adc = analogRead(pin);
    if (adc < 1 || adc > 1023) return NAN;

    float resistencia = ResistenciaSerieNTC * ((1023.0 / adc) - 1.0);
    float tempK = 1.0 / (1.0/298.15 + (1.0/BETA) * log(resistencia/RESISTENCIA_NOMINAL));
    return tempK - 273.15;
}

float leerTemperaturaFiltrada() {
    float suma = 0;
    int lecturasValidas = 0;

    for (int i = 0; i < NUM_LECTURAS; i++) {
        float t = leerTemperaturaNTC(NTC_PIN);
        if (!isnan(t) && t > -50 && t < TEMP_MAX) {
            suma += t;
            lecturasValidas++;
        }
        delay(5);
    }
    
    if (lecturasValidas == 0) return NAN;
    return suma / lecturasValidas;
}

bool validarTemperatura(float temp, bool bloqueante) {
    if (isnan(temp)) {
        mostrarError("Error: Sensor NTC", bloqueante);
        return false;
    }
    if (temp < 10 || temp > TEMP_MAX) {
        mostrarError("Ajuste NTC!", bloqueante);
        return false;
    }
    return true;
}

void verificarEEPROM() {
    static unsigned long ultimaVerificacion = 0;
    const unsigned long intervalo = 3600000; // 1 hora
    
    if (millis() - ultimaVerificacion > intervalo) {
        PIDParams params;
        EEPROM.get(EEPROM_ADDR_PID, params);
        byte checksum = (byte)(params.Kp + params.Ki + params.Kd);

        if (checksum != params.checksum) {
            emergenciaTermica("ERROR EEPROM");
        }
        ultimaVerificacion = millis();
    }
}

void controlTemperatura() {
    static unsigned long ultimoSobretemperatura = 0;

    if (isnan(temperaturaActual)) {
        emergenciaTermica("FALLO SENSOR");
    }
    else if (temperaturaActual < -50) {
        emergenciaTermica("TEMPERATURA BAJA");
    }
    else if (temperaturaActual >= TEMP_MAX) {
        if(millis() - ultimoSobretemperatura > 5000) {
            emergenciaTermica("SOBRETEMPERATURA");
        }
        ultimoSobretemperatura = millis();
    }
}

void actualizarLEDs() {
    if (autoTuning) {
        tiraLED_AMARILLO();
    } else if (calentando) {
        tiraLED_ROJO();
    } else {
        tiraLED_AZUL();
    }
}

void actualizarEncoder() {
    static int ultimoEstadoDT = HIGH;
    static unsigned long ultimoCambio = 0;
    
    debouncerEncoderCLK.update();
    debouncerEncoderSW.update();
    
    int estadoCLK = debouncerEncoderCLK.read();
    int estadoDT = digitalRead(ENCODER_DT_PIN);
    
    if (estadoCLK != ultimoEstadoDT) {
        if (millis() - ultimoCambio > 50) {
            int cambio = (estadoDT == estadoCLK) ? -5 : 5;
            setpoint = constrain(setpoint + cambio, 25, 250);
            
            if (setpoint != temperaturaObjetivo) {
                temperaturaObjetivo = setpoint;
                EEPROM.update(EEPROM_ADDR_SETPOINT, setpoint);
            }
            ultimoCambio = millis();
        }
    }
    ultimoEstadoDT = estadoCLK;

    if (debouncerEncoderSW.fell()) {
        calentando = !calentando;
        tiraLED_BLANCO();
        delay(100);
        actualizarLEDs();
    }
    
    if (debouncerEncoderSW.duration() > 3000 && !autoTuning && temperaturaActual > 100) {
        iniciarAutoTuning();
    }
}

void manejarControlTemperatura() {
    static unsigned long ultimoControl = 0;
    const unsigned long intervaloControl = 100;
    
    float diferencia = temperaturaObjetivo - temperaturaActual;
    
    if (!autoTuning) {
        if (diferencia > HISTERESIS_ON) {
            calentando = true;
        } else if (diferencia < -HISTERESIS_OFF) {
            calentando = false;
        }
    }

    if (millis() - ultimoControl >= intervaloControl) {
        if (autoTuning) {
            if (millis() - tiempoAutoTune > 300000) { // 5 minutos máximo
                abortarAutoTuning();
            }
            if (autotune.Runtime() != 0) {
                finalizarAutoTuning();
            }
        } else {
            controladorPID.Compute();
        }
        ultimoControl = millis();
    }
    
    if (crucePorCeroDetectado && calentando && !autoTuning) {
        ejecutarControlSSR();
    }
}

void dibujarGrafica(int tempActual, int tempObjetivo) {
    static int valores[64];
    static int index = 0;
    static int lastYSetpoint = -1;
    
    int ySetpoint = map(tempObjetivo, 0, TEMP_MAX, OLED_ALTO-1, 15);
    
    if(ySetpoint != lastYSetpoint) {
        oled.drawLine(0, lastYSetpoint, OLED_ANCHO-1, lastYSetpoint, SSD1306_BLACK);
        oled.drawLine(0, ySetpoint, OLED_ANCHO-1, ySetpoint, SSD1306_WHITE);
        lastYSetpoint = ySetpoint;
    }
    
    oled.drawLine(index*2, OLED_ALTO-1, index*2, 15, SSD1306_BLACK);
    valores[index] = map(tempActual, 0, TEMP_MAX, OLED_ALTO-1, 15);
    oled.drawLine(index*2, OLED_ALTO-1, index*2, valores[index], SSD1306_WHITE);
    
    index = (index + 1) % 64;
}

void actualizarPantalla() {
    static unsigned long ultimoRefresh = 0;
    if (millis() - ultimoRefresh < 500) return;
    ultimoRefresh = millis();

    oled.clearDisplay();
    
    // Header
    oled.setCursor(0, 0);
    oled.print(F("ESTACION SMD "));
    oled.print(autoTuning ? F("[AT]") : calentando ? F("[ON]") : F("[OFF]"));
    
    // Temperaturas
    oled.setCursor(0, 15);
    oled.print(F("Actual: ")); 
    oled.print(temperaturaActual, 1); 
    oled.print(F("°C"));
    
    oled.setCursor(0, 25);
    oled.print(F("Objetivo: ")); 
    oled.print(setpoint); 
    oled.print(F("°C"));

    // PID
    oled.setCursor(0, 35);
    oled.print(F("PID: "));
    oled.print(controladorPID.GetKp(), 1); oled.print(F(","));
    oled.print(controladorPID.GetKi(), 1); oled.print(F(","));
    oled.print(controladorPID.GetKd(), 1);

    // Gráfico
    dibujarGrafica(temperaturaActual, setpoint);
    
    // Footer
    oled.setCursor(0, 55);
    oled.print(F("Gira+Hold: AT"));
    oled.display();
}

  void detectarCrucePorZC() {
    static unsigned long ultimaInterrupcion = 0;
    unsigned long ahora = millis();
    
    // Filtro debounce para interrupciones
    if (ahora - ultimaInterrupcion > 2) {
        crucePorCeroDetectado = true;
        ultimaInterrupcion = ahora;
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        while(1); // Fallo crítico
    }

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println(F("     ESTACION DE"));
    oled.println(F("      SOLDADURA"));
    oled.println(F(" "));
    oled.println(F("   aspimaker 06/2025"));
    oled.display();
    delay(2000);

    pinMode(SSR_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW);  // por seguridad apagamos el SSR

    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    pinMode(ZC_PIN, INPUT_PULLUP);

    debouncerEncoderCLK.attach(ENCODER_CLK_PIN, INPUT_PULLUP);
    debouncerEncoderCLK.interval(DEBOUNCE_TIME);

    debouncerEncoderSW.attach(ENCODER_SW_PIN, INPUT_PULLUP);
    debouncerEncoderSW.interval(DEBOUNCE_TIME);

    attachInterrupt(digitalPinToInterrupt(ZC_PIN), detectarCrucePorZC, FALLING);

    PIDParams params = cargarValidarPIDParams();
    controladorPID.SetTunings(params.Kp, params.Ki, params.Kd);
    
    byte valorEEPROM = EEPROM.read(EEPROM_ADDR_SETPOINT);
    if (valorEEPROM >= 25 && valorEEPROM <= 250) {
        setpoint = valorEEPROM;
        temperaturaObjetivo = setpoint;
    }

    controladorPID.SetMode(AUTOMATIC);
    controladorPID.SetSampleTime(50);
    controladorPID.SetOutputLimits(0, 255);

    float temp = leerTemperaturaFiltrada();
    if (!validarTemperatura(temp, true)) {
        emergenciaTermica("FALLO SENSOR");
    }
}

void loop() {

    static unsigned long ultimoLoop = millis();
    static unsigned long ultimoControl = millis();
    
    // Verificación del bucle principal
    if (millis() - ultimoLoop > 5000) {
        emergenciaTermica("FALLO_LOOP");
    }
    ultimoLoop = millis();

    // Lectura de temperatura
    temperaturaActual = leerTemperaturaFiltrada();
    if (!validarTemperatura(temperaturaActual, false)) {
        return;
    }

    // Control principal
    verificarEEPROM();

    controlTemperatura();

    actualizarEncoder();

    manejarControlTemperatura();

    actualizarPantalla();

    actualizarLEDs();
}

