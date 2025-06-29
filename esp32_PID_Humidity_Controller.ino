#define MQTT_MAX_PACKET_SIZE 512
#include <WiFi.h>
#include <WiFiClientSecure.h> // APENAS para MQTT TLS
#include <PubSubClient.h>
#include <HTTPClient.h>      // Para OpenWeatherMap
#include <Arduino_JSON.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <math.h>
#include "Adafruit_SHT4x.h"

// --- Configurações de Rede ---
const char* ssid = ; 
const char* password = ; 

// --- Configurações OpenWeatherMap API ---
String apiKeyOpenWeather = ; 
String cidadeOpenWeather = "Vila%20Real,PT";

// --- Configurações MQTT ---
const char* mqtt_server_address = ;
const int mqtt_port = 8883;
const char* mqtt_user = ; 
const char* mqtt_pass = ; 
const char* mqtt_topic_pub_data = ;

// --- Certificado CA Raiz para HiveMQ Cloud (ISRG Root X1) ---
const char* hivemq_ca_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n" \
(...)
"-----END CERTIFICATE-----\n";

// --- Pinos I2C (Sensor SHT41) ---
const int SHT_SDA_PIN = 5;
const int SHT_SCL_PIN = 6;

// --- Constantes do Sistema e Controlo PID ---
const float V_sala = 220.0f;
const float rho_ar = 1.2f;
const float Q_max_fan_vol = 0.122f; // Max "esvazamento" do ventilador simulado
float HR_setpoint = 50.0f; 
const unsigned long Ts_millis_control = 5000UL; // 5 segundos
const unsigned long Ts_millis_api = 3 * 60 * 1000UL; 

// --- Ganhos PID (AJUSTA ESTES VALORES PARA TUNING) ---
float Kp = 6.0f;  // Ganho Proporcional   
float Ki = 4.5f;  // Ganho Integral 
float Kd = 0.2f;  // Ganho Derivativo (começa com 0)

// --- Variáveis Globais ---
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
WiFiClientSecure clientSecureMQTT; 
PubSubClient mqttClient(clientSecureMQTT);
WiFiUDP ntpUDP;
const long utcOffsetInSeconds = 3600; 
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", utcOffsetInSeconds, 3600000);

unsigned long tempoAnteriorControlo = 0;
unsigned long tempoAnteriorAPI = 0;

float hr_medida_real, temp_medida_real; // Medidas reais do sensor SHT41
float erro_atual_pid, erro_anterior_pid = 0, erro_duas_amostras_atras_pid = 0;
float u_output_pid, u_output_pid_anterior = 0; // Saída do PID (0-100%)
float q_vent_simulada; // Vazão de ventilação simulada (m^3/s)

float ah_int_modelo_estado_anterior; // Humidade absoluta interna do modelo (kg_agua/kg_ar_seco)
float hr_int_modelo_prevista;        // Humidade relativa interna prevista pelo modelo (%) - ESTA É A VARIÁVEL CONTROLADA NA SIMULAÇÃO
float hr_simulada_para_feedback_pid; // Humidade simulada do ciclo anterior para cálculo do erro

// Variáveis para condições externas (API ou fixas)
float temp_externa_api = 20.0f;
float hr_externa_api = 50.0f;
float ah_externa_api_calculada;

// --- Controlo da Simulação ---
bool usar_temp_fixa_modelo = true; 
const float temp_fixa_para_modelo = 22.0f; // °C - Temperatura interna constante para o modelo
bool usar_cond_externas_fixas = true; 
const float hr_externa_fixa = 60.0f;    // % - Humidade externa fixa (MAIS ALTA QUE A INTERNA INICIAL)
const float temp_externa_fixa = 20.0f;  // °C - Temperatura externa fixa

// --- Funções Psicrométricas ---
float calcularPsat(float T_c) {
    double T_k = T_c + 273.15;
    const double C8 = -5.8002206E+03; const double C9 = 1.3914993E+00;
    const double C10 = -4.8640239E-02; const double C11 = 4.1764768E-05;
    const double C12 = -1.4452093E-08; const double C13 = 6.5459673E+00;
    double ln_P_sat = C8/T_k + C9 + C10*T_k + C11*pow(T_k,2) + C12*pow(T_k,3) + C13*log(T_k);
    return exp(ln_P_sat);
}
float hrToAH(float hr, float temp_c) {
    hr = constrain(hr, 0.0f, 100.0f);
    float p_sat = calcularPsat(temp_c);
    float p_v = (hr / 100.0f) * p_sat;
    float p_atm = 101325.0f;
    if (p_atm - p_v <= 1e-3) { return (0.62198f * p_sat) / (p_atm - p_sat + 1e-3f); } // Adicionado pequeno valor para evitar divisão por zero
    return (0.62198f * p_v) / (p_atm - p_v);
}
float ahToHR(float ah, float temp_c) {
    if (ah < 0) ah = 0; // Humidade absoluta não pode ser negativa
    float p_atm = 101325.0f;
    float p_v = (ah * p_atm) / (0.62198f + ah);
    float p_sat = calcularPsat(temp_c);
    if (p_sat <= 1e-6) { return 50.0; } // Evitar divisão por zero, retorna um valor médio se p_sat for muito baixo
    float hr_calc = (p_v / p_sat) * 100.0f;
    return constrain(hr_calc, 0.0f, 100.0f);
}

// --- Funções de Rede e Setup ---
void setup_wifi_e_ntp() {
    delay(10); Serial.println();
    Serial.print("A conectar a "); Serial.println(ssid);
    WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
    int wifi_tries = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_tries < 30) {
        delay(500); Serial.print("."); wifi_tries++;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFalha ao conectar ao WiFi. A reiniciar..."); delay(5000); ESP.restart();
    }
    Serial.println("\nWiFi conectado!"); Serial.print("Endereço IP: "); Serial.println(WiFi.localIP());
    Serial.println("A inicializar cliente NTP...");
    timeClient.begin();
    Serial.print("A aguardar sincronização NTP: ");
    int ntp_tries = 0;
    while(!timeClient.update() && ntp_tries < 10) {
        timeClient.forceUpdate(); Serial.print("."); delay(1000); ntp_tries++;
    }
    if(ntp_tries < 10){
        Serial.println("\nHora NTP sincronizada:"); Serial.println(timeClient.getFormattedTime());
    } else {
        Serial.println("\nFalha ao sincronizar hora NTP. A ligação TLS MQTT pode falhar.");
    }
}

// Função para buscar dados da OpenWeatherMap
void fetchOpenWeatherMapData() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String serverPath = "http://api.openweathermap.org/data/2.5/weather?q=" + cidadeOpenWeather +
                            "&appid=" + apiKeyOpenWeather + "&units=metric"; // HTTP URL

        Serial.print("A aceder a OpenWeatherMap (HTTP): "); Serial.println(serverPath);
        
        // http.begin() para HTTPClient normal não precisa do objeto clientSecureAPI
        http.begin(serverPath.c_str()); 
        int httpResponseCode = http.GET();

        if (httpResponseCode > 0) {
            Serial.print("Código de Resposta HTTP (OpenWeatherMap): "); Serial.println(httpResponseCode);
            if (httpResponseCode == HTTP_CODE_OK) {
                String payload = http.getString();
                JSONVar parsedJson = JSON.parse(payload);
                if (JSON.typeof(parsedJson) == "undefined") {
                    Serial.println("Falha ao processar JSON da API OpenWeatherMap.");
                } else {
                    if (parsedJson.hasOwnProperty("main") && parsedJson["main"].hasOwnProperty("temp")) {
                        temp_externa_api = double(parsedJson["main"]["temp"]);
                    }
                    if (parsedJson.hasOwnProperty("main") && parsedJson["main"].hasOwnProperty("humidity")) {
                        hr_externa_api = double(parsedJson["main"]["humidity"]);
                    }
                    ah_externa_api_calculada = hrToAH(hr_externa_api, temp_externa_api);
                    Serial.print("API Data -> TempExt: "); Serial.print(temp_externa_api);
                    Serial.print("C, HRExt: "); Serial.print(hr_externa_api);
                    Serial.print("%, AHExt_calc: "); Serial.println(ah_externa_api_calculada, 6);
                }
            }
        } else {
            Serial.print("Erro na chamada OpenWeatherMap API. Código HTTP: "); Serial.println(httpResponseCode);
        }
        http.end();
    } else {
        Serial.println("WiFi desconectado. Não é possível obter dados da API.");
    }
}

void reconnectMQTT() {
    int retries = 0;
    while (!mqttClient.connected() && retries < 5) {
        Serial.print("A tentar conexão MQTT (TLS)... ");
        String clientId = "ESP32_PID_Client-"; clientId += String(random(0xffff), HEX);
        if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
            Serial.println("Conectado ao HiveMQ!");
        } else {
            Serial.print("Falha, rc="); Serial.print(mqttClient.state());
            Serial.println(" Tentando novamente em 5 segundos.");
            delay(5000);
        }
        retries++;
    }
    if (!mqttClient.connected() && retries >=5 ){
      Serial.println("Muitas falhas ao conectar ao MQTT. Reiniciando...");
      delay(1000); ESP.restart();
    }
}

void publishMQTTData() {
    if (!mqttClient.connected()) {
        reconnectMQTT();
        if (!mqttClient.connected()) {
             Serial.println("MQTT não conectado. Não é possível publicar."); return;
        }
    }
    timeClient.update();

    JSONVar jsonData;
    jsonData["timestamp_device_epoch"] = timeClient.getEpochTime(); // Epoch time
    jsonData["timestamp_device_str"] = timeClient.getFormattedTime(); // String formatada
    
    jsonData["hr_real_sht41"] = round(hr_medida_real * 100.0) / 100.0; 
    jsonData["temp_real_sht41"] = round(temp_medida_real * 100.0) / 100.0; 
    
    jsonData["hr_setpoint_pid"] = HR_setpoint;
    jsonData["erro_pid"] = round(erro_atual_pid * 100.0) / 100.0; 
    jsonData["output_pid_percent"] = round(u_output_pid * 100.0) / 100.0;
    jsonData["q_vent_sim_m3s"] = round(q_vent_simulada * 100000.0) / 100000.0;
    
    jsonData["hr_modelo_prevista_percent"] = round(hr_int_modelo_prevista * 100.0) / 100.0; 
    
    jsonData["temp_externa_api"] = round(temp_externa_api * 100.0) / 100.0;
    jsonData["hr_externa_api"] = round(hr_externa_api * 100.0) / 100.0;
    jsonData["ah_externa_api_calc"] = round(ah_externa_api_calculada * 1000000.0) / 1000000.0;
    jsonData["ah_modelo_estado_anterior"] = round(ah_int_modelo_estado_anterior * 1000000.0) / 1000000.0;

    String jsonString = JSON.stringify(jsonData);
    Serial.print("A publicar no MQTT ("); Serial.print(mqtt_topic_pub_data); Serial.print("): ");
    Serial.println(jsonString);
    if (mqttClient.publish(mqtt_topic_pub_data, jsonString.c_str())) {
        Serial.println("Mensagem MQTT publicada com sucesso.");
    } else {
        Serial.println("Falha ao publicar mensagem MQTT.");
    }
}

void setup() {
    Serial.begin(115200); delay(1000);
    setup_wifi_e_ntp();
    clientSecureMQTT.setCACert(hivemq_ca_cert);
    mqttClient.setServer(mqtt_server_address, mqtt_port);
    mqttClient.setBufferSize(1024);

    Serial.println("Inicializando sensor SHT4x...");
    Wire.begin(SHT_SDA_PIN, SHT_SCL_PIN);
    if (!sht4.begin(&Wire)) {
        Serial.println("FATAL: Sensor SHT4x não encontrado!"); while (1) delay(1000);
    }
    sht4.setPrecision(SHT4X_HIGH_PRECISION); sht4.setHeater(SHT4X_NO_HEATER);
    Serial.println("Sensor SHT4x inicializado.");

    // Leitura inicial do SHT41 (para hr_medida_real e temp_medida_real que serão publicadas)
    sensors_event_t sht_humidity_event, sht_temp_event; «
    if (sht4.getEvent(&sht_humidity_event, &sht_temp_event) && !isnan(sht_humidity_event.relative_humidity) && !isnan(sht_temp_event.temperature)) {
        hr_medida_real = sht_humidity_event.relative_humidity;
        temp_medida_real = sht_temp_event.temperature;
    } else {
        Serial.println("Falha na leitura inicial SHT4x, usando defaults para medidas reais.");
        hr_medida_real = 60.0f; 
        temp_medida_real = 22.0f; 
    }

    // Configurar condições externas (fixas ou da API)
    if (usar_cond_externas_fixas) {
        Serial.println("Usando condições externas FIXAS para simulação.");
        temp_externa_api = temp_externa_fixa;
        hr_externa_api = hr_externa_fixa;
        ah_externa_api_calculada = hrToAH(hr_externa_fixa, temp_externa_fixa);
    } else {
        Serial.println("A obter dados iniciais da API OpenWeatherMap...");
        fetchOpenWeatherMapData(); 
    }
    
    // Determinar a temperatura a ser usada pelo modelo
    float temp_para_modelo_no_setup;
    if (usar_temp_fixa_modelo) {
        Serial.println("Usando temperatura FIXA para o modelo de simulação.");
        temp_para_modelo_no_setup = temp_fixa_para_modelo;
    } else {
        temp_para_modelo_no_setup = temp_medida_real; 
    }

    // --- Inicialização para a Simulação em Malha Fechada ---
    float hr_inicial_simulada = 45.0f; 
    // HR_setpoint já está definido globalmente

    ah_int_modelo_estado_anterior = hrToAH(hr_inicial_simulada, temp_para_modelo_no_setup);
    hr_int_modelo_prevista = hr_inicial_simulada; 
    hr_simulada_para_feedback_pid = hr_int_modelo_prevista;

    erro_anterior_pid = HR_setpoint - hr_simulada_para_feedback_pid;
    erro_duas_amostras_atras_pid = erro_anterior_pid; 
    u_output_pid_anterior = 0.0f;  // Ventilador simulado começa desligado
    u_output_pid = u_output_pid_anterior;
    q_vent_simulada = (u_output_pid / 100.0f) * Q_max_fan_vol;
    
    Serial.println("--- INÍCIO DA SIMULAÇÃO EM MALHA FECHADA ---");
    Serial.print("Setpoint HR: "); Serial.println(HR_setpoint);
    Serial.print("HR Inicial Simulada: "); Serial.println(hr_inicial_simulada);
    Serial.print("Erro PID Inicial (simulado): "); Serial.println(erro_anterior_pid, 2);
    Serial.print("Temp usada no modelo: "); Serial.println(temp_para_modelo_no_setup);
    Serial.print("AH Externa (Fixa/API): "); Serial.println(ah_externa_api_calculada, 6);
    Serial.print("AH Interna Inicial Modelo: "); Serial.println(ah_int_modelo_estado_anterior, 6);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi desconectado! Tentando reconectar...");
        setup_wifi_e_ntp();
    }
    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    mqttClient.loop();

    unsigned long currentTime = millis();

    // --- Loop de Controlo PID e Modelo ---
    if (currentTime - tempoAnteriorControlo >= Ts_millis_control) {
        tempoAnteriorControlo = currentTime;

        // Leitura de Sensores (para hr_medida_real e, opcionalmente, temp_medida_real se o modelo não usar valor fixo)
        sensors_event_t sht_humidity_event, sht_temp_event; // Renomeado
        if (!sht4.getEvent(&sht_humidity_event, &sht_temp_event) || isnan(sht_humidity_event.relative_humidity) || isnan(sht_temp_event.temperature)) {
            Serial.println("AVISO: Falha/Leitura inválida (NaN) do SHT4x! Usando últimos valores válidos para dados reais.");
        } else {
            hr_medida_real = sht_humidity_event.relative_humidity;
            if (!usar_temp_fixa_modelo) { // Só atualiza temp_medida_real se o modelo a usar
                temp_medida_real = sht_temp_event.temperature; 
            } else { // Se modelo usa temp fixa, ainda podemos querer ler a temp real para MQTT
                 temp_medida_real = sht_temp_event.temperature;
            }
        }
        
        float temp_atual_para_o_modelo = usar_temp_fixa_modelo ? temp_fixa_para_modelo : temp_medida_real;
        
        // Atualizar condições externas se não forem fixas (e já passou o tempo de Ts_millis_api)
        if (!usar_cond_externas_fixas && (currentTime - tempoAnteriorAPI >= Ts_millis_api || tempoAnteriorAPI == 0) ) {
            tempoAnteriorAPI = currentTime; 
            Serial.println("\nAtualizando dados da API OpenWeatherMap...");
            fetchOpenWeatherMapData();
        }

        Serial.println("---------------------------------------------");
        Serial.print(currentTime / 1000); Serial.print("s ");
        Serial.print("|| SENSOR -> T_real: "); Serial.print(temp_medida_real, 1);
        Serial.print("C, HR_real: "); Serial.print(hr_medida_real, 1); Serial.println("%");
        Serial.print("   MODELO USANDO -> Temp_modelo: "); Serial.print(temp_atual_para_o_modelo, 1);
        Serial.print("C, AH_ext (fixa/API): "); Serial.print(ah_externa_api_calculada, 6);
        Serial.print(", HR_sim_anterior: "); Serial.println(hr_simulada_para_feedback_pid, 1);


        erro_atual_pid = HR_setpoint - hr_simulada_para_feedback_pid;

        float Ts_seg = Ts_millis_control / 1000.0f;
        float p_term = Kp * (erro_atual_pid - erro_anterior_pid);
        float i_term_calculado = Ki * Ts_seg * erro_atual_pid;
        
        // Lógica Anti-Windup ajustada para ação direta U = Uant + deltaU
        // Se output está no máximo (100) e PID ainda quer aumentar (delta_u_raw > 0), limita o termo integral positivo.
        if (u_output_pid_anterior >= 100.0f && (p_term + i_term_calculado > 0) ) { // Considera apenas P e I para o sinal de delta_u
            if (i_term_calculado > 0) i_term_calculado = 0; // Zera I positivo para não acumular mais para cima
        } 
        // Se output está no mínimo (0) e PID ainda quer diminuir (delta_u_raw < 0), limita o termo integral negativo.
        else if (u_output_pid_anterior <= 0.0f && (p_term + i_term_calculado < 0) ) {
             if (i_term_calculado < 0) i_term_calculado = 0; // Zera I negativo para não acumular mais para baixo
        }

        float d_term = (Kd / Ts_seg) * (erro_atual_pid - 2.0f * erro_anterior_pid + erro_duas_amostras_atras_pid);
        float delta_u_raw = p_term + i_term_calculado + d_term;

        u_output_pid = u_output_pid_anterior + delta_u_raw; // AÇÃO DIRETA: Erro positivo aumenta output (mais ventilação)
        u_output_pid = constrain(u_output_pid, 0.0f, 100.0f);
        
        q_vent_simulada = (u_output_pid / 100.0f) * Q_max_fan_vol;

        Serial.print("   PID    -> Erro (sim): "); Serial.print(erro_atual_pid, 2);
        Serial.print(" [P: "); Serial.print(p_term, 2);
        Serial.print(", I_calc: "); Serial.print(i_term_calculado, 2);
        Serial.print(", D: "); Serial.print(d_term, 2);
        Serial.print("] delta_u_raw: "); Serial.print(delta_u_raw, 2);
        Serial.print(" -> u_out: "); Serial.print(u_output_pid, 2);
        Serial.print("%, Q_vent_sim: "); Serial.println(q_vent_simulada, 5);
        
        float G_int_mass_modelo = -0.000010f; // Geração interna de humidade (pode ser ajustada)
        
        // Equação do modelo ajustada: ventilação adiciona (AH_ext - AH_int)
        float delta_ah_modelo = (Ts_seg / (V_sala * rho_ar)) *
                                  (G_int_mass_modelo + (q_vent_simulada * rho_ar) * (ah_externa_api_calculada - ah_int_modelo_estado_anterior));
        
        float ah_int_modelo_proximo_calc = ah_int_modelo_estado_anterior + delta_ah_modelo;
        ah_int_modelo_proximo_calc = max(0.0f, ah_int_modelo_proximo_calc);
        hr_int_modelo_prevista = ahToHR(ah_int_modelo_proximo_calc, temp_atual_para_o_modelo);

        Serial.print("   MODELO -> AH_ant_int: "); Serial.print(ah_int_modelo_estado_anterior, 6);
        Serial.print(", delta_AH: "); Serial.print(delta_ah_modelo, 7); 
        Serial.print(", AH_prox_calc: "); Serial.print(ah_int_modelo_proximo_calc, 6);
        Serial.print(", HR_modelo_prevista: "); Serial.print(hr_int_modelo_prevista, 1); Serial.println("%");

        hr_simulada_para_feedback_pid = hr_int_modelo_prevista;
        
        erro_duas_amostras_atras_pid = erro_anterior_pid;
        erro_anterior_pid = erro_atual_pid;
        u_output_pid_anterior = u_output_pid;
        ah_int_modelo_estado_anterior = ah_int_modelo_proximo_calc;
        
        publishMQTTData();
    }
}