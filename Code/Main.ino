#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <ESP32Servo.h>

Servo myservo;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Configuração dos pinos I2C
#define SDA_PIN 21  // Pino SDA (D21)
#define SCL_PIN 23  // Pino SCL (D23)

float dist; // Distância em centímetros
float kp = 1.8, ki = 0.2801, kd = 18.0;
float erro = 0, erro_anterior = 0;
float setpoint;
float saidaPID = 0;
float PID_i = 0;
unsigned long tempoAnterior = 0;
unsigned long intervaloControle = 80; // Intervalo ajustado para 80 ms

// Filtro de média móvel para definir set-point
const int numLeituras = 10; // Número de leituras para a média
int leituras[numLeituras];  // Array para armazenar as leituras
int indice = 0;             // Índice atual do array
int total = 0;              // Soma das leituras

const int pinoAnalogico = 34; // Pino GPIO34 (pino analógico no ESP32)

void setup() {
  Serial.begin(115200);
  while (!Serial) {delay(1);}

  // Inicializa a comunicação I2C com os pinos customizados
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!lox.begin()) {
    Serial.println(F("Falha ao inicializar o sensor"));
    while(1);
  }
  
  myservo.attach(13);
}

void loop() {
  unsigned long tempoAtual = millis();
  if (tempoAtual - tempoAnterior >= intervaloControle) {
    tempoAnterior = tempoAtual;

    if (dist != -1) { // Se a leitura for válida executa PID
      // Cálculo do PID

     dist = get_dist(5); // 5 leituras por ciclo

     setpoint = get_setpoint();

     erro = setpoint - dist;
      
      if (-5 < erro && erro < 5) {
        PID_i += ki * erro;
      } else {
        PID_i = 0;
      }
      
      float derivadaErro = erro - erro_anterior;

      saidaPID = (kp * erro) + PID_i + (kd * derivadaErro);
      
      saidaPID = constrain(saidaPID, -45, 45); // Limita a saída
      myservo.write(127 + saidaPID);  //Âgulo da barra na horizontal + saída do PID

      erro_anterior = erro;  //Atualiza o erro

//---------Impressão de valores para análise em gráfico no Serial Plotter------
Serial.print(0);
Serial.print(","); 
Serial.print(dist);
Serial.print(","); 
Serial.print(setpoint);
Serial.print(","); 
Serial.println(40);
delay(5);   
    
    }
  
  }
}





//----------------Função para obter a distância do sensor---------------------
float get_dist(int n) {
  long soma = 0;
  int validas = 0;
  
  for (int i = 0; i < n; i++) {
    VL53L0X_RangingMeasurementData_t medida;
    lox.rangingTest(&medida, false);
    
    if (medida.RangeStatus != 4) { // Verifica leitura válida
      soma += medida.RangeMilliMeter;
      validas++;
    
    delay(5); // Pequeno delay para estabilidade
    float distancia_cm = (soma / validas) / 10.0; // Converte para cm
    //Serial.print("Distancia (cm): ");
    //Serial.println(distancia_cm);
    return distancia_cm;
    }
   
    else {
    Serial.println("Leitura invalida");
    return -1; // Retorna -1 se todas as leituras forem inválidas
  }
}
}




//--------------Função para obter o set-point a partir da tensão lida no potenciômetro-----------
float get_setpoint() {
  // Subtrai a leitura mais antiga do total
  total -= leituras[indice];
  
  // Lê o valor analógico e armazena no array
  leituras[indice] = analogRead(pinoAnalogico);
  
  // Adiciona a nova leitura ao total
  total += leituras[indice];
  
  // Avança para a próxima posição no array
  indice = (indice + 1) % numLeituras;
  
  // Calcula a média e mapeia para a faixa de 0 a 25
  float setpoint = (total / numLeituras) * 0.0055;
  
  return setpoint;
}