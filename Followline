int sensor[5] = { A0, A1, A2, A3, A4 };
int pesos[5] = { -2, -1, 0, 1, 2 };
int leitura[5];
int normal[5];
float erro = 0;
int vetormin[5] = { 1023, 1023, 1023, 1023, 1023 };
int vetormax[5] = { 0, 0, 0, 0, 0 };
float erroanterior = 0;
// MOTORES
int IN1 = 6;
int IN2 = 9;
//int ENA = 5;  // PWM
int velocidade_motor_esquerdo = 0;
int motor_esquerdo[3] = { IN1, IN2 };
int IN3 = 5;
int IN4 = 3;
//int ENB = 6;  // PWM
int velocidade_motor_direito = 0;
int motor_direito[3] = { IN3, IN4 };
int minimo_motor = -255;
int maximo_motor = 255;
unsigned long tempoAN = 0;
bool esq90 = false;
bool dir90 = false;
float kp = 50;
float kd = 1.2;
int vel_base = 70;
bool gap;

// FUNÇÕES
void setup() {
  Serial.begin(9600);
  // PinMode dos sensores
  for (int i = 0; i < 5; i++) {
    pinMode(sensor[i], INPUT);
  }
  // PinMode dos motores
  for (int i = 0; i < 3; i++) {
    pinMode(motor_esquerdo[i], OUTPUT);
    pinMode(motor_direito[i], OUTPUT);
  }
  Serial.println("Calibrando...");
  calibracao();
  tempoAN = millis();
}
void calibracao() {
  unsigned long tempo_inicial = millis();
  while (millis() - tempo_inicial < 5000) {
    for (int i = 0; i < 5; i++) {
      leitura[i] = analogRead(sensor[i]);
      if (leitura[i] < vetormin[i]) {
        vetormin[i] = leitura[i];
      }
      if (leitura[i] > vetormax[i]) {
        vetormax[i] = leitura[i];
      }
    }
  }
}
void motores_on() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, vel_base);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, vel_base);
}
void motores_off() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void virar_esq(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
 
}
void virar_dir(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

}
void detectar90(){
  if(normal[0]>=90 && normal[1]>=90 && normal[2]>=90 && normal[3] <=90  && normal[4]<=90){
  esq90 = true;
  dir90 = false;
  }
  else if(normal[3]>90 && normal[4]>90 && normal[2]>90 && normal[1]<90 && normal[0] <90){
  dir90 = true;
  esq90 = false;
  }
  else
    dir90 = false;
    esq90 = false;
}

void curva_esquerda_90() {
  while (esq90) {
    //normalizacao();

    // gira sobre o próprio eixo
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);

    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  /*
    // condição de saída → achou linha no centro
    if (normal[2] > 90) {
      break;
    }
    */

  }
}

void curva_direita_90() {
  while (dir90) {
    //normalizacao();

    // gira sobre o próprio eixo
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);

    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
/*
    // condição de saída → achou linha no centro
    if ((normal[2] > 90) && (normal[0] < 100) && (normal[3] < 100) && (normal[4] < 100)){
      break;
    }
    */
  }
}

void normalizacao() {
  for (int i = 0; i < 5; i++) {
    leitura[i] = analogRead(sensor[i]);
    int intervalo = vetormax[i] - vetormin[i];
    if (intervalo == 0) intervalo = 1;
    normal[i] = (vetormax[i] - leitura[i]) * 100 / (intervalo);
    normal[i] = constrain(normal[i], 0, 100);
    Serial.print("|");
    Serial.print(normal[i]);
  }
  Serial.println("|");
  //delay(500);
}
float posicao_line() {
  float somapesos = 0;
  float somaleituras = 0;
  float posicaoponderada = 0;

  for (int i = 0; i < 5; i++) {
    somapesos += (normal[i] * pesos[i]);
    somaleituras += normal[i];
  }

  if (somaleituras == 0) {
    gap = true;
    return erroanterior; // mantém direção
  } else {
    gap = false;
  }

  posicaoponderada = somapesos / somaleituras;
  return posicaoponderada;
}

void controle() {
  erro = posicao_line();
  unsigned long tempoAT = millis();
  float dt = (tempoAT - tempoAN)/1000.0;
  tempoAN = tempoAT;
  if(dt<0.001) dt = 0.001;
  float der = (erro - erroanterior) / dt;
  float correcao = kp * erro + der*kd;
  erroanterior = erro;
  velocidade_motor_esquerdo = vel_base + (int)correcao;
  velocidade_motor_direito = vel_base - (int)correcao;
  velocidade_motor_esquerdo = constrain(velocidade_motor_esquerdo, minimo_motor, maximo_motor);
  velocidade_motor_direito = constrain(velocidade_motor_direito, minimo_motor, maximo_motor);
 if (velocidade_motor_esquerdo >= 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, velocidade_motor_esquerdo);
  } else {
    analogWrite(IN1, -velocidade_motor_esquerdo);
    analogWrite(IN2, 0);
  }
  if (velocidade_motor_direito >= 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, velocidade_motor_direito);
  } else {
    analogWrite(IN3, -velocidade_motor_direito);
    analogWrite(IN4, 0);
  }
}
void loop() {
  normalizacao();
  //controle();
  detectar90();

  if (esq90) {
    curva_esquerda_90();
  } 
  else if (dir90) {
    curva_direita_90();
  } 
  else {
    controle();
  }
  if(gap == true){
    controle();
  }
}
