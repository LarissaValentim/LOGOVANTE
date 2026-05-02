int sensor[5] = { A0, A1, A2, A3, A4 }, vetormin[5] = {1023,1023,1023,1023,1023}, vetormax[5] = {0,0,0,0,0};
int pesos[5] = { -2, -1, 0, 1, 2 }, leitura[5], normal[5];
float erro = 0, erroanterior = 0;
int IN1 = 6, IN2 = 9, IN3 = 5, IN4 = 3;
int velocidade_motor_esquerdo = 0, velocidade_motor_direito = 0;
int motor_esquerdo[2] = { IN1, IN2 };
int motor_direito[2] = { IN3, IN4 };
int minimo_motor = -255, maximo_motor = 255;
unsigned long tempoAN = 0;
bool esq90 = false, dir90 = false;
float kp = 50;
float kd = 1.2;
int vel_base = 70;
bool gap;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 5; i++) {
    pinMode(sensor[i], INPUT);
  }

  for (int i = 0; i < 2; i++) {
    pinMode(motor_esquerdo[i], OUTPUT);
    pinMode(motor_direito[i], OUTPUT);
  }

  calibracao();
  tempoAN = millis();
}

void calibracao() {
  unsigned long tempo_inicial = millis();
  while (millis() - tempo_inicial < 5000) {
    for (int i = 0; i < 5; i++) {
      leitura[i] = analogRead(sensor[i]);
      if (leitura[i] < vetormin[i]) vetormin[i] = leitura[i];
      if (leitura[i] > vetormax[i]) vetormax[i] = leitura[i];
    }
  }
}

void motores_on() {
  analogWrite(IN1, 0);
  analogWrite(IN2, vel_base);
  analogWrite(IN3, 0);
  analogWrite(IN4, vel_base);
}

void motores_off() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void virar_esq(){
  analogWrite(IN1, 0);
  analogWrite(IN2, 255);
  analogWrite(IN3, 255);
  analogWrite(IN4, 0);
}

void virar_dir(){
  analogWrite(IN1, 255);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 255);
}

void detectar90(){
  if(normal[0] >= 90 && normal[1] >= 90 && normal[2] >= 90 && normal[3] < 90 && normal[4] < 90){
    esq90 = true;
    dir90 = false;
  }
  else if (normal[3] > 90 && normal[4] > 90 && normal[2] > 90 && normal[1] < 90 && normal[0] < 90) {
    dir90 = true;
    esq90 = false;
  }
  else {
    dir90 = false;
    esq90 = false;
  }
}

void curva_esquerda_90() {
  while (esq90) {
    normalizacao();
    detectar90();

    analogWrite(IN1, 120);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, 120);

    if (normal[2] > 90) {
      esq90 = false;
    }
  }
}

void curva_direita_90() {
  while (dir90) {
    normalizacao();
    detectar90();

    analogWrite(IN1, 0);
    analogWrite(IN2, 120);
    analogWrite(IN3, 120);
    analogWrite(IN4, 0);

    if ((normal[2] < 90) && (normal[0] >= 90) && (normal[3] < 90) && (normal[4] < 90)) {
      dir90 = false;
    }
  }
}

void normalizacao() {
  for (int i = 0; i < 5; i++) {
    leitura[i] = analogRead(sensor[i]);
    int intervalo = vetormax[i] - vetormin[i];
    if (intervalo == 0) intervalo = 1;

    normal[i] = (vetormax[i] - leitura[i]) * 100 / intervalo;
    normal[i] = constrain(normal[i], 0, 100);
  }
}

float posicao_line() {
  float somapesos = 0;
  float somaleituras = 0;

  for (int i = 0; i < 5; i++) {
    somapesos += (normal[i] * pesos[i]);
    somaleituras += normal[i];
  }

  if (somaleituras == 0) {
    gap = true;
    return erroanterior;
  } else {
    gap = false;
  }

  return somapesos / somaleituras;
}

void controle() {
  erro = posicao_line();

  unsigned long tempoAT = millis();
  float dt = (tempoAT - tempoAN)/1000.0;
  tempoAN = tempoAT;

  if(dt < 0.001) dt = 0.001;

  float der = (erro - erroanterior) / dt;
  float correcao = kp * erro + kd * der;

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

void gap_w(){
  posicao_line();
  while(gap){
    motores_off();
  }
}

void loop() {
  normalizacao();
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
}
