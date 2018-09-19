// TerraDome by MagicManu
// 05/2018
// Video : https://www.youtube.com/watch?v=OtoiuGWk26w
// Blog : http://blog.magicmanu.com

#include <LiquidCrystal.h>  // Ecran LCD
#include <RGBLED.h>         // Leds RVB
#include <Servo.h>          // Servo moteurs
#include <PID_v1.h>         // Regul PID

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
char Tampon[40];        // Valeur temporaire pour affichage sur LCD
bool RefreshLCD = HIGH;

// Ecran et boutons
int lcd_key      = 0;
int lcd_key_last = 10;
int adc_key_in   = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

RGBLED rgbLed1(48,50,52,COMMON_CATHODE);  // LED RVB 1
RGBLED rgbLed2(49,51,53,COMMON_CATHODE);  // LED RVB 2

Servo ServoR; // Porte droite
Servo ServoL; // Porte gauche

const byte IN_TEMP = 46;   // Capteur de temperature et humidite
const byte IN_LUMI = A10;  // Capteur de lumiere

const byte OUT_LUMI = 43;  // Relais lumiere
const byte OUT_HEAT = 45;  // Relais chauffage
const byte OUT_VENT = 36;  // Ventilateur

const byte OUT_LED_OR = 40;  // Voyant orange
const byte OUT_LED_BL = 38;  // Voyant bleu

float temperature, humidity;  // Valeur T°C et Hum
float cons_temp = 27;         // Consigne de temperature
float cons_lumi = 8;          // Consigne de lumiere
float EcartTemp;
int light;                    // Puissance lumiere

bool LumiOn = LOW;            // Etat eclairage

// Pour la lecture de la température toutes les secondes
long TimeReadTemp = 0; 
int CptReadTemp = 1000;

// Pour commander l'éclarage toues les 60 secondes
long TimeCdeLight = 0; 
int CptCdeLight = 10000;

// Pour faire clignoter les voyants
long TimeVoyant = 0; 
int CptVoyant = 0;
bool ReverseV = LOW;  // Inversion Orange / Bleu

int NbVoyant = 0; // Nombre de clignotement avant maneuvre des portes


int STEP_GATE = 0;    // Etape ouverture des portes
bool OpenLast = LOW;  // Dernier etat des portes


/* Code d'erreur de la fonction readDHT11()  */
const byte DHT_SUCCESS = 0;        // Pas d'erreur
const byte DHT_TIMEOUT_ERROR = 1;  // Temps d'attente dépassé
const byte DHT_CHECKSUM_ERROR = 2; // Données reçues erronées





void setup() {

  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 100;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  

  // Servos des portes
  ServoR.attach(39);  // Droite
  ServoL.attach(41);  // Gauche

  // Portes fermées
  ServoR.write(4);
  ServoL.write(82);  

  // Init entrées
  pinMode(IN_TEMP, INPUT_PULLUP);

  // Init sorties
  pinMode(OUT_LUMI, OUTPUT);
  pinMode(OUT_HEAT, OUTPUT);
  pinMode(OUT_VENT, OUTPUT);
  pinMode(OUT_LED_OR, OUTPUT);
  pinMode(OUT_LED_BL, OUTPUT);  
  digitalWrite(OUT_LUMI, HIGH);
  digitalWrite(OUT_HEAT, HIGH);
  digitalWrite(OUT_VENT, LOW);
  digitalWrite(OUT_LED_OR, LOW);
  digitalWrite(OUT_LED_BL, LOW);

  // Init afficheur
  lcd.begin(16, 2);       
  lcd.setCursor(0,0);
  lcd.print("   TerraDome");
  rgbLed1.writeRGB(255,0,0);  
  rgbLed2.writeRGB(255,0,0);
  delay(1500);
  
  lcd.setCursor(0,1);
  lcd.print("   by MagicManu");  
  rgbLed1.writeRGB(0,255,0);
  rgbLed2.writeRGB(0,255,0);
  delay(2000);

}




void loop() {

  // Lecture des boutons
  lcd_key = read_LCD_buttons();
  if (lcd_key != lcd_key_last){
    switch (lcd_key){
      // Lumiere MOINS
      case btnLEFT:
      {
        cons_lumi--;
        if (cons_lumi < 1)
          cons_lumi = 1;
        TimeCdeLight = 0;
        break;
      }
      // Lumiere PLUS
      case btnRIGHT:
      {
        cons_lumi++;
        if (cons_lumi > 10)
          cons_lumi = 10;
        TimeCdeLight = 0;
        break;
      }      
      // Temp PLUS
      case btnUP:
      {
        cons_temp++;
        if (cons_temp > 40)
          cons_temp = 40;
        break;
      }
      // Temp MOINS
      case btnDOWN:
      {
        cons_temp--;
        if (cons_temp < 10)
          cons_temp = 10;
        break;
      }
    }
    lcd_key_last = lcd_key;
    RefreshLCD = HIGH;
  }
  

  // Lecture de la temperature et l'humidité une fois par seconde
  CptReadTemp = millis() - TimeReadTemp;
  if (CptReadTemp > 1000){
    readDHT11(IN_TEMP, &temperature, &humidity);
    TimeReadTemp = millis();
    RefreshLCD = HIGH;
  }


  // Affichage des valeurs
  if (RefreshLCD){
    // Affichage Temperature et humidité
    lcd.setCursor(0,0);
    sprintf(Tampon, "T:%d/%d C  H:%d%%", (int) temperature, (int) cons_temp, (int) humidity);   
    lcd.print(Tampon);  
    lcd.setCursor(7,0);    
    lcd.print((char)223);
  
    // Affichage niveau de lumiere
    light = analogRead(IN_LUMI) / 100;
    lcd.setCursor(0,1);
    sprintf(Tampon, "Lumi: %d/%d      ", (int) light, (int) cons_lumi);     
    lcd.print(Tampon);

    RefreshLCD = LOW;
  }
  



  // Commande éclairage plantes (changement possible toutes les 60 secondes)
  CptCdeLight = millis() - TimeCdeLight;
  if (CptCdeLight > 10000){
    LumiOn = (light > cons_lumi);
    digitalWrite(OUT_LUMI, LumiOn);
    TimeCdeLight = millis();
  }


  // Commande chauffage / ventilateur / portes
  EcartTemp = round(temperature - cons_temp);
  if (EcartTemp > 2){
    // Aeration
    OpenGates(HIGH);
    digitalWrite(OUT_HEAT, HIGH); // Arrête le chauffage
    digitalWrite(OUT_VENT, HIGH); // Demarre le ventilateur
    rgbLed1.writeRGB(0,0,255);  
    rgbLed2.writeRGB(0,0,255);
  }
  else if (EcartTemp <= -2) {
    // Chauffage
    OpenGates(LOW);
    //digitalWrite(OUT_HEAT, LOW); // Demarre le chauffage
    digitalWrite(OUT_VENT, LOW); // Arrête le ventilateur
    rgbLed1.writeRGB(255,0,0);  
    rgbLed2.writeRGB(255,0,0);

    // Regulation PID
    Input = temperature;
    myPID.Compute();
    unsigned long now = millis();
    if (now - windowStartTime > WindowSize)
      windowStartTime += WindowSize;
    if (Output > now - windowStartTime)
      digitalWrite(OUT_HEAT, LOW); // Demarre le chauffage  
    else
      digitalWrite(OUT_HEAT, HIGH); // Arrête le chauffage
  }
  else if (EcartTemp == 0){
    // A température
    OpenGates(LOW); 
    digitalWrite(OUT_HEAT, HIGH); // Arrête le chauffage
    digitalWrite(OUT_VENT, LOW);  // Arrête le ventilateur
    rgbLed1.writeRGB(0,255,0);  
    rgbLed2.writeRGB(0,255,0);
  }
  

}

// Commande des portes
void OpenGates(bool Open){

  CptVoyant = millis() - TimeVoyant;

  if (STEP_GATE == 3 and OpenLast != Open)
    STEP_GATE = 0;  
  

  switch (STEP_GATE){
    case 0:
      STEP_GATE = 1;
      break;
      
    case 1:
      // Clignotement voyants Orange / Bleu
      if (CptVoyant > 200){
        digitalWrite(OUT_LED_OR, ReverseV);
        digitalWrite(OUT_LED_BL, !ReverseV);    
        ReverseV = !ReverseV;
        TimeVoyant = millis();
        NbVoyant++;
        if (NbVoyant > 20)
          STEP_GATE = 2;
      }
      break;
      
    case 2:      
      // Maneuvre des portes
      digitalWrite(OUT_LED_OR, LOW);
      digitalWrite(OUT_LED_BL, LOW);   
      if (Open){
        // Ouverture
        ServoR.write(90);
        ServoL.write(4);    
      }
      else{
        // Fermeture
        ServoR.write(4);
        ServoL.write(82);            
      }
      OpenLast = Open;
      NbVoyant = 0;
      STEP_GATE = 3;
      break;    
  }


}


// Lecture des boutons
int read_LCD_buttons(){
  adc_key_in = analogRead(0);      // read the value from the sensor 
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 250)  return btnUP; 
  if (adc_key_in < 300)  return btnDOWN; 
  if (adc_key_in < 450)  return btnLEFT; 
  if (adc_key_in < 850)  return btnSELECT;  
  
  return btnNONE;  // when all others fail, return this...
}



/**
 * Lit la température et le taux d'humidité mesuré par un capteur DHT11.
 *
 * @param pin Broche sur laquelle est câblée le capteur.
 * @param temperature Pointeur vers la variable stockant la température.
 * @param humidity Pointeur vers la variable stockant le taux d'humidité.
 * @return DHT_SUCCESS si aucune erreur, DHT_TIMEOUT_ERROR en cas de timeout, ou DHT_CHECKSUM_ERROR en cas d'erreur de checksum.
 */
byte readDHT11(byte pin, float* temperature, float* humidity) {
  
  /* Lit le capteur */
  byte data[5];
  byte ret = readDHTxx(pin, data, 18, 1000);
  
  /* Détecte et retourne les erreurs de communication */
  if (ret != DHT_SUCCESS) 
    return ret;
    
  /* Calcul la vraie valeur de la température et de l'humidité */
  *humidity = data[0];
  *temperature = data[2];

  /* Ok */
  return DHT_SUCCESS;
}



/**
 * Fonction bas niveau permettant de lire la température et le taux d'humidité (en valeurs brutes) mesuré par un capteur DHTxx.
 */
byte readDHTxx(byte pin, byte* data, unsigned long start_time, unsigned long timeout) {
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  // start_time est en millisecondes
  // timeout est en microsecondes
 
  /* Conversion du numéro de broche Arduino en ports / masque binaire "bas niveau" */
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *ddr = portModeRegister(port);   // Registre MODE (INPUT / OUTPUT)
  volatile uint8_t *out = portOutputRegister(port); // Registre OUT (écriture)
  volatile uint8_t *in = portInputRegister(port);   // Registre IN (lecture)
  
  /* Conversion du temps de timeout en nombre de cycles processeur */
  unsigned long max_cycles = microsecondsToClockCycles(timeout);
 
  /* Evite les problèmes de pull-up */
  *out |= bit;  // PULLUP
  *ddr &= ~bit; // INPUT
  delay(100);   // Laisse le temps à la résistance de pullup de mettre la ligne de données à HIGH
 
  /* Réveil du capteur */
  *ddr |= bit;  // OUTPUT
  *out &= ~bit; // LOW
  delay(start_time); // Temps d'attente à LOW causant le réveil du capteur
  // N.B. Il est impossible d'utilise delayMicroseconds() ici car un délai
  // de plus de 16 millisecondes ne donne pas un timing assez précis.
  
  /* Portion de code critique - pas d'interruptions possibles */
  noInterrupts();
  
  /* Passage en écoute */
  *out |= bit;  // PULLUP
  delayMicroseconds(40);
  *ddr &= ~bit; // INPUT
 
  /* Attente de la réponse du capteur */
  timeout = 0;
  while(!(*in & bit)) { /* Attente d'un état LOW */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }
    
  timeout = 0;
  while(*in & bit) { /* Attente d'un état HIGH */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }

  /* Lecture des données du capteur (40 bits) */
  for (byte i = 0; i < 40; ++i) {
 
    /* Attente d'un état LOW */
    unsigned long cycles_low = 0;
    while(!(*in & bit)) {
      if (++cycles_low == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }

    /* Attente d'un état HIGH */
    unsigned long cycles_high = 0;
    while(*in & bit) {
      if (++cycles_high == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }
    
    /* Si le temps haut est supérieur au temps bas c'est un "1", sinon c'est un "0" */
    data[i / 8] <<= 1;
    if (cycles_high > cycles_low) {
      data[i / 8] |= 1;
    }
  }
  
  /* Fin de la portion de code critique */
  interrupts();
 
  /*
   * Format des données :
   * [1, 0] = humidité en %
   * [3, 2] = température en degrés Celsius
   * [4] = checksum (humidité + température)
   */
   
  /* Vérifie la checksum */
  byte checksum = (data[0] + data[1] + data[2] + data[3]) & 0xff;
  if (data[4] != checksum)
    return DHT_CHECKSUM_ERROR; /* Erreur de checksum */
  else
    return DHT_SUCCESS; /* Pas d'erreur */
}
