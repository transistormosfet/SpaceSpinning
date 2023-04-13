/*
   Codice di Davide Di Stasio
   “All Rights Reserved"
   04/11/2021
*/

byte customChardwn[8] = {                                                 //carattere personalizzato freccia in basso
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b10101,
  0b01110,
  0b00100
};

byte customCharup[8] = {                                                    //carattere personalizzato freccia in alto
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};
//#include <Stepper.h>                                                        //inclusione libreria stepper
#include <LiquidCrystal_I2C.h>                                              //inclusione libreria LCD
//#include <Servo.h>                                                          //inclusione libreria servo
#include <Adafruit_Sensor.h>                                                //librerie sensore umidità e temperatura
#include <DHT.h>
#include <DHT_U.h>
#include<Wire.h>                                                            //libreria I2C per RTC(default)
#include"RTClib.h"                                                          //libreria RTC
RTC_DS1307 rtc;                                                                 //oggetto rtc
#define PINSD 53                                                            //pin SS dell'RTC arduino mega 53, arduino uno 10
#include<SPI.h>                                                             //libreria SPI per modulo SD(default)
#include<SD.h>                                                              //libreria SD (default)
#include <Adafruit_MCP4725.h>                                               //libreria per dac MCP4725
//#define motorInterfaceType 1;                                               //solo due pin: 4->DIR | 5(PWM)->STEP
#include <Adafruit_PWMServoDriver.h>                                        //libreria per pwm esterno modulo PCA9685 servomotori
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();                    //creazione oggetto pwm

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates

File logfile;                                                               //creazione oggetto logfile (default)

const int stepspergiro = 200;                                               // step per un giro 360°, dipende dal modello di motore (datasheet) nema8
int passi = 60;                                                             //step (in secondi) di incremento e decremento parametri temporali

int stato = 0;                                                              //stato per lo switch case (macchina a stati)

bool first = true;                                                          //serve a non far blinkare le scritte (flag che indica solo la prima volta che si entra in un menù)

bool inizio = false;                                                        //stato di inizio sequenza di start del timer
bool pausa = false;                                                          //stato di pausa sequenza di elettrofilatura
bool sicurezza = true;                                                      //parametro sicurezza per bottone di stop
bool primavolta = true;
bool primerrimavolta = true;

unsigned long tsicurezza = 2500;                                            //tempo sicurezza scarica Hv = 2500 ms

int x = 6808;                                                                 //parametro per lunghezza scorrimento nastro 200 == 1giro di motore
int i = 0;                                                                  //contatore ausiliario di step alla volta
int N = 1;                                                                  //incamera il numero di prove decise da menù
int n = 0;                                                                  //scala insieme al numero di prove eseguite
int NMAX = 5;                                                               //numero max di prove da definire

float voltaggio = 10.0;
float voltaggioMAX = 20.0;                                                  //voltaggio massimo da definire kV
int PWM;
float portata = 1;                                                          //portata in mL/h
float portataMAX;                                                           //portata massima da definire
float volumestep = 0.0000804944311857;                                                //volume di uno step del motore 2 in mL (Valore per fullstep=0.015119)

unsigned long tde = 300, t0, t1 , tstep, tstep2, tdata;                     //tde = t debounce 300 ms e t0,t1 variabili di servizio per conteggio debounce (t0) e contatore 1s (t1)
unsigned long deltapausa = 0, tpausain = 0, tpausafin = 0;
unsigned long tempo = 60;                                                   //variabili tmpo di elettrofilatura in memoria impostate dall'utente
unsigned long VARtempo = 0 ;                                                //variabili tempi di elettrofilatura trascorsa che incrementano durante il processo
unsigned long tstart = 0;                                                   //tempo inizio sequenza temporizzata
unsigned long tempoMAX = 36000;                                             //tempo max(in secondi) 3600s(secondi in 1 ora) * 10 ore
unsigned long pausapasso;

float dvoltaggio = 0 ;                                                      //variazione voltaggio durante le prove dV
float dportata = 0 ;                                                        //variazione portata durante le prove dQ

float temperatura = 0;                                                      //variabile immagazzinamento temperatura
float umidita = 0;                                                          //è scritto male di proposito, non cambiare

LiquidCrystal_I2C lcd(0x27, 16, 2);                                         //inizializzazione LCD indirizzo e tipo 16X2

//Servo myservo;                                                              //creazione oggetto myservo

#define DHTPIN 2                                                            //pin sensore DHT11
#define DHTTYPE    DHT22                                                    //definisci tipo di sensore
DHT_Unified dht(DHTPIN, DHTTYPE);

Adafruit_MCP4725 dac;                                                       //definisco l'oggetto dac per il generatore Hv
#define DAC_RESOLUTION (8)

//Stepper motore(stepspergiro, 9, 13);                                         //pin Arduino-> 13,9
//Stepper motore2(stepspergiro, 12, 11);                                      //pin Arduino-> 11->DIR | 12(PWM)-> STEP POMPA

void setup() {
  /*
     definizione entrate e uscite
  */
  pinMode(A0, INPUT);                                                       //up sw (tutti i bottoni premuti vanno alti)
  pinMode(A1, INPUT);                                                       //down sw
  pinMode(A2, INPUT);                                                       //dx sw
  pinMode(A3, INPUT);                                                       //sx sw
  pinMode(A4, INPUT);                                                       //Bottone fast forward premuto-> HIHG
  pinMode(A5, INPUT);                                                       //Bottone fast rewind premuto-> HIGH
  pinMode(8, INPUT_PULLUP);                                                 //Bottone STOP premuto-> LOW
  pinMode(7, OUTPUT);                                                       //LED STOP
  pinMode(13, OUTPUT);                                                      //step pin motore
  pinMode(9, OUTPUT);                                                       //dir  pin motore
  pinMode(12, OUTPUT);                                                      //step pin motore
  pinMode(11, OUTPUT);                                                      //dir pin motore
  pinMode(6, OUTPUT);                                                       //pin del relè che stacca a monte l'uscita del dac
  pinMode(5, OUTPUT);                                                       //pin del relè ausiliario
  digitalWrite(5,LOW);                                                      //tieni spento il relè ausiliario
  digitalWrite(6,LOW);                                                      //stacca il relè
  //  motore.setSpeed(60);                                                  //setta velocità motore rpm
  //  myservo.attach(10);                                                   //definisci pin servo
  //  myservo.write(0);                                                     //sblocco sportello
  pwm.setPWM(0, 0,SERVOMIN);                                                //sblocco sportello (numero servo,inizio impulso,fine impulso);
  pwm.setPWM(1, 0,SERVOMIN);                                                //sblocco sportello (numero servo,inizio impulso,fine impulso);
  pwm.setPWM(2, 0,SERVOMIN);                                                //sblocco sportello (numero servo,inizio impulso,fine impulso);
  
  Wire.begin();                                                             //inizializza I2C
  rtc.begin();                                                              //inizializza Real Time Clock
  if (!rtc.isrunning()) {                                                   //se prima volta funzionamento RTC
    rtc.adjust(DateTime(__DATE__, __TIME__));                               //immetti data e ora della compilazione sketch in RTC
  }
  if (!SD.begin(PINSD)) {                                                   //se la SD non collegata e formattata correttamente FAT
    while (1) {                                                             //blocca in un ciclo infinito senza uscita
      digitalWrite(7, HIGH);                                                //lampeggia spia STOP
      delay(500);
      digitalWrite(7, LOW);
      delay(500);
    }
  }
  dht.begin();                                                              //inizializza sensore dht
  lcd.init();                                                              //inizializza lcd
  lcd.backlight();                                                          //accendi retroilluminazione

  lcd.createChar(0, customChardwn);                                         //crea il carattere custom 0 fraccia basso
  lcd.createChar(1, customCharup);                                          //crea il carattere custom 1 freccia alto  lcd.begin(16, 2);                                                         //inizializza LCD
  lcd.setCursor(0, 0);
  lcd.print(" SpaceSpinning  ");
  lcd.setCursor(3, 1);
  lcd.print("20/10/2021");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DAVIDE DI STASIO");
  lcd.setCursor(3, 1);
  lcd.print("20/10/2021");
  delay(1000);
  lcd.clear();
  dac.begin(0x60);        //0x60 indirizzo del clone cinese, 0x62 indirizzo dac originale adafruit
  pwm.begin();            //inizializzazione pwm esterno
  dac.setVoltage(0, false);             //spegni generatore Hv
  digitalWrite(6,HIGH);                     //accendi il relè
  pwm.setOscillatorFrequency(27000000); //27MHz
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

}


void loop() {
  /*
     misura umidità e temperatura
  */
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  temperatura = event.temperature;
  dht.humidity().getEvent(&event);
  umidita = event.relative_humidity;                                        //umidità è scritto male di proposito NON CAMBIARE

  /*
     storage dati in SD
  */
  if (millis() - tdata > 6000) {                                            //ogni minuto = 60000ms
    tdata = millis();                                                       //resetta variabile servizio
    DateTime now = rtc.now();                                               //preleva info da RTC
    logdata(now, temperatura, umidita, inizio, portata, voltaggio, n);                                 //usa funzione ausiliaria logdata per incamerare data, temp,hum
  }

  /*
     ----------------------SICUREZZA-------------------------------------------------
  */
  if ((digitalRead(8) == LOW) && (sicurezza == true) && (millis() - t0 > tde)) {        //se premo il bottone e la sicurezza è vera e supero il debounce
    t0 = millis();                                                                      //cambio lo stato in falso(STOP) e resetto la variabile di servizio
    n = 0;                                                                              //azzera conteggio prove per resettare l'intero sistema
    analogWrite(6, 0);                                                                  //spengo generatore Hv
    pompa(0);                                                                           //SPEGNI POMPA
    lcd.setCursor(0, 0);                                                                //scrivi immediatamente su LCD emergency stop
    lcd.print(" EMERGENCY STOP ");
    lcd.setCursor(0, 1);
    lcd.print("                  ");
    inizio = false;                                                                     //azzero flag inizio sequenza elettrofilatura
    sicurezza = false;                                                                  //no sicurezza
    pausa = false;                                                                      //azzero flag pausa (se fosse stato impostato precedentemente in questo modo lo resetto)
    tpausain = tpausafin = deltapausa = 0;                                              //resetto tempi di pausa
  }
  else if ((digitalRead(8) == LOW) && (sicurezza == false) && (millis() - t0 > tde)) {  //se premo il bottone e la sicurezza è falsa (STOP) e supero il debounce    to = millis();                                                                      //cambio lo stato in vero (ripresa dell'eserimento) e resetto variabile di servizio
    t0 = millis();                                                                      //riazzera contatore debounce
    sicurezza = true;                                                                   //flag sicurezza TRUE ma non flag INIZIO (perché l'elettrofilatura non deve ripartire)
    primavolta = true;                                                                  //ripristina flag primavolta scorrimento nastro
    setStato(40);                                                                       //ritorno al menù S40
  }

  /*
     ---------------------------------------MACCHINA A STATI PER LCD-------------------------------
  */
  if (sicurezza) {
    switch (stato) {
      case 0:
        S0();
        break;
      case 10:
        S10();
        break;
      case 20:
        S20();
        break;
      case 30:
        S30();
        break;
      case 40:
        S40();
        break;
      case 1:
        S1();
        break;
      case 2:
        S2();
        break;
      case 3:
        S3();
        break;
      case 4:
        S4();
        break;
      case 5:
        S5();
        break;
      case 6:
        S6();
        break;
      case 7:
        S7();
        break;
      case 8:
        S8();
        break;
      case 9:
        S9();
        break;
    }

    /*
      controllo del fast forward e fast rewind con pulsanti
    */

    if (digitalRead(A4) == 1) {
      fast_forward();
    }
    if (digitalRead(A5) == 1) {
      fast_rewind();
    }

    /*
      procedura di elettrofilatura solo quando
      ho premuto il bottone dx nel menu START cioè inizio == true
      && tempo trascorso(millis) da quando premo dx da start valutando anche pause < tempo impostato di elettrofilatura
    */

    if ((inizio) && ((millis() - (tstart + deltapausa)) < (tempo * 1000)) && (n < N)) {  //se inizzi filatura e sei sotto al tempo di filatura di un campione (valutando eventuali tempi di pausa introdotti dall'uscita dai menù s5 s6 s7) e non hai finito l'intera sequenza di prove N

      if ((millis() - t1 > 1000)) {                                     //ogni secondo
        t1 = millis();                                                  //resetta il contatore ausiliario per lo step di 1 secondo
        VARtempo = (millis() - (tstart + deltapausa)) / 1000;           //refresh valore dell'indicatore di tempo
      }
      //      myservo.write(90);                                                //blocca portello
      pwm.setPWM(0, 0, SERVOMAX);                                           // valore massimo (da rivedere il valore per 90°)
      pwm.setPWM(1, 0, SERVOMAX);                                           // valore massimo (da rivedere il valore per 90°)
      pwm.setPWM(2, 0, SERVOMAX);                                           // valore massimo (da rivedere il valore per 90°)
      PWM = ((((n * dvoltaggio) + voltaggio) / voltaggioMAX) * 4095);   //restituisce il valore per il dac 12bit (da 0 a (2^12)-1 ) a seconda anche del numero della prova n (1^, 2^,ecc..)
      if (PWM > 4095) PWM = 4095;                                       //valore max accettato di pwm è 4095 = (2^12)-1
      dac.setVoltage(PWM, false);                                       //regola gen Hv
      pompa(portata);                                                   //aziona motore pompa secondo la portata
    }
    else if ((inizio) && ((millis() - (tstart + deltapausa)) > (tempo * 1000)) && (n < N)) {        //se finisci il tempo di una prova ma non l'intera sequenza di N prove
      n++;                                                              //incrementa contatore prove ausiliario
      pompa(0);                                                         //SPEGNI POMPA
      dac.setVoltage(0, false);                                         //spegni generatore Hv
      nastro();                                                         //muovi nastro trasportatore di L precisa
      VARtempo = 0;                                                     //azzera conteggio tempo di elettrofilatura trascorsa indicato su LCD
      tstart = millis();                                                //riazzera tstart per far ripartire la sequenza (n+1)esima
      tpausain = tpausafin = deltapausa = 0;                            //riazzera contatori di pausa (va fatto ogni qual volta si riazzera tstart)
    }
    else if ((n >= N) && (inizio == true)) {                            //se finisco procedura elettrofilatura superando tutte le N prove
      pompa(0);                                                         //SPEGNI POMPA
      dac.setVoltage(0, false);                                         //spegni generatore Hv
      nastro();                                                         //muovi nastro trasportatore di L precisa
      VARtempo = 0;                                                     //azzera conteggio tempo indicato su LCD
      setStato(40);                                                     //ritorna al menù start
      if ((millis() - t0) > tsicurezza) {                               //per aspettare il tempo di sicurezza scarica Hv (2,5s)
        t0 = millis();
        //        myservo.write(0);                                               //sblocco sportello
        pwm.setPWM(0, 0, SERVOMIN);
        pwm.setPWM(1, 0, SERVOMIN);
        pwm.setPWM(2, 0, SERVOMIN);
        inizio = false;                                                 //resetto flag inizio filatura
        n = 0;                                                          //resetto contatore ausiliario di prove
        VARtempo = 0;                                                   //resetto contatore tempo trascorso
        setStato(40);                                                   //vado al menù S40
      }
    }
  }

  /*
      controllo del servomotore di chiusura e della spia luminosa di stop
  */
  if ((!sicurezza) && (millis() - t0 > tsicurezza )) {                  //per aspettare il tempo di sicurezza scarica Hv (2,5s)
    t0 = millis();
    if (primavolta) {                                                   //prima volta che sicurezza è falsa(impedisce che scorrimento nastro si ripeta)
      primavolta = false;                                               //azzeri fag primavolta
      nastro();                                                         //muovi il nastro (scarta il campione automaticamente)
    }
    //    myservo.write(0);                                                   //sblocco sportello
    pwm.setPWM(0, 0, SERVOMIN);
    pwm.setPWM(1, 0, SERVOMIN);
    pwm.setPWM(2, 0, SERVOMIN);
    digitalWrite(7, HIGH);                                              //led STOP acceso
  }
  else if (sicurezza)  {
    digitalWrite(7, LOW);                                               //led STOP spento
  }
  if ((t0 >= 4000000000) || (t1 >= 4000000000) || (tstep >= 4000000000) || (tdata >= 4000000000) || (tpausain >= 4000000000) || (tpausafin >= 4000000000)) { //resetta arduino se una delle variabili temporali si riempiono troppo
    asm volatile (" jmp 0 ");                                               //codice ASSEMBLY per RESET arduino (jump to 0 line) è come premere il pulsante RESET della scheda
  }
}

/*
   funzione di servizio storage dati in SD
*/
void logdata(DateTime now, float t, float h, bool in, float Q, float v, int n ) {                              //prendi le variabili di input
  String msg =                                                              //forma una stringa di nome msg
    String(pad(now.day())) + "/" +                                          //assembla i vari parametri convertiti in stringhe
    String(pad(now.month())) + "/" +                                        // \t è una tabulazione
    String(pad(now.year())) + "\t" +                                        //pad() è una funzione ausiliaria di padding
    String(pad(now.hour())) + ":" +
    String(pad(now.minute())) + ":" +
    String(pad(now.second())) + "\t" +
    "T: " + "\t" + String(t) + "\t" + "  C" + "\t" +
    "H: " + "\t" + String(h) + "\t" + " %" + "\t" +
    "in" + "\t" + String(in) + "\t" +
    "Q: " + "\t" + String(Q) + "\t" + " mL/min" + "\t" +
    "V: " + "\t" + String(v) + "\t" + " kV"
    "n: " + "\t" + String(n);

  logfile = SD.open("thlog.txt", FILE_WRITE);                               //apri il file in modalita scrittura e chiamalo thlog.txt (nomi corti max 6 caratteri altrimanti non funziona)
  if (logfile) {                                                            //se logfile OK
    logfile.println(msg);                                                   //inserisci su nuova riga la stringa msg
    logfile.close();                                                        //chiudi il file (OBBLIGATORIO)
  }
}

/*
  funzione ausiliaria di padding
*/
String pad(int a) {                                                         //restituisci una stringa input numero a
  String ret = "";                                                          //forma una stringa nome ret
  if (a < 10) ret = "0" + String(a);                                        //se il numero ha 1 cifra cioè <10 aggiungi uno 0 davanti ad a
  else ret = String(a);                                                     //altrimenti lascia stare
  return ret;                                                               //restituisci stringa
}

/*
  -----------gestione avanzamento veloce pompa------------------------
*/
void fast_forward() {
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(12, LOW);
}

void fast_rewind() {
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  digitalWrite(12, LOW);
}

/*
  -----------gestione avanzamento nastro------------------------
*/
void nastro() {
  while (i <= x) {                                                      //i contatore di steps
    if (((millis() - tstep) > 2)) {                                    //esegui ogni 2 ms
      tstep = millis();                                                 //resetta variabile ausiliaria
      i++;                                                              //incrementa i di 1
      digitalWrite(9, LOW);                                            //gira in senso orario di 1 step per volta;
      digitalWrite(13, HIGH);
      digitalWrite(13, LOW);
    }
  }
  i = 0;                                                                //finito il ciclo azzera contatore
}
/*
  funzione per regolazione portata pompa
*/
void pompa(float flow) {
  if (!(flow == 0)) {

    if (primerrimavolta) { // così ci entra una sola volta e lancia il timer una volta sola. Verrà poi stoppato quando sarà chiamato pompa(0)
      pausapasso = ((volumestep / (flow)) * 3600000000) / 16; // il diviso 16 è dovuto al clock utilizzato
      startTimer1(pausapasso); //lancia il timer
      primerrimavolta = false;
    }
    //se flow == 0 non azionare la pompa
    //unsigned long pausapasso = volumestep / (flow / 3600000);           // t= Volume di 1 step/Q.|.Q in mL/ms e Volume di 1 step in mL
  }
  else {
    TCCR1B = 0;//stop timer
    TCNT1 = 0;//clear timer counts//se pompa(0)
    digitalWrite(11, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(12, LOW);                                                //togli enable dai drivers
    primerrimavolta = true; //resetto

  }
}

void startTimer1(unsigned long current_period)  //https://electronoobs.com/eng_arduino_tut140.php
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= B00000100;
  TIMSK1 |= B00000010;
  OCR1A = current_period;
  sei();
}
ISR(TIMER1_COMPA_vect)   // https://electronoobs.com/eng_arduino_tut140.php
{
  TCNT1  = 0;
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(12, LOW);
}

/*
  STATI DELLO SWITCH E CAMBIO STATO
*/
void setStato(int s) {                                                  //funzione di comodo per passare da uno stato all'altro
  stato = s;                                                            //impone lo stato desiderato
  first = true;                                                         //resetta la flag first imponendola true
  lcd.clear();                                                          //ripulisci LCD
}

void S0() {
  if (first) {                                                          //se è la prima volta che entri in questo menù scrivi le etichette
    lcd.setCursor(0, 0);
    lcd.print("N TESTS         ");                                      //gli spazzi servono per ripulire la riga da eventuali caratteri rimanenti
    lcd.setCursor(0, 1);
    lcd.write((byte)0);                                                 //freccia basso
    lcd.print("NEXT       >SET");
    first = false;                                                      //cambia il flag per far capire che hai già scritto le etichete
  }
  /*
    leggi i bottoni e muoviti di conseguenza tra gli stati
  */
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde)) {                //se spingi bottone basso -> stato = 10
    t0 = millis();
    setStato(10);
  }
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde) ) {               //se spingi bottone destra -> stato = 1
    t0 = millis();
    setStato(1);
  }
};

void S10() {
  if (first) {                                                          //se è la prima volta che entri in questo menù scrivi le etichette
    lcd.setCursor(0, 0);
    lcd.print("time            ");                                      //gli spazzi servono per ripulire la riga da eventuali caratteri rimanenti
    lcd.setCursor(0, 1);
    lcd.write((byte)0);                                                 //freccia basso
    lcd.print("NEXT       >SET");
    first = false;                                                      //cambia il flag per far capire che hai già scritto le etichete
  }
  /*
    leggi i bottoni e muoviti di conseguenza tra gli stati
  */
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde)) {                //se spingi bottone basso -> stato = 20
    t0 = millis();
    setStato(20);
  }
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde) ) {               //se spingi bottone destra -> stato = 2
    t0 = millis();
    setStato(2);
  }
};

void S20() {
  if (first) {                                                          //se è la prima volta che entri in questo menù scrivi le etichette
    lcd.setCursor(0, 0);
    lcd.print("HVreg           ");                                      //gli spazzi servono per ripulire la riga da eventuali caratteri rimanenti
    lcd.setCursor(0, 1);
    lcd.write((byte)0);                                                 //freccia basso
    lcd.print("NEXT       >SET");
    first = false;                                                       //cambia il flag per far capire che hai già scritto le etichete
  }
  /*
    leggi i bottoni e muoviti di conseguenza tra gli stati
  */
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde)) {                //se spingi bottone basso -> stato = 30
    t0 = millis();
    setStato(30);
  }
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde) ) {               //se spingi bottone destra -> stato = 3
    t0 = millis();
    setStato(3);
  }
};

void S30() {
  if (first) {                                                          //se è la prima volta che entri in questo menù scrivi le etichette
    lcd.setCursor(0, 0);
    lcd.print("FLOW            ");                                      //gli spazzi servono per ripulire la riga da eventuali caratteri rimanenti
    lcd.setCursor(0, 1);
    lcd.write((byte)0);                                                 //freccia basso
    lcd.print("NEXT       >SET");
    first = false;                                                  //cambia il flag per far capire che hai già scritto le etichete
  }
  /*
    leggi i bottoni e muoviti di conseguenza tra gli stati
  */
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde)) {                //se spingi bottone basso -> stato = 40
    t0 = millis();
    setStato(40);
  }
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde) ) {               //se spingi bottone destra -> stato = 4
    t0 = millis();
    setStato(4);
  }
};
void S40() {
  if (first) {                                                          //se è la prima volta che entri in questo menù scrivi le etichette
    lcd.setCursor(0, 0);
    lcd.print(" START SEQUENCE?");                                      //gli spazzi servono per ripulire la riga da eventuali caratteri rimanenti
    lcd.setCursor(0, 1);
    lcd.write((byte)0);                                                 //freccia basso
    lcd.print("NEXT     >START");
    first = false;                                                      //cambia il flag per far capire che hai già scritto le etichete
  }
  /*
    leggi i bottoni e muoviti di conseguenza tra gli stati
  */
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde)) {                //se spingi bottone basso -> stato = 0
    t0 = millis();
    setStato(0);
  }
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde) ) {               //se spingi bottone destra -> stato = 5
    t0 = millis();
    setStato(5);
    inizio = true;                                                      //tag inizio sequenza elettrofilatura ON
    if (!pausa)tstart = millis();                                                 //segna tempo di partenza filatura (se non rientri da una pausa)
    if (pausa) {                                                        //se rientri da una pausa
      pausa = false;                                                    //azzera il tag pausa
      tpausafin = millis();                                             //memorizza il tempo fine pausa
      deltapausa = (deltapausa + (tpausafin - tpausain));               //calcola la pausa aggiuntiva che hai fatto da quando sei uscito da S5,S6,S7 premendo il bottone a sx
    }
  }
};
void S1() {
  if (first) {
    lcd.setCursor(0, 0);
    lcd.print("N=              ");
    lcd.setCursor(0, 1);
    lcd.write((byte)1);                                                 //freccia alto
    lcd.print("+ ");
    lcd.write((byte)0);                                                 //freccia basso
    lcd.print("-       <SET");
    first = false;
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde)) {                //se spingi bottone sinistra -> stato = 0
    t0 = millis();
    setStato(0);
  }
  if ((digitalRead(A0) == 1) && (millis() - t0 > tde)) {                //se spingi alto -> incrementa N
    t0 = millis();
    N++;
  }
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde)) {                //se spingi basso && debounce -> decrementa tempo iniziale timer 2
    t0 = millis();
    N--;
  }                                                                     //il parametro N diventa ciclico
  if (N > NMAX) {
    N = 0;
  }
  if (N < 0) {
    N = NMAX;
  }
  lcd.setCursor(3, 0);
  lcd.print(N);
  lcd.print("            ");
};
void S2() {
  if (first) {
    lcd.setCursor(0, 0);
    lcd.print("t=     s");
    lcd.setCursor(0, 1);
    lcd.write((byte)1);                                                 //freccia alto
    lcd.print("+ ");
    lcd.write((byte)0);
    lcd.print("-       <SET");
    first = false;
  }
  if ((digitalRead(A0) == 1) && (millis() - t0 > tde)) {                //se spingi bottone up -> incrementa tempo
    t0 = millis();
    tempo = tempo + passi;
  }
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde) ) {               //se spingi bottone basso -> decrementa tempo
    t0 = millis();
    tempo = tempo - passi;
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde)) {                //se spingi bottone sinistra -> stato S10
    t0 = millis();
    setStato(10);
  }                                                                             //il parametro tempo diventa ciclico
  if (tempo >= (tempoMAX + passi) && (tempo < (tempoMAX + passi + passi))) {    //compreso nel primo segmento vicino al tempoMAX
    tempo = 0;
  }
  if (tempo <= (4294967296 - passi) && (tempo > (4294967296 - passi - passi))) {  //compreso nel primo segmento vicino al massimo di unsigned long (4294967296)
    tempo = tempoMAX;
  }
  lcd.setCursor(2, 0);
  lcd.print(tempo);
  lcd.print("s");
  lcd.print("                ");
};
void S3() {
  if (first) {                                                          //se è la prima volta che entri in questo menù scrivi le etichette
    lcd.setCursor(0, 0);
    lcd.print("Hv=    kV         ");
    lcd.setCursor(0, 1);
    lcd.write((byte)1);                                                 //freccia alto
    lcd.print("+ ");
    lcd.write((byte)0);
    lcd.print("-  >dV  <SET");
    first = false;
  }
  /*
    leggi i bottoni e muoviti di conseguenza negli sati
  */
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde)) {                //se spingi bottone destra -> stato = 8
    t0 = millis();
    setStato(8);
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde) ) {               //se spingi bottone sinistra -> stato = 20
    t0 = millis();
    setStato(20);
  }
  if ((digitalRead(A0) == 1) && (millis() - t0 > tde)) {                //se spingi alto -> incrementa voltaggio
    t0 = millis();
    voltaggio = voltaggio + 0.1;
  }
  if ((digitalRead(A1) == 1) &&  (millis() - t0 > tde)) { //se spingi basso && debounce -> decrementa voltaggio
    t0 = millis();
    voltaggio = voltaggio - 0.1;
  }                                                                     //il parametro voltaggio diventa ciclico
  if (voltaggio > voltaggioMAX) {
    voltaggio = 0;
  }
  if (voltaggio < 0) {
    voltaggio = voltaggioMAX;
  }
  lcd.setCursor(4, 0);                                                  //riaggiorna di continuo la variabile temporale lasciando immutate le etichette scritte la prima volta che sei entrato in questo menù
  lcd.print(voltaggio);
  lcd.print(" kV");
  lcd.print("                ");
};
void S8() {
  if (first) {
    lcd.setCursor(0, 0);
    lcd.print("dV=    kV       ");
    lcd.setCursor(0, 1);
    lcd.write((byte)1);                                                 //freccia alto
    lcd.print("+ ");
    lcd.write((byte)0);
    lcd.print("-  >Hv  <SET");
    first = false;
  }
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde)) {                //se spingi bottone destra -> stato = 3
    t0 = millis();
    setStato(3);
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde) ) {               //se spingi bottone sinistra -> stato = 20
    t0 = millis();
    setStato(20);
  }
  if ((digitalRead(A0) == 1) && (millis() - t0 > tde)) {                //se spingi alto -> incrementa dV
    t0 = millis();
    dvoltaggio = dvoltaggio + 0.1;
  }
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde)) {                //se spingi basso -> decrementa dV
    t0 = millis();
    dvoltaggio = dvoltaggio - 0.1;
  }
  if (dvoltaggio <= 0) {                                                //non far diventare negativo dV
    dvoltaggio = 0;
  }
  lcd.setCursor(4, 0);
  lcd.print(dvoltaggio);
  lcd.print(" kV");
  lcd.print("                ");
};
void S4() {
  if (first) {
    lcd.setCursor(0, 0);
    lcd.print("Q=    mL/h");
    lcd.setCursor(0, 1);
    lcd.write((byte)1);                                                   //freccia alto
    lcd.print("+ ");
    lcd.write((byte)0);
    lcd.print("-  >dQ  <SET");
    first = false;
  }
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde)) {                  //se spingi bottone destra -> stato = 9
    t0 = millis();
    setStato(9);
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde) ) {                 //se spingi bottone sinistra -> stato = 30
    t0 = millis();
    setStato(30);
  }
  if ((digitalRead(A0) == 1) && (millis() - t0 > tde)) {                  //se spingi alto -> incrementa portata
    t0 = millis();
    portata = portata + 0.05;
  }
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde)) {                  //se spingi basso -> decrementa portata
    t0 = millis();
    portata = portata - 0.05;
  }
  if (portata < 0) {                                                      //non far diventare la portata negativa
    portata = 0;
  }
  lcd.setCursor(2, 0);
  lcd.print(portata);
  lcd.print("mL/h");
  lcd.print("                ");
};
void S9() {
  if (first) {                                                            //se è la prima volta che entri in questo menù scrivi le etichette
    lcd.setCursor(0, 0);
    lcd.print("dQ=   mL/h      ");
    lcd.setCursor(0, 1);
    lcd.write((byte)1);                                                   //freccia alto
    lcd.print("+ ");
    lcd.write((byte)0);
    lcd.print("-  >Q   <SET");
    first = false;
  }
  if ((digitalRead(A2) == 1) && (millis() - t0 > tde)) {                  //se spingi bottone destra -> stato = 4
    t0 = millis();
    setStato(4);
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde)) {                  //se spingi bottone sinistra -> stato = 30
    t0 = millis();
    setStato(30);
  }
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde) ) {                 //se spingi bottone basso -> decrementa dQ
    t0 = millis();
    dportata = dportata - 0.01;
  }
  if ((digitalRead(A0) == 1) && (millis() - t0 > tde) ) {                 //se spingi bottone alto -> incrementa dQ
    t0 = millis();
    dportata = dportata + 0.01;
  }
  if (dportata < 0) dportata = 0;                                         //non far andare dQ negativa
  lcd.setCursor(3, 0);                                                    //riaggiorna di continuo la variabile dQ lasciando immutate le etichette scritte la prima volta che sei entrato in questo menù
  lcd.print(dportata);
  lcd.print("mL/h");
  lcd.print("                ");
};
void S5() {
  if (first) {
    lcd.setCursor(0, 0);
    lcd.print("T=     C H=    %");
    lcd.setCursor(0, 1);
    lcd.print("<PAUSE     NEXT");
    lcd.write((byte)0);                                                   //freccia basso
    first = false;
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde)) {                  //se spingi bottone sinistra -> stato = 40
    t0 = millis();
    inizio = false;                                                       //azzera tag processo filatura
    pausa = true;                                                         //inizia la pausa
    tpausain = millis();                                                  //registra il tempo iniziale di pausa
    pompa(0);                                                             //SPEGNI POMPA
    dac.setVoltage(0, false);                                             //spegni alta tensione
    setStato(40);
  }
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde) ) {                 //se spingi bottone basso -> stato = 6
    t0 = millis();
    setStato(6);
  }
  lcd.setCursor(2, 0);
  lcd.print(temperatura);
  lcd.print("C H=");
  lcd.print(umidita);                                                     //scritto male di proposito NON CAMBIARE
  lcd.print(" %");
  lcd.print("                ");
};
void S6() {
  if (first) {
    lcd.setCursor(0, 0);
    lcd.print("Hv=   kV Q=     ");
    lcd.setCursor(0, 1);
    lcd.print("<PAUSE     NEXT");
    lcd.write((byte)0);                                                   //freccia basso
    first = false;
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde)) {                  //se spingi bottone sinistra -> stato = 40
    t0 = millis();
    inizio = false;                                                       //azzera tag processo filatura
    pausa = true;
    tpausain = millis();                                                  //registra il tempo iniziale di pausa
    pompa(0);                                                             //SPEGNI POMPA
    dac.setVoltage(0, false);                                             //spegni alta tensione
    setStato(40);
  }
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde) ) {                 //se spingi bottone basso -> stato = 7
    t0 = millis();
    setStato(7);
  }
  lcd.setCursor(2, 0);
  lcd.print(((n * dvoltaggio) + voltaggio));                              //voltaggio attuale tenendo conto dell'incremento della prova in svolgimento
  lcd.print("kV Q");
  lcd.print(((n * dportata) + portata));                                  //scritto male di proposito NON CAMBIARE
  lcd.print("                ");
};
void S7() {
  if (first) {
    lcd.setCursor(0, 0);
    lcd.print("N=      t=     s");
    lcd.setCursor(0, 1);
    lcd.print("<PAUSE     NEXT");
    lcd.write((byte)0);                                                   //freccia basso
    first = false;
  }
  if ((digitalRead(A3) == 1) && (millis() - t0 > tde)) {                  //se spingi bottone sinistra -> stato = 40
    t0 = millis();
    inizio = false;                                                       //pausa elettrofilatura
    pausa = true;                                                         //inizia la pausa
    tpausain = millis();                                                  //registra il tempo iniziale di pausa
    pompa(0);                                                             //SPEGNI POMPA
    dac.setVoltage(0, false);                                             //spegni alta tensione
    setStato(40);
  }
  if ((digitalRead(A1) == 1) && (millis() - t0 > tde) ) {                 //se spingi bottone basso -> stato = 5
    t0 = millis();
    setStato(5);
  }
  lcd.setCursor(3, 0);
  lcd.print(n);                                                           //n prova in svolgimento
  lcd.print("    t=");
  lcd.print(VARtempo);                                                    //tempo di elettrofilatura che avanza
  lcd.print(" s               ");
};
