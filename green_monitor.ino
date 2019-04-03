/*----------------------------------------------------------------------*
*                 Programme integre pour le Green Monitor               *
*                 Monitoring d'un mur vegetal d'interieur               *
*                                                                       *
*                                Alice AUBRY                            *
*                              Clément LANGLOIS                         *
*                             Marc-Alexis MAGAND                        *
*                                                                       *
*                            TS9-SI  annee 2014-2015                    *
*                                                                       *
*                                             Version 7.6 du 13-05-2015 *
*----------------------------------------------------------------------*/

//--------------  Parametres de reglage du GREEN MONITOR ----------------

int seuilHumid      =   500;   // Seuil d'humidite minimale qui
  // declenche
                               // l'arrosage. (0 sec, 1000 mouille)                                                  
int   seuilLum      =   150;   // Seuil de luminosite pour allumer 
                               // l'eclairage.(0 nuit, 1000 jour)
int      matin      =     8;   // Heure seuil de debut d'eclairage
int      soir       =    20;   // Heure seuil de fin d'eclairage
int      tempo      =   500;   // Attente de la boucle de mesure en ms
float limPhBasse    =  6.45;   // limite d'alarme pH basse
float limPhHaute    =  7.55;   // limite d'alarme pH haute
int  dureEcran      =  5000;   // Duree d'affichage des ecrans LCD en ms
long interDataLog   = 10000;   // Duree entre deux enregistrements sur SD 
                               // en ms(10s = 10 000, 15 min = 900 000 s)

unsigned int dureeArrosage = 7000;    // Duree de l'arrosage souhaitee en 
                                      // ms, par exemple 15000 pour 15 s
String nom  = "DATALOG.TXT";  // Nom du fichier de Log sur la carte SD

//------------ Bibliotheque I2C pour RTC, BMP085, Ecran LCD -------------

#include <Wire.h>         // Bibliotheque I2C

//--- Capteur BMP085 : pression atmospherique et temperature de l'air ---

#include <Adafruit_BMP085.h>     // Bibliotheque pour le capteur MBP0858
Adafruit_BMP085 bmp;             // Instanciation du capteur

// ------------------------- Ecran LCD sur bus I2C ----------------------

#include <LiquidCrystal_I2C.h>         // bibliotheque LCD I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);    // Adresse LCD I2C = 0x27 pour LCD     
                                       // et 4 lignes de 20 caracteres

boolean transition =   false;    // Flag de la transiton inter-ecran
long previousMillisEcran = 0;    // Memorise le temps ecoule depuis le
                                 // dernier changement Ecran

//-------------------- Capteur DHT22 humidite de l'air ------------------

#include "DHT.h"                 // bibliotheque DHT
#define pinDHT22            6    // Broche Data du DHT22 sur le Sensor   
                                 // shield
DHT dht;                         // Instanciation du capteur

float humidity;                  // humidite de l'air mesuree
long previousMillisDHT  =   0;   // Memorise le temps ecoule depuis le
                                 // dernier changement DHT

//-------------- DS18B20 temperature d'eau sur bus One Wire -------------

#include <OneWire.h>             // librairie pour capteur OneWire

int broche_OneWire = 38;         // Declaration de broche capteur

byte data  [12];       // Tableau de 12 octets pour lecture des 9
                       // registres de RAM et 3 registres d'EEPROM du
                       // capteur OneWire
byte adresse[8];       // Tableau de 8 octets pour stockage du code  
                       // d'adresse 64 bits du composant One Wire

int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
int compt               = 0;     // variable de comptage du nombre de 
                                 // capteurs sur le bus One Wire
byte present            = 0;
byte capteurlu          = 0;

float tempEau;                   // Temperature de l'eau

OneWire  capteur(broche_OneWire);    // Instanciation d'un objet One Wire 
                                     // capteur sur la broche voulue

//-------------------------- Humidite du feutre -------------------------

#define pinHumFeutre      A0    // broche de la sonde d'hmidite du feutre
int humFeutre;




//--------------- Photoresistance, mesure de la luminosité --------------

#define pinLum            A4          // broche de la photoresistance sur le Sensor Shield de la MEGA

#define eclairage_ON  digitalWrite(brocheEclairage, LOW );
#define eclairage_OFF digitalWrite(brocheEclairage, HIGH);

int brocheEclairage    =   8;         // Eclairage fil brun sur IN2 de la
                                      // platine relais
float Lum              =   0.;        // Luminosite lue par le capteur

//----------------------- Capteur de niveau d'eau ----------------------- 

int brocheNiveau       =   4;        // Broche de connexion du capteur de 
                                     // niveau, l'autre sur GND.
                                     // Resistance de pull-up obligatoire 
                                     // entre signal et VCC.
boolean etatLevel = false;           // Etat du capteur : false ouvert, 
                                     // true ferme.

//----------------------- Debimettre et arrosage ------------------------

volatile int NbTopsFan =  0 ;         // Mesure le front montant du signal
float volume           =  0.;         // Volume d'eau passee par le 
                                      // debitmetre pour un arrossage
float volumeCumule     =  0.;         // Volume cumule d'arrosage entre 2 enregistrements sur SD
int hallsensor         = 18 ;         // Broche 3 signal du capteur a effet 
                                      // hall (interruption 1 sur la MEGA                   
                                      // et la UNO).
long heureDebut        =  0 ;
long heureFin          =  0 ;
int brocheArrosage     = 11 ;        // Arrosage fil violet sur IN1 de la 
                                     // platine relais

#define arrosage_ON  digitalWrite(brocheArrosage, LOW );
#define arrosage_OFF digitalWrite(brocheArrosage, HIGH);

long previousMillisArr =    0;      // Memorise le temps ecoule depuis le 
                                    // dernier changement arrosage
boolean etatArrosage   =    0;

int coeffDebit         =  413;        // Coefficient de calcul du debit


void rpm () {                      // Fonction appelee par l'interruption
NbTopsFan++;                       // Cette variable volatile incremente   
                                   // le nombre de montees du signal du 
                                   // capteur a effet Hall du debitmetre
}

//----------------------- RTC horloge sur bus I2C -----------------------

#include "RTClib.h"
RTC_DS1307 rtc;

//----------------------------- Sonde pH --------------------------------

char  phData[20];                   // Tableau de 20 caracteres pour
                                    // recevoir les donnees venant du 
                                    // capteur pH
char  computerData[20];             // Tableau de 20 caracteres pour 
                                    // recevoir les donnees venant du 
                                    // moniteur serie (PC)
byte  receivedFromComputer =   0 ;  // Nombre de caracteres recus depuis 
                                    // l'ordinateur (du moniteur serie)
byte  receivedFromSensor  =    0 ;  // Nombre de caracteres recus du 
                                    // capteur pH
float ph                  =    0.;  // Valeur du pH en nombre flottant
long  previousMillisPH    =    0 ;  // Memorise le temps ecoule depuis le 
                                    // dernier changement DHT
int   interPH             =  2500;  // Fréquence de lecture du pH en ms

//----------------------------- Carte SD --------------------------------

#include <SD.h>
#include <SPI.h>

Sd2Card card;                       // Declaration des parametres de la 
                                    // bibliotheque SD

int chipSelect             =   53;  // Broche CS du lecteur de carte SD
String dataStringSD        =   "";  // Initialisation de la chaine (ligne 
                                    // d'enregistrements) a ecrire dans 
                                    // datalog.txt
long prevMillisSD          =    0;  // Reinisialise le temps de la SD
boolean ecritSD            = true;  // flag d'écriture fichier sur SD. 
                                    // true = OK, false = probleme
boolean SD_Card            = true;  // flag de presence d'une SD card. 
                                    // true = carte presente, false = 
                                    // carte absente

File dataFile;                      // Instanciation du fichier courant

//-----------------------------------------------------------------------

void setup() {

  Serial.begin (57600);               // Broches  0 et  1 COM 0 pour le
                                      // moniteur serie PC
  Serial3.begin(57600);               // Broches 14 et 15 COM 3 pour le
                                      // capteur pH

  Wire.begin();                       // Demarrage de l'I2C

// ------------ Dessin des caracteres speciaux pour l'ecran LCD ---------

  byte degre[8] = {
    0b00111,
    0b00101,
    0b00111,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
  };

  byte e_acc_aigu[8] = {
    0b00010,
    0b00100,
    0b01110,
    0b10001,
    0b11111,
    0b10000,
    0b01110,
    0b00000
  };

  byte e_acc_grav[8] = {
    0b01000,
    0b00100,
    0b01110,
    0b10001,
    0b11111,
    0b10000,
    0b01110,
    0b00000
  };

  byte c_cedil[8] = {
    0b00000,
    0b00000,
    0b01110,
    0b10000,
    0b10000,
    0b10001,
    0b01110,
    0b00100
  };

  byte copyright[8] = {
    0b01110,
    0b10001,
    0b10110,
    0b10100,
    0b10110,
    0b10001,
    0b01110,
    0b00000
  };

  byte coeur[8] = {
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b11111,
    0b01110,
    0b00100,
    0b00000
  };
  byte a_acc_circ[8] = {
    0b01110,
    0b10001,
    0b00110,
    0b00001,
    0b01111,
    0b10001,
    0b01111,
    0b00000
  };

//----------------------- Demarrage de l'ecran LCD ----------------------

lcd.init();                                // Initialise l'ecran LCD
lcd.backlight();                           // Allumage du retro-eclairage

// ---------- Création des caracteres speciaux, 8 maxi, de 0 à 7 --------
// NB : pour le caractere special "0", s'il n'est pas dans
// une variable, il faut le declarer en "cast"au format
// "byte", sinon ca provoque une erreur de compilation

  lcd.createChar(0, degre);
  lcd.createChar(1, e_acc_aigu);
  lcd.createChar(2, e_acc_grav);
  lcd.createChar(3, c_cedil);
  lcd.createChar(4, copyright);
  lcd.createChar(5, coeur);
  lcd.createChar(6, a_acc_circ);

  rtc.begin();                                    // Demarrage de la RTC

  if (!rtc.isrunning() || !bmp.begin()) {         // Alarmes cablages RTC 
                                                  // et du capteur BMP085

    Serial.print( " Attention, RTC ou BMP085 pas ou mal c");
    Serial.write(226) ;
    Serial.print("bl");
    Serial.write(233) ;

    lcd.setCursor(5, 0)   ;
    lcd.print("Attention");
    lcd.setCursor(0, 1)   ;
    lcd.print("Un capteur n'est pas");
    lcd.setCursor(1, 2);
    lcd.print("c")     ;
    lcd.write(6) ;
    lcd.print("bl");
    lcd.write(1);
    lcd.print(" ou mal c");
    lcd.write(6);
    lcd.print("bl");
    lcd.write(1);
    lcd.setCursor(1, 3);
    lcd.print("  RTC ou BMP085");
    for (int i = 0 ; i < 10 ; ++i) {         // Clignotement de Attention
      lcd.setCursor(5 , 0);
      lcd.print("          ");
      delay(300);
      lcd.setCursor(5, 0);
      lcd.print("Attention");
      delay(300);
    }
    while (1) {}                             // Boucle d'attente infinie
  }                                          // tant qu'on n'a pas repare

//---------- détection des capteurs présents sur le bus One Wire --------

  while (capteur.search(adresse) == true) {         
    // Tant qu'un nouveau capteur est détecté, la fonction search renvoie
    // true
    // si un élément 1-wire est trouvé. Stocke son adresse dans le
    // tableau adresse
    // a l'adresse de début du tableau adresse[8] déclare
    // Ce code est execute pour chaque capteur détecte
    compt = compt + 1;                      // incremente la variable de
                                             // comptage du nombre de compteurs

//--------- contrôle du CRC pour le capteur DS18B20 ---------------------
// Le dernier octet de l'adresse 64bits est un code de contrôle CRC
// A l'aide de la fonction crc8 on peut vérifier si ce code est valide
// Le CRC de l'adresse 64 bits est le 8ème octet de l'adresse (index 7 du tableau)

    if (capteur.crc8( adresse, 7) != adresse[7]) { 
      // Verification validite code CRC de l'adresse 64 bits
      Serial.println ("CRC adresse 64 bits du capteur NON VALIDE");
      lcd.setCursor(0, 1);
      lcd.print("capteur temp");
      lcd.write(1);
      lcd.print("rature");
      lcd.setCursor(0, 2);
      lcd.print("d'eau : adresse");
      lcd.setCursor(0 , 3);
      lcd.print("non trouv");
      lcd.write(1);
      lcd.print("e");
      lcd.setCursor(0, 0);
      lcd.blink();
      lcd.print("ATTENTION");
      while (1) {}
    }
  }                             // Fin boucle while test presence capteur

  if (compt == 0) {             // Si aucun capteur n'a été detecte
    Serial.println("Aucun capteur present sur le bus 1-wire");
    lcd.setCursor(0, 1);
    lcd.print("capteur temp");
    lcd.write(1);
    lcd.print("rature");
    lcd.setCursor(0, 2);
    lcd.print("d'eau non trouv");
    lcd.write(1);
    lcd.setCursor(0, 0);
    lcd.blink();
    lcd.print("ATTENTION");
    while (1) {}
  }
//-----------------------------------------------------------------------

  dht.setup(pinDHT22);                   // Demarrage du DHT22

  pinMode (brocheEclairage,  OUTPUT);
  eclairage_OFF;                         // Eclairage eteint au demarrage

  pinMode (brocheArrosage,   OUTPUT);
  arrosage_OFF;                          // Arrosage arrete au demarrage

  pinMode (brocheNiveau  ,    INPUT);    // La broche "lit" l'etat de
                                         // l'interrupteur, donc c'est
                                         // une entree

  pinMode(hallsensor,         INPUT);    // Broche du capteur en entree
                                         // pinMode
  attachInterrupt(5 , rpm,   RISING);    // on lie la fonction rpm a
    // l'interruption "5" et on compte uniquement sur les fronts montants
    // (broche 18 = interruption 5 sur la MEGA)

//------------------- Carte SD ------------------------------------------
  pinMode(chipSelect, OUTPUT);           // Obligatoire de declarer la
                                         // broche CS en sortie

Serial.println("------------------------------------------------------");
  Serial.println("Initialisation de la carte MicroSD ...");

  if (!SD.begin(chipSelect)) {

    Serial.print("Lecteur SD mal c");   // Message d'erreur SD sur
    Serial.write(226);                   // moniteur serie
    Serial.print( "bl");
    Serial.write(233);
    Serial.print(", carte d");
    Serial.write(233);
    Serial.print("fectueuse, ou non pr");
    Serial.write(233);
    Serial.println("sente");

    lcd.setCursor(0, 0);                 // Message d'erreur SD sur LCD
    lcd.print("Lecteur SD mal c");
    lcd.write(6);
    lcd.print("bl");
    lcd.write(1);
    lcd.setCursor(0, 1);
    lcd.print("Carte d");
    lcd.write(1);
    lcd.print("fectueuse");
    lcd.setCursor(0, 2);
    lcd.print("ou absente");

    while (1) {}                         // boucle d'attente infinie tant  }                                        // qu'on n'a pas repare

  else {
    Serial.print("carte initialis");
    Serial.write(233);
    Serial.println("e");
  }

  Serial.println("------------------------------------------------------");

//-------------- Affichage de l'ecran d'accueil sur LCD -----------------
  lcd.setCursor(1, 0);
  lcd.print("Bonjour");
  lcd.setCursor(1, 1);
  lcd.print("Bienvenue sur le ");
  lcd.setCursor(1, 2);
  lcd.print("Green Monitor");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print ("Realis");
  lcd.write(1);
  lcd.print (" par :");
  lcd.setCursor(0, 1);
  lcd.print("Alice       AUBRY");
  lcd.setCursor(0, 2);
  lcd.print("Cl");
  lcd.write(1);
  lcd.print("ment     LANGLOIS");
  lcd.setCursor(0, 3);
  lcd.print("Marc-Alexis MAGAND");
  delay(3000);
  lcd.clear();

  previousMillisEcran = millis();
  prevMillisSD        = millis();
  previousMillisArr   = millis();
  previousMillisPH    = millis() - 2500;
  previousMillisDHT   = millis() + 2000;           // Pour avoir une lecture du DHT22 dès le début
}

//-----------------------------------------------------------------------

void loop() {

  unsigned long currentMillis = millis();  // stocke la valeur de
                                           // l'instant courant de debut
                                           // de boucle


//-- Lecture du BMP085 - Pression atmospherique et temperature de l'air -
  float tempAir         = bmp.readTemperature();
  float pressionAtm     = bmp.readPressure() / 100.;    // en hPa

//-------- Lecture du DHT22 - Humidite de l'air -------------------------
// Ce capteur necessite au minimum 2 secondes entre 2 lectures
// Utilisation de la fonction millis() pour realiser une lecture non
// bloquante toute les 2 secondes

  if (currentMillis - previousMillisDHT > 2000) {
    humidity = dht.getHumidity();      // lecture de l'humidite
 //seulement toute les 2
                                          // secondes, non bloquante
    previousMillisDHT = currentMillis;    // reinitialise l'instant de
  }                                       // debut

//------- Lecture du DS18B20 - Temperature de l'eau ---------------------
  byte i;

  if ( !capteur.search(adresse)) {         // reinitialise la recherche
    capteur.reset_search();       // quand tous les capteurs ont
    capteurlu = 0;          // été lus
    return;
  }

  capteurlu = capteurlu + 1;               // incremente la variable de
  // comptage du nombre de
  // capteurs

  if (capteurlu > 0 ) {
    capteur.reset();
    capteur.select(adresse);
    capteur.write(0x44, 1);                // demarre la conversion, avec
  // l'alimentation fantome ON a
  // la fin

    present = capteur.reset();
    capteur.select(adresse);
    capteur.write(0xBE);                   // lit le Scratchpad

    for ( i = 0; i < 9; i++) {             // On veut 9 octets
      data[i] = capteur.read();
    }

    LowByte = data[0];
    HighByte = data[1];
    TReading = (HighByte << 8) + LowByte;
    SignBit = TReading & 0x8000;           // teste le bit de signe

    if (SignBit) {                         // negatif
      TReading = (TReading ^ 0xffff) + 1;  // 2's comp
    }

    Tc_100 = (6 * TReading) + TReading / 4;  // multiplie par (100 *
    // 0.0625) or 6.25
    Whole = Tc_100 / 100;                    // Separe partie entiere et
    Fract = Tc_100 % 100;         // fractionnaire

// On ne traite pas les temperatures negatives, ce qui ne derait jamais
// arriver sur un mur vegetal d'interieur
    tempEau = Whole + ((float)Fract / 100);
  }

//------- Lecture de l'humidite du feutre -------------------------------
  int humFeutreValue = analogRead(pinHumFeutre); // lecture du capteur
                                                 // d'humidite du feutre
  humFeutre = map(humFeutreValue, 0, 1023, 0, 1000);     // "Remappe" les
                                // valeur de 0 à 1000 au lieu de 0 à 1023
  humFeutre  = 1000 - humFeutre;                    // inverse uniquement
                                        // pour test sans humidite reelle

//-------- Lecture de la luminosite -------------------------------------
  int Lum = analogRead(pinLum);       // lecture du capteur de luminosite
  Lum = map(Lum, 0, 1023, 0, 1000);   // "Remappe" les valeur de 0 à 1000
                                      // au lieu de 0 à 1023

//------- Lecture du niveau d'eau ---------------------------------------
  etatLevel = digitalRead(brocheNiveau);  // lecture du capteur de niveau

//-------- Lecture du pH ------------------------------------------------
  if (currentMillis - previousMillisPH > interPH ) {
    Serial3.print(tempEau);             // Envoi de la commande de
    Serial3.print("\r");                // correction du pH avec la
                                        // temperature
    Serial3.print("R\r");               // Envoi de la commande pour une 
                                        // seule lecture du pH

    if (Serial3.available() > 0) {      // Si on voit que le circuit pH a 
                                        // envoye un caractere
      receivedFromSensor = Serial3.readBytesUntil(13, phData, 20); 
          // On lit les donnes envoyees par le circuit pH jusqu'a
    // recevoir un <CR> (code ASCII 13). On compte aussi le nombre
    // de caracteres recus.
    // On charge les caracteres recus dans le tableau phData
      phData[receivedFromSensor] = 0; // On ajoute un "0" dans le tableau 
                                      // juste apres le dernier caractere 
                                      // recu. Cela stoppe la
                                      // transmission de donnees
                                      // incorrectes qui pourraient
                                      // rester dans le buffer
      ph = atof(phData);              // Conversion de la chaine de
    }                                 // caracteres ASCII en flottant

    for (int i = 0 ; i < 20 ; ++i) {  // remise a  0 du buffer reception
      computerData[i] = 0;
    }

    for (int i = 0 ; i < 20 ; ++i) {   // remise a  0 du buffer emission
      phData[i] = 0;
    }

    receivedFromComputer = 0;          // Remise a  0 des compteurs
    receivedFromSensor   = 0;
    previousMillisPH = currentMillis;  // reinitialise l'instant de debut
  }
//-------- Verification de la carte SD ----------------------------------
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    SD_Card = false;                    // Carte SD absente, passage du
  }                                     // flag de presence SD a false
  else {
    SD_Card = true;                     // Carte SD presente, passage du
  }                                     // flag de presence SD a true


//-------  TRAITEMENTS --------------------------------------------------

// Declenchement de l'eclairage en fonction de la luminosite et de
// l' heure

  DateTime heure = rtc.now();

  if ( Lum < seuilLum && heure.hour() >= matin && heure.hour() <= soir) {
    // Si luminosite inferieure au
    eclairage_ON;                      // seuil, on allume
          // mais seulement dans la plage de
    // jour. La nuit, on reste eteint
  }
  else {                               // Sinon, on eteint
    eclairage_OFF
  }

  boolean etatEclairage = !digitalRead(brocheEclairage);
    // contraire (!) car l'eclairage
    // est ON avec la broche LOW

//- Declenchement de l'arrosage en dessous du seuil d'humidite du feutre

  if (humFeutre <= seuilHumid) {      // Si la sonde est seche, on arrose
    arrosage_ON;
    etatArrosage = 1;
    previousMillisArr = currentMillis; // reinitialise l'instant de debut
                                       // de l'arrosage
  }

  else {  // Sinon, on verifie si le delai est ecoule depuis le dernier
          // changement d'etat de l'arrosage
          // C'est le cas si la difference entre la valeur courante
          // renvoyee par millis() et la valeur de la variable du temps
          // du dernier changement d'etat de l'arrosage est superieure a
          // la valeur de l'intervalle d'arrosage.

    if (currentMillis - previousMillisArr > dureeArrosage && etatArrosage == 1) {
      arrosage_OFF; // on arrete d'arroser au bout de la duree d'arrosage
      volume = (float) NbTopsFan / (float) coeffDebit;
      volumeCumule += volume;        // Pour cumuler le volume
                                     // d'arrosage entre 2
                                     // enregistrements de donnees sur SD

      Serial.println("-----------------------------------");
      Serial.print("Volume arros");
      Serial.write(233);
      Serial.print("   =    ");
      Serial.print(volume, 2);
      Serial.println("  litres");
      Serial.println("-----------------------------------");

      etatArrosage = 0;
      NbTopsFan    = 0;               // reinitialise le debitmetre
    }
  }

//-------- Ecriture des donnees sur la carte SD -------------------------

  if (currentMillis - prevMillisSD > interDataLog ) {
  // Programmation non bloquante

// SD.open n'accepte qu'un tableau de Char comme nom de fichier, pas une 
// String. Il faut convertir
    char nomFichier [nom.length() + 1];
                   
// Initialisation du tableau de caracteres de la taille de la String nom

    nom.toCharArray(nomFichier, sizeof(nomFichier));      
// Transformation de la String en tableau de caracteres

    DateTime JMAHM = rtc.now();
// Extraction de la date et de l'heure sur la RTC

//-------- Preparation des donnees a ecrire -----------------------------
// On construit une longue chaine de caracteres dataString pour assembler les donnees a enregistrer sur une seule ligne

//---------------- Date et heure ----------------------------------------

    if ( JMAHM.day() < 10 ) {         // Ajout d'un "zero" si le jour est  
      dataStringSD += "0";            // inferieur a 10 pour avoir des
    }                                 // chaines de dates alignees
    dataStringSD += JMAHM.day();
    dataStringSD += "/";              // Separateur jour-mois "/"

    if ( JMAHM.month() < 10 ) {       // Ajout d'un "zero" si le mois est dataStringSD += "0";                  // inferieur a 10 pour avoir
    }                                 // des chaines de dates alignees
    dataStringSD += JMAHM.month();
    dataStringSD += "/";              // Separateur mois-annee "/"

    dataStringSD += JMAHM.year();
    dataStringSD += " ";

    if ( JMAHM.hour() < 10 ) {        // Ajout de "zero" si les heures
                                      // sont inferieures a 10
      dataStringSD += "0";            // pour avoir des chaines de dates
    }                                 // alignees
    dataStringSD += JMAHM.hour();
    dataStringSD += ":";              // Separateur heures - minutes ":"
    if ( JMAHM.minute() < 10 ) {      // Ajout de "zero" si les minutes
                                      // sont inferieures a 10 pour avoir
      dataStringSD += "0";            // des chaines de dates alignees
    }
    dataStringSD += JMAHM.minute();
    dataStringSD += ",";              // Separateur de champ (virgule)

//---------------- Temperature de l'air ---------------------------------
    dataStringSD += tempAir;
    dataStringSD += ",";

//---------------- Pression atmospherique -------------------------------
    dataStringSD += pressionAtm;
    dataStringSD += ",";

//---------------- Humidite de l'air ------------------------------------
    dataStringSD += humidity;
    dataStringSD += ",";
//---------------- Luminosite -------------------------------------------
    dataStringSD += Lum;
    dataStringSD += ",";

//---------------- Temperature de l'eau ---------------------------------
    dataStringSD += tempEau;
    dataStringSD += ",";

//----------------- pH --------------------------------------------------
    dataStringSD += ph;
    dataStringSD += ",";

// ---------------- Humidite du feutre ----------------------------------
    dataStringSD += humFeutre;
    dataStringSD += ",";

//----------------- Niveau d'eau ----------------------------------------
    if (etatLevel == true) {
      dataStringSD += "OK";
      dataStringSD += ",";
    }
    else {
      dataStringSD += "BAS";
      dataStringSD += ",";
    }

//----------------- Volume d'arrosage -----------------------------------
    dataStringSD += volumeCumule;

    volumeCumule = 0.;   // Reinitialisation du volume cumule. Pret pour
                         // un nouvel enregistrement

//------------------ Ouverture du fichier -------------------------------
// Un seul fichier peut etre ouvert a la fois, il faudra fermer celui-la
// avant d'en ouvrir un autre
    if (SD_Card = false) {   // Verification de la presence d'une carte
                             // SD avant d'ecrire
      dataStringSD = "";     // On ne veut plus rien dans la chaine si le
                             // lecteur de SD card ne marche pas
      Serial.println("");
      Serial.println("------------------------------------------------------------------");
      Serial.println("Carte SD absente, pas d'enregistrement");
      Serial.println("------------------------------------------------------------------");
      Serial.println("");
    }

    else {          // Carte SD presente, on peut ecrire l'enregistrement
      File dataFile = SD.open( nomFichier , FILE_WRITE );
      if (dataFile) {            // Si tout se passe bien avec le fichier
        ecritSD = true;
        dataFile.println(dataStringSD);     // Ecriture de dataStringSD a
                                            // la fin du fichier

        Serial.println("");
        Serial.println("--------- Ecriture d'un enregistrement sur carte SD ----------");
        Serial.println(dataStringSD);       // Ecriture egalement sur le
                                            // moniteur serie
        Serial.println("--------------------------------------------------------------");
        Serial.println("");

        dataFile.close();    // Fermeture du fichier
        SD_Card = true;      // Flag force, sinon, il est a false a
      }                      // la boucle suivant. Apres, il redevient OK

      else {                 // S'il y a un probleme avec le fichier
        ecritSD = false;

        Serial.println("");
        Serial.println("----------------------------------------");
        Serial.println("Echec d'ouverture du fichier : "); 
    // Si le fichier n'a pas pu etre ouvert, afficher le message d'erreur
        Serial.println(nom);
        Serial.println("----------------------------------------");
        Serial.println("");
        Serial.println("");
      }
    }

    prevMillisSD = currentMillis;  // Reinitialisation du temps SD
    dataStringSD = "";             // Reinitialisation de la String.
  }                                // Prete pour un nouvel enregistrement

//--------- Affichage des donnees sur le moniteur serie -----------------

//--------- Temperature d'air et pression atmospherique -----------------
  Serial.print("Temp");
  Serial.write(233);
  Serial.print("rature air =   ");
  Serial.print(tempAir, 1);             // Avec une seule decimale
  Serial.print("   ");
  Serial.write(176);
  Serial.println("C");
  Serial.print("Pression atm    = ");
  Serial.print(pressionAtm , 2);        // en hPa
  Serial.println(" hPa");

//--------- Humidite de l'air -------------------------------------------
  Serial.print("Humidit");
  Serial.write(233);
  Serial.print(" air    =   ");
  Serial.print(humidity, 1);
  Serial.println("    %");

//--------- Luminosite et eclairage -------------------------------------
  Serial.print("Luminosit");
  Serial.write(233);
  Serial.print("      =  ");
  Serial.print(Lum);

  if (Lum >= seuilLum ) {
    Serial.println( "  (OK)" );
  }
  else {
    Serial.println( " (SOMBRE)");
  }

  Serial.print("Eclairage       =  ");

  if ( etatEclairage == true) {            // Si etatEclairage est true, 
    Serial.println(" ON ");                // on affiche ON
  }
  else {                                   // Sinon on affiche OFF
    Serial.println("OFF");
  }

//--------- Temperature de l'eau ----------------------------------------
  Serial.print("Temp");
  Serial.write(233);
  Serial.print("rature eau =   ");
  Serial.print(tempEau , 1);               // Une seule decimale
  Serial.print("   ");                     // (chiffres significatifs)
  Serial.write(176);                       // Caractere ASCII "degre"
  Serial.println("C");

//-------- pH -----------------------------------------------------------
  Serial.print("pH              =    ");
  Serial.print( ph , 2);

  if ( ph > limPhHaute ) {
    Serial.println( " (pH trop haut)" );
  }
  else if ( ph < limPhBasse ) {
    Serial.println( " (pH trop bas)" );
  }
  else {
    Serial.println( " (pH OK)");
  }

//--------- Humidite du feutre ------------------------------------------
  Serial.print("Humidit");
  Serial.write(233);
  Serial.print(" feutre = ");
  Serial.print(humFeutre);               // 0 sec, 1000 mouille
  if (humFeutre <= seuilHumid) {         // Si la valeur est inferieure
    Serial.println(" (Sec)");            // au seuil d'humidite
  }
  else {
    Serial.println(" (Humide)");
  }

  Serial.print("Arrosage        =  ");

  if ( etatArrosage == true) {           // Si etatArrosage est true, on
    Serial.println(" ON ");              // affiche ON
  }
  else {                                 // Sinon on affiche OFF
    Serial.println("OFF");
  }

//-------- Niveau d'eau -------------------------------------------------
  Serial.print("Niveau d'eau    = ");

  if (etatLevel == true ) {
    Serial.println( "  OK" );
  }
  else {
    Serial.println( " BAS");
  }

//-------- Carte SD -----------------------------------------------------
  Serial.print("Carte SD        = ");

  if ( SD_Card == false) {                   // Carte SD absente
    Serial.println(" Absente, pas d'enregistrement");
  }
  else {                                     // Carte SD presente
    Serial.println("  OK");
  }

  Serial.println("-----------------------------------");

//--------- Fin des affichages sur le moniteur serie --------------------


//--------- Affichage des donnees sur l'ecran #1 du LCD -----------------
  if (currentMillis - previousMillisEcran <= dureEcran) {   
                                           // Programmation non bloquante

    if (transition == 1 ) {                // gestion du scrolling entre
                                           // les differents ecrans
      for (int i = 0 ; i < 20 ; i++) {
        lcd.scrollDisplayRight();
        delay(25);
      }

      transition = 0;
      lcd.clear();
    }

    lcd.setCursor(0, 0);
    lcd.print("Temp air   : ");
    lcd.print(tempAir, 1);
    lcd.setCursor(18, 0);
    lcd.write(0);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Hum air    : ");
    lcd.print(humidity, 0);
    lcd.print("  ");
    lcd.setCursor(19, 1);
    lcd.print("%");
    lcd.setCursor(0, 2);
    lcd.print("Pression   :");
    lcd.print(pressionAtm, 0);
    lcd.setCursor(17, 2);
    lcd.print("hPa");
    lcd.setCursor(0, 3);
    lcd.print("Luminosit");
    lcd.write(1);
    lcd.print(" : ");

    if (Lum <= seuilLum ) {
      lcd.print("SOMBRE");
    }
    else {
      lcd.print("OK    ");
    }
  }

//--------- Affichage des donnees sur l'ecran #2 du LCD -----------------
  if (currentMillis - previousMillisEcran > dureEcran && currentMillis - previousMillisEcran <= 2 * dureEcran ) {

    if (transition == 0 ) {

      for (int i = 0 ; i < 20 ; i++) {            // transition scrolling
        lcd.scrollDisplayRight();                 // entre ecrans
        delay(25);
      }

      transition = 1;
      lcd.clear();
    }

    lcd.setCursor(0, 0);
    lcd.print("Temp eau   : ");
    lcd.print(tempEau, 1);
    lcd.setCursor(18, 0);
    lcd.write(0);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Hum feutre : ");

    if (humFeutre <= seuilHumid ) {
      lcd.print("Sec");
    }
    else {
      lcd.print("OK ");
    }

    lcd.setCursor(0, 2);
    lcd.print("pH         : ");
    lcd.print(ph, 1);
    lcd.setCursor(0, 3);
    lcd.print("Niveau eau : ");

    if (etatLevel == true ) {
      lcd.print("OK ");
    }

    else {
      lcd.print("BAS");
    }
  }

//--------- Affichage des alertes sur l'ecran #3 du LCD -----------------
  if (currentMillis - previousMillisEcran > 2 * dureEcran && currentMillis - previousMillisEcran <= 3 * dureEcran ) {
    if (transition == 1 ) {

      for (int i = 0 ; i < 20 ; i++) {            // transition scrolling
        lcd.scrollDisplayRight();                 // entre ecrans
        delay(25);
      }

      transition = 0;
      lcd.clear();
    }

    lcd.setCursor(7, 0);
    lcd.print("Alerte");

    if ( etatLevel == false ) {                   // Alarme niveau d'eau
      lcd.setCursor(1, 1);
      lcd.print(" Niveau d'eau BAS");
    }

    else {
      lcd.setCursor(0, 1);
      lcd.print("                    ");
    }

    if ( ph < limPhBasse ) {                      // Alarme pH faible
      lcd.setCursor(3, 2);
      lcd.print("pH trop FAIBLE");
    }

    if ( ph > limPhHaute ) {                      // Alarme pH haut
      lcd.setCursor(3, 2);
      lcd.print("pH trop HAUT  ");
    }

    if ( SD_Card == false) {                      // Alarme carte SD absente
      lcd.setCursor(0, 3);
      lcd.print("SD absente : REBOOT ");
      while (1) {}      // blocage du programme en cas de carte manquante
    }
    else {
      lcd.setCursor(0, 3);
      lcd.print("                    ");
    }

    if ( etatLevel == 1 && ph < limPhHaute && ph > limPhBasse && SD_Card == true ) {
      lcd.setCursor(0, 0);
      lcd.print("                    ");          // Aucune alarme
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.write(5);
      lcd.print(" Aucune alerte ");
      lcd.write(5);
      lcd.print("  ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
    }
  }

  if (currentMillis - previousMillisEcran > 3 * dureEcran ) {
    previousMillisEcran = currentMillis;        // reinitialise l'instant
    transition = 1;                             // de debut
  }
//--------- Fin des affichages sur l'ecran LCD --------------------------

  delay(tempo);                       // Boucle d'attente entre 2 mesures
}
//-----------------------------------------------------------------------

