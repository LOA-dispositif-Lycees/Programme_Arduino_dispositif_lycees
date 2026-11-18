#include <Arduino.h>
#include <Wire.h>
#include "SensirionI2cSps30.h"
#include "SensirionI2cScd30.h"
#include "DFRobot_GNSSAndRTC.h"
#include <SPI.h>
#include "SdFat.h"
#include <SoftwareSerial.h>
#include "DFRobot_BT401.h"

// ==========================================
// Constantes
// ==========================================

#define NO_ERROR 0
SensirionI2cSps30 sensorSPS;
SensirionI2cScd30 sensorSCD; 

SdFat SD;
SdFile myFile;
SdFile StockageFile;

SoftwareSerial gpsRTC(5,6);  
DFRobot_GNSSAndRTC_UART gnssRtc(&gpsRTC, UART_BAUDRATE);

SoftwareSerial btSerial(2,3);
DFRobot_BT401 bt;

// ==========================================
// Fonction écriture des données
// ==========================================

// Cette fonction écrit les données sur n'importe quel support (SD, Serial, Bluetooth)
void writeSensorData(Print &output, const char* ts, DFRobot_GNSSAndRTC::sTimeData_t &Date, DFRobot_GNSSAndRTC::sLonLat_t &lat, DFRobot_GNSSAndRTC::sLonLat_t &lon, float temp, float hum, float co2, uint16_t mc1, uint16_t mc2, uint16_t mc4, uint16_t mc10, uint16_t nc05, uint16_t nc1, uint16_t nc2, uint16_t nc4, uint16_t nc10, bool t = false,  bool isCsvFile = false) {
  
// Format d'affichage selon le support (CSV fichier avec _ et : ou flux continu avec des virgules)
  if (t){
    output.print(ts);
    output.print(',');
  }
  
  output.print(F("XXX")); // ID de l'établissement à changer avec les 2 premières lettres de l'établissement et le code I/E en 1 caractère
  output.print(',');
  
  output.print(Date.year);
  output.print(isCsvFile ? '_' : ',');
  output.print(Date.month);
  output.print(isCsvFile ? '_' : ',');
  output.print(Date.day);
  output.print(',');
  
  output.print(Date.hour);
  output.print(isCsvFile ? ':' : ',');
  output.print(Date.minute);
  output.print(isCsvFile ? ':' : ',');
  output.print(Date.second);
  output.print(',');
  
  output.print(lat.latitudeDegree, 5);
  output.print(',');
  output.print(lon.lonitudeDegree, 5);
  output.print(',');
  output.print(temp, 2);
  output.print(',');
  output.print(hum, 2);
  output.print(',');
  output.print(co2, 2);
  output.print(',');
  output.print(mc1);
  output.print(',');
  output.print(mc2);
  output.print(',');
  output.print(mc4);
  output.print(',');
  output.print(mc10);
  output.print(','); 
  output.print(nc05);
  output.print(',');
  output.print(nc1);
  output.print(',');
  output.print(nc2);
  output.print(',');
  output.print(nc4);
  output.print(',');
  
  if (isCsvFile) {
    output.println(nc10);
  } else {
    output.println(nc10); // ou print selon si on veut un retour à la ligne
  }
}

// ==========================================
// Setup
// ==========================================

void setup() {  
  Serial.begin(9600);    
  Wire.begin();

  // SPS30
  sensorSPS.begin(Wire, SPS30_I2C_ADDR_69); 
  while(sensorSPS.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_UINT16) != NO_ERROR){
    delay(1000);
  }
  delay(100); 

  // SCD30
  sensorSCD.begin(Wire, SCD30_I2C_ADDR_61);
  while(sensorSCD.startPeriodicMeasurement(0) != NO_ERROR){
    delay(1000);  
  }
  delay(100); 

 // GNSS & RTC
  while (!gnssRtc.begin()) {
    delay(1000);
  }
  delay(100);
  gnssRtc.setHourSystem(gnssRtc.e24hours);
  gnssRtc.calibRTC(); // calibration de la RTC 1 seule fois au démarrage ; bien s'assurer d'être en extérieur ou proche des fenêtres pour la calibration
  delay(100); 
  
  // SD
  pinMode(10, OUTPUT);
  while(!SD.begin(4, SPI_HALF_SPEED)) { // initialise le bus SPI à une vitesse spécifique
    delay(1000);  
  }
  delay(100); 

  // Bluetooth
  // Le Bluetooth démarre par défaut à 115200 
  // Il faut le démmarer à 115200 pour changer sa vitesse de transmission à 9600
  btSerial.begin(115200); // vitesse de transmission par défaut
  delay(1000); 
  while(!bt.begin(btSerial)){
    delay(1000);  
  }
  btSerial.println("AT+CT01"); // commande pour modifier la vitesse de communication à 9600
  delay(500);
  // Redémarrage du port série à 9600 bauds
  btSerial.end();
  delay(100);
  btSerial.begin(9600);
  delay(500);
  while(!bt.begin(btSerial)){
    delay(1000);  
  }
}

// ==========================================
// Loop
// ==========================================

void loop() {
  gpsRTC.listen(); // 2 périphériques sur SoftwareSerial : on demande d'écouter le GNSS & RTC  
  delay(100); // on lui laisse le temps de recevoir les données

  static unsigned long mesureTimer = millis();
  
  if (millis() - mesureTimer >= 60000){ // 1 mesure par min
    mesureTimer = millis();

    // Lecture SPS30
    uint16_t mc1p0 = 0, mc2p5 = 0, mc4p0 = 0, mc10p0 = 0;
    uint16_t nc0p5 = 0, nc1p0 = 0, nc2p5 = 0, nc4p0 = 0, nc10p0 = 0;
    uint16_t typicalParticleSize = 0;
    sensorSPS.readMeasurementValuesUint16(mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, typicalParticleSize);

    // Lecture SCD30
    float co2Concentration = 0, temperature = 0, humidity = 0;
    sensorSCD.readMeasurementData(co2Concentration, temperature, humidity);

    // Lecture GNSS & RTC
    DFRobot_GNSSAndRTC::sTimeData_t Date = gnssRtc.getRTCTime();
    DFRobot_GNSSAndRTC::sLonLat_t lat = gnssRtc.getLat();
    DFRobot_GNSSAndRTC::sLonLat_t lon = gnssRtc.getLon();

    // 1. Écriture sur la carte SD du jour
    char filename[20]; 
    snprintf(filename, sizeof(filename), "%02d_%02d_%02d.csv", Date.year%100, Date.month, Date.day); // 1 fichier par jour dont le nom  est au format AA_MM_JJ.csv
    bool fileExists = SD.exists(filename);

    if(myFile.open(filename, O_RDWR | O_CREAT | O_AT_END)){
      if (!fileExists) { // si le fichier n'existe pas encore, on commence par lui mettre une entête sinon on continue d'écrire sur le fichier sans réécrire l'entête
        myFile.println(F("ID,Date,Heure_UTC,Latitude,Longitude,Temperature,Humidite,CO2,PM1,PM2.5,PM4,PM10,NPM0.5,NPM1,NPM2.5,NPM4,NPM10")); // entête
      }
      writeSensorData(myFile, "", Date, lat, lon, temperature, humidity, co2Concentration, mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, false, true);
      myFile.close();
    } 
    
    // 2. Envoi des données
    btSerial.listen(); // 2 périphériques sur SoftwareSerial : on demande d'écouter le bluetooth 
    delay (1000); // on lui laisse le temps de recevoir les données

    // Buffer unique
    char rxBuffer[32];

    // 2.1 via le port série
    if(Serial.available() > 0) { // si on reçoit une donnée sur le port série
      int len = Serial.readBytesUntil('\n', rxBuffer, sizeof(rxBuffer) - 1); // lire la donnée reçue 
      rxBuffer[len] = '\0'; 
      if(rxBuffer[0] == 'C'){ // si le premier caractère reçu correspond à C
        if (StockageFile.open("Off.csv", O_READ)) { // d'abord commencer par vérifier si un fichier "Off.csv" existe
          while (StockageFile.available()) { // tant que le fichier existe 
            Serial.write(StockageFile.read()); // le lire puis le supprimer
          }
          StockageFile.close();
          SD.remove("Off.csv");
        } // puis affichage temps réel des données avec &rxBuffer[1] le deuxième caractère reçu qui correspond à un timestamp
        writeSensorData(Serial, &rxBuffer[1], Date, lat, lon, temperature, humidity, co2Concentration, mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, true, false); 
      }
    }   
    
    // 2.2 via bluetooth 
    else if(btSerial.available() > 0) { // sinon, si on reçoit une donnée sur le Bluetooth
      int len = btSerial.readBytesUntil('\n', rxBuffer, sizeof(rxBuffer) - 1); // lire la donnée reçue 
      rxBuffer[len] = '\0';
      if(rxBuffer[0] == 'C'){ // si le premier caractère reçu correspond à C
        if (StockageFile.open("Off.csv", O_READ)) { // d'abord commencer par vérifier si un fichier "Off.csv" existe
          while (StockageFile.available()) { // tant que le fichier existe 
            btSerial.write(StockageFile.read()); // le lire puis le supprimer
          }
          StockageFile.close();
          SD.remove("Off.csv");
        } // puis affichage temps réel des données avec &rxBuffer[1] le deuxième caractère reçu qui correspond à un timestamp
        writeSensorData(btSerial, &rxBuffer[1], Date, lat, lon, temperature, humidity, co2Concentration, mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, true, false);
      }
    } else {
      
      // 3. Stockage de secours si connexion est coupée
      if (StockageFile.open("Off.csv", O_RDWR | O_CREAT | O_AT_END)) { // sinon, créer un fichier de sauvegarde temporaire qui se videra à la première reconnexion ; permet d'éviter toute intervention physique
        writeSensorData(StockageFile, "", Date, lat, lon, temperature, humidity, co2Concentration, mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, false, false);
        StockageFile.close();
      }else {
        Serial.println(F("Erreur : Impossible d'ouvrir Off.csv!"));
      }
    }
  }
}
