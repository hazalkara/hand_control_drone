#include <Servo.h>

#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h> 
#include <SPI.h>


Servo servoThrottle;
Servo servoYaw;
Servo servoPitch;
Servo servoRoll;

Enrf24 nRF(9, 10, 2);  // CE , CS, IRQ bacaklarının Arduino ile bağlantıları tanımlandı

char gelen_bilgi[32]; // gelen string türünde bilgi için geçici dizi değişkken

const byte alici_adresi[] = { 0xDE, 0xAD, 0xBE, 0x0F, 0x01 };    // alıcı-verici adresi aynı olacak

int y_map, r_map, p_map, t_map, y_dizi ,p_dizi ,r_dizi;

uint8_t t_dizi;


void setup() 
{
  Serial.begin(115200); 
  Serial.println("test");
  SPI_Baslat();
  NRF_Baslat();


  servoThrottle.attach(7);  //fonk yap servo baslatmaları 
  servoYaw.attach(6); 
  servoPitch.attach(5); 
  servoRoll.attach(4); 
  
}
void loop()
{
  NRF_Gelen();
  
 
  
}
void SPI_Baslat(){
  SPI.begin();                                                                                
  SPI.setDataMode(SPI_MODE0); // SPI MODE0 seçildi, nrf24l01 MODE0 ile iletişim kurmaktadır
  SPI.setBitOrder(MSBFIRST); // bit sıralaması MSB'den LSB'ye doğru ayarlandı
}
void NRF_Baslat(){

  nRF.begin(2000000,124);        //datarate(150Hz civarı =1000000 iken, en optimum) 124=kanal                                                                

  nRF.setRXaddress(alici_adresi); // alıcı adresi ayarlandı
  
  nRF.enableRX();  // Dinlemeye başla
  
}
void NRF_Gelen(){
  
  
  if (nRF.read(gelen_bilgi))  // bilgi geldiyse bunu gelen_bilgi değişkenine aktar
  {
    
    y_dizi= gelen_bilgi[1];
    p_dizi= gelen_bilgi[2];
    r_dizi= gelen_bilgi[3];
    t_dizi= gelen_bilgi[0];

    y_map= map(y_dizi, -180, 180, 0, 180);
    p_map= map(p_dizi, -180, 180, 0, 180);
    r_map= map(r_dizi, -180, 180, 0, 180);
    t_map= map(t_dizi, 1, 180, 0, 140);

    servoThrottle.write(t_map);
    servoYaw.write(y_map);
    servoPitch.write(p_map);
    servoRoll.write(r_map);

    
    Serial.print("ypr_MAP\t");
    Serial.print("\t");
    Serial.print(y_map); 
    Serial.print("\t");
    Serial.print(p_map); 
    Serial.print("\t");
    Serial.print(r_map); 
    Serial.print("\t");
    Serial.println(t_map); 
    
  }
  
}