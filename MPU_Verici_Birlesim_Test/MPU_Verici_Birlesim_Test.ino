#include <Enrf24.h>
#include <nRF24L01.h>
#include <SPI.h>

char dizi[32];

int y,p,r,t=0;

Enrf24 nRF(9, 10, 2);   // CE , CS , IRQ bacaklarının Arduino ile bağlantıları 

const byte verici_adresi[] = { 0xDE, 0xAD, 0xBE, 0x0F, 0x01 };     // verici adresi tanımlandı aynı adres alıcıda da tanımlanmalıdır

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h" 


MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 3  // interrupt/kesme pini

#define pot A1   //pot a0 tanumlandı

unsigned long eskiZaman=0;
unsigned long yeniZaman;
unsigned long farkZaman;


// MPU kontrol-durum değişkenleri
bool dmpReady = false;  //  DMP baslatma durumu
uint8_t mpuIntStatus;   //  Mpu dan gelen interrupt durum bitini tutar
uint8_t devStatus;      // her mpu isleminden sonra durumu döndürür
uint16_t packetSize;    // beklenen DMP paket boyutunu tutar (default 42 byte)
uint16_t fifoCount;     // FIFOda bulunan byte sayısını tutar
uint8_t fifoBuffer[64]; // FIFO depo arabelleği(buffer)


// hareket değişkenleri
Quaternion q;           // [w, x, y, z]         quaternion 
VectorFloat gravity;    // [x, y, z]            gravity vektörü
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll (quaternion ve yerçekimi vektöründen hesaplanan ypr değerlerini tutar)
int throt;

// teapot demosu paketi
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // MPU kesme pini durumu 
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    
    I2C_Baslat();
    MPU_Baslat();
    Kalib();

    SPI_Baslat();
    NRF_Baslat();
}

void loop() {
  yeniZaman = millis();
  if(yeniZaman-eskiZaman > 10){

    throt= analogRead(A1);
    throt= map(throt, 0, 1023, 1, 180);
    
   
    eskiZaman = yeniZaman;
    
 }
  
  
  YPRoll();
  NRF_Gonder();

 
}
void YPRoll(){
      
    if (!dmpReady) return; 
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { //  Fifodan paket oku(___güncel paket)

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // Euler açlarını derece olarak yazdır(radyan ->derece)
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            Serial.print("ypr\t");
            Serial.print(int (ypr[0] * 180/M_PI));
            Serial.print("\t");
            Serial.print(int (ypr[1] * 180/M_PI));
            Serial.print("\t");
            Serial.println(int (ypr[2] * 180/M_PI));
        #endif

      
    }
    
    

}
void I2C_Baslat(){
      //                I2C BASLAT(I2C veriyoluna dahil etme)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Derleme hatası verirse kapat!11!
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}
void MPU_Baslat(){
    //                  MPU-I2C BASLAT
    //Serial.println(F("I2C cihazları baslatılıyor..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT); 


    //Serial.println(F("Cihaz baglantılari test ediliyor..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 baglanti basarili") : F("MPU6050 baglanti basarisiz"));


    // DMP baslat(dmp yapılandırması..)                 
    //Serial.println(F("DMP baslatiliyor..."));
    devStatus = mpu.dmpInitialize();
}
void Kalib(){
      // gyro hassasiyetleri için offsetler belirlendi(Drone üzerinde tekrar yapıalcak!!)     ||||KALİBRASYONLAR|||
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 default (bizim MPUda farklı oalbilir!1!)

    if (devStatus == 0) {
        // offset ve kalibrasyon 
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        
        //Serial.println(F("DMP etkinleştiriliyor..")); 
        mpu.setDMPEnabled(true);

        // Ardu kesme(harici) etkinleştir
        //Serial.print(F("Kesme etkinleştiriliyor Ardu.. ")); 
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN)); 
        //Serial.println(F(")...")); //YORUM HATTA SİL 
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        //Serial.println(F("DMP ready! Waiting for first interrupt...")); 
        dmpReady = true; //LOOPta kullanmak için hazır

        // DMP paket boyutu al
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        //bellek yüklemesi ve yapılandırma başarısız
        //Serial.print(F("DMP baslatma basarisiz "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
}

void SPI_Baslat(){
  SPI.begin();  //                                                                                    
  SPI.setDataMode(SPI_MODE0);  // SPI MODE0 seçildi, nrf24l01 MODE0 ile iletişim kurmaktadır
  SPI.setBitOrder(MSBFIRST);   // bit sıralaması MSB'den LSB'ye doğru ayarlandı
}
void NRF_Baslat(){

  nRF.begin(2000000,124);       //  datarate 250000/1000000/2000000, kanal 0/125                      

  nRF.setTXaddress(verici_adresi);   // verici adresi ayarlandı

 

}
void NRF_Gonder(){

    y= (ypr[0] * 180/M_PI);     //radyandan dereceye çevrilen yaw değeri integer olur
    p= (ypr[1] * 180/M_PI);
    r= (ypr[2] * 180/M_PI);
    t= throt;

    dizi[0]=t;
    dizi[1]=y;
    dizi[2]=p;
    dizi[3]=r;
  
  nRF.print(dizi); //nrf gonder dizi                                
  nRF.flush();

  
  /*Serial.print("ypr\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(p);
  Serial.print("\t");
  Serial.print(r);
  Serial.print("\t");
  Serial.println(t);*/
   
}



