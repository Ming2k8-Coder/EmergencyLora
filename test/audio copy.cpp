#include <Arduino.h>
#include <driver/i2s.h>
#include <audio.h>
#include <firealram.wav.h>
//--------------------------
// I2S Configuration
#define bck 18
#define ws 19
#define data 5
//--------------------------
bool sound = false;
bool i2sInitialized = false;
#define SAMPLE_RATE 8000
#define BITS_PER_SAMPLE 16
#define SPK I2S_NUM_0
unsigned const char* TheData;
uint32_t DataIdx=0;                           // index offset into "TheData" for current  data t send to I2S

struct WavHeader_Struct
    {
      //   RIFF Section    
      char RIFFSectionID[4];      // Letters "RIFF"
      uint32_t Size;              // Size of entire file less 8
      char RiffFormat[4];         // Letters "WAVE"
      
      //   Format Section    
      char FormatSectionID[4];    // letters "fmt"
      uint32_t FormatSize;        // Size of format section less 8
      uint16_t FormatID;          // 1=uncompressed PCM
      uint16_t NumChannels;       // 1=mono,2=stereo
      uint32_t SampleRate;        // 44100, 16000, 8000 etc.
      uint32_t ByteRate;          // =SampleRate * Channels * (BitsPerSample/8)
      uint16_t BlockAlign;        // =Channels * (BitsPerSample/8), effectivly the size of a single sample for all chans.
      uint16_t BitsPerSample;     // 8,16,24 or 32
    
      // Data Section
      char DataSectionID[4];      // The letters "data"
      uint32_t DataSize;          // Size of the data that follows
    }WavHeader;
    

void i2s_init()
{
    if (!i2sInitialized) {

    static const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,                            // Note, this will be changed later
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // high interrupt priority
        .dma_buf_count = 8,                             // 8 buffers
        .dma_buf_len = 1024,                            // 1K per buffer, so 8K of buffer space
        .use_apll=0,
        .tx_desc_auto_clear= true, 
        .fixed_mclk=-1    
    };

// These are the physical wiring connections to our I2S decoder board/chip from the esp32, there are other connections
// required for the chips mentioned at the top (but not to the ESP32), please visit the page mentioned at the top for
// further information regarding these other connections.

    static const i2s_pin_config_t pin_config = {
    .bck_io_num = bck,                                 // The bit clock connectiom, goes to pin 27 of ESP32
    .ws_io_num = ws,                                  // Word select, also known as word select or left right clock
    .data_out_num = data,                               // Data out from the ESP32, connect to DIN on 38357A
    .data_in_num = -1                // we are not interested in I2S data into the ESP32
};
        esp_err_t err3 = i2s_driver_install(SPK, &i2s_config, 0, NULL);
        esp_err_t err4 = i2s_set_pin(SPK, &pin_config);

        if (err3 == ESP_OK && err4 == ESP_OK) {
            ESP_LOGI("audio_client", "i2s_init() - successful");
            Serial.println("I2s sound Succes init...");
            i2sInitialized = true;
        } else {
            ESP_LOGW("audio_client", "i2s_init() - error");
            Serial.println("I2 Sound FAILED...");
        }
    }
    audioprepare();
}


    
//------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------
// I2S configuration structures



//------------------------------------------------------------------------------------------------------------------------


void audioprepare() {
    const unsigned char *WavFile=rawData;
     memcpy(&WavHeader,WavFile,44);                    // Copy the header part of the wav data into our structure                       // Dump the header data to serial, optional!
    if(ValidWavData(&WavHeader))
    {                        // Tell it the pins you will be using
      i2s_set_sample_rates(SPK, WavHeader.SampleRate);      //set sample rate 
      TheData=WavFile+44;   
      Serial.println("Audio Getting started")   ;                                 // set to start of data  
    }
}                 


void AlaramOn(void *pvParameter)
{while(1){ 
  //Serial.println("AlaramOns");   
  uint8_t Mono[4];                             // This holds the data we actually send to the I2S if mono sound
  const unsigned char *Data;                   // Points to the data we are going to send                                              
  size_t BytesWritten = 0;                         // Returned by the I2S write routine, we are not interested in it

  // The WAV Data could be mono or stereo but always 16 bit, that's a data size of 2 byte or 4 bytes  
  // Unfortunatly I2S only allows stereo, so to send mono we have to send the mono sample on both left and right
  // channels. It's a bit of a faf really!
  while(DataIdx<WavHeader.DataSize){
  if(WavHeader.NumChannels==1)     // mono
  {
    Mono[0]=*(TheData+DataIdx);                 // copy the sample to both left and right samples, this is left
    Mono[1]=*(TheData+DataIdx+1);      
    Mono[2]=*(TheData+DataIdx);                 // Same data to the right channel
    Mono[3]=*(TheData+DataIdx+1);     
    Data=Mono;    
  }
  else                            // stereo
    Data=TheData+DataIdx;
    //Serial.print("datawiten");Serial.print(":");Serial.println(WavHeader.BlockAlign);
    esp_err_t err5 =i2s_write(SPK,Data,4,&BytesWritten,1000); 
    Serial.print("Errwriting:");Serial.println(err5);
    DataIdx+=WavHeader.BlockAlign;}               // If we gone past end of data reset back to beginning
    DataIdx=0;                            // increase the data index to next next sample
                                  
    
  }
}

bool ValidWavData(WavHeader_Struct* Wav)
{
  
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0) 
  {    
    Serial.print("Invlaid data - Not RIFF format");
    return false;        
  }
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
  {
    Serial.print("Invlaid data - Not Wave file");
    return false;           
  }
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0) 
  {
    Serial.print("Invlaid data - No format section found");
    return false;       
  }
  if(memcmp(Wav->DataSectionID,"data",4)!=0) 
  {
    Serial.print("Invlaid data - data section not found");
    return false;      
  }
  if(Wav->FormatID!=1) 
  {
    Serial.print("Invlaid data - format Id must be 1");
    return false;                          
  }
  if(Wav->FormatSize!=16) 
  {
    Serial.print("Invlaid data - format section size must be 16.");
    return false;                          
  }
  if((Wav->NumChannels!=1)&(Wav->NumChannels!=2))
  {
    Serial.print("Invlaid data - only mono or stereo permitted.");
    return false;   
  }
  if(Wav->SampleRate>48000) 
  {
    Serial.print("Invlaid data - Sample rate cannot be greater than 48000");
    return false;                       
  }
  if(Wav->BitsPerSample!=16) 
  {
    Serial.print("Invlaid data - Only 16 bits per sample permitted.");
    return false;                        
  }
  return true;
}


void DumpWAVHeader(WavHeader_Struct* Wav)
{
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0)
  {
    Serial.print("Not a RIFF format file - ");    
    PrintData(Wav->RIFFSectionID,4);
    return;
  } 
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
  {
    Serial.print("Not a WAVE file - ");  
    PrintData(Wav->RiffFormat,4);  
    return;
  }  
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0)
  {
    Serial.print("fmt ID not present - ");
    PrintData(Wav->FormatSectionID,3);      
    return;
  } 
  if(memcmp(Wav->DataSectionID,"data",4)!=0)
  {
    Serial.print("data ID not present - "); 
    PrintData(Wav->DataSectionID,4);
    return;
  }  
  // All looks good, dump the data
  Serial.print("Total size :");Serial.println(Wav->Size);
  Serial.print("Format section size :");Serial.println(Wav->FormatSize);
  Serial.print("Wave format :");Serial.println(Wav->FormatID);
  Serial.print("Channels :");Serial.println(Wav->NumChannels);
  Serial.print("Sample Rate :");Serial.println(Wav->SampleRate);
  Serial.print("Byte Rate :");Serial.println(Wav->ByteRate);
  Serial.print("Block Align :");Serial.println(Wav->BlockAlign);
  Serial.print("Bits Per Sample :");Serial.println(Wav->BitsPerSample);
  Serial.print("Data Size :");Serial.println(Wav->DataSize);
}

void PrintData(const char* Data,uint8_t NumBytes)
{
    for(uint8_t i=0;i<NumBytes;i++)
      Serial.print(Data[i]); 
      Serial.println();  
}