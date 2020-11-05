

#include <stdio.h>
#include <stdlib.h>
#include <math.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "MQ2.h"

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC1_CHANNEL_0;    //GPIO 36 For channel 0 --> gas sensor analog pin

static const adc_bits_width_t width = ADC_WIDTH_BIT_10;  //ADC capture width is 10 (Range 0-1023)

static const adc_atten_t atten = ADC_ATTEN_DB_0;      //No input attenumation, ADC can measure up to approx. 800 mV.

void begin()
{

    printf("Calibrating...\n");
    Ro = MQCalibration();
    printf("Calibration Done\n");
    printf("Ro: %.2lf Kohm\n",Ro); 
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 

float MQCalibration() {
    
    //printf("MQCalibration\n");
    float val=0;
    adc1_config_width(width);
    adc1_config_channel_atten(channel,atten);
    for (int i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
        int caliread = adc1_get_raw(channel);
        val += MQResistanceCalculation(caliread);
        printf(":i value:%d:Calibration ADC:%d:Val====:%.4f::\n",i,caliread,val);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    
    val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
    val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                          //according to the chart in the datasheet 
    printf("Calibration value: %f\n", val);

  return val; 
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc) 
{
   return (((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/****************** read infinite times ****************************************/
float* read(bool print){

   lpg = MQGetGasPercentage(MQRead()/Ro,GAS_LPG);
   co = MQGetGasPercentage(MQRead()/Ro,GAS_CO);
   smoke = MQGetGasPercentage(MQRead()/Ro,GAS_SMOKE);

       if(lpg > 1000 || co > 1000 || smoke > 1000){
       printf("!!!GAS DETECTTED!!!\n");
       printf("\nLPG::%.2f::ppm\n",lpg);
       printf("\nCO::%.2f::ppm\n",co);
       printf("\nSMOKE::%.2f::ppm\n\n",smoke);   
   }
       else
   {
       printf("::GAS NOT DETECTED::\n");
   }
   lastReadTime = xthal_get_ccount();
   float values[3] = {lpg,co,smoke};
   return values;
}

/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead() 
{
    int i;
    float rs=0;
    adc1_config_width(width);
    adc1_config_channel_atten(channel,atten);
    
    int val = adc1_get_raw(channel);
    printf("ADC1 Raw Value :: Sensor Reading:: %d :\n", val);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(val, adc_chars);
    printf("ADC1 Raw Value :: In Voltage:: %d :\n",val, voltage);
    
    for (i=0;i<READ_SAMPLE_TIMES;i++) 
    {
        rs += MQResistanceCalculation(val);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
        
    rs = rs/READ_SAMPLE_TIMES;
    return rs;  
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 

float MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if ( gas_id == GAS_LPG ) 
  {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } 
  else if ( gas_id == GAS_CO ) 
  {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } 
  else if ( gas_id == GAS_SMOKE ) 
  {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
  return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10,(((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}


/*****************************  readLPG Value --every 10sec interval **********************************/
float readLPG(){
    if (xthal_get_ccount()<(lastReadTime + 10000) && lpg != 0)
    {
        return lpg;
    }
    else
    {
        return lpg = MQGetGasPercentage(MQRead()/10,GAS_LPG);
    }
}

/*****************************  readCO Value -- every 10sec interval **********************************/
float readCO()
{
    if (xthal_get_ccount()<(lastReadTime + 10000) && co != 0){
        return co;
    }
    else
    {
        return co = MQGetGasPercentage(MQRead()/10,GAS_CO);
    }
}
/*****************************  readSmoke Value -- every 10sec interval **********************************/

float readSmoke()
{
    if (xthal_get_ccount()<(lastReadTime + 10000) && smoke != 0)
    {
        return smoke;
    }
    else
    {
        return smoke = MQGetGasPercentage(MQRead()/10,GAS_SMOKE);
    }
}

void app_main()
{
    begin();
    while(1){
        read(true);
    }
    printf("Main App End\n");
}
