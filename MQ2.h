#ifndef MQ2_h
#define MQ2_h
#endif



/************************Hardware Related Macros************************************/
#define         RL_VALUE                     5     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          9.83  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     50   //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  500   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         50    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            5     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      0
#define         GAS_CO                       1
#define         GAS_SMOKE                    2

/*****************************Globals***********************************************/
float           LPGCurve[3]  = {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};;    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] = {2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms                                              

float lpg = 0;
float co = 0;
float smoke = 0;

/**********************************************function-define-----------------------------*/
float* read(bool print);

float readLPG();
float readCO();
float readSmoke();
void begin();


float MQRead();
float MQGetGasPercentage(float rs_ro_ratio, int gas_id);
int MQGetPercentage(float rs_ro_ratio, float *pcurve);
float MQCalibration();
float MQResistanceCalculation(int raw_adc);

int lastReadTime = 0;
