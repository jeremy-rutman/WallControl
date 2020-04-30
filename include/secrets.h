// Use this file to store all of the private credentials 
// and connection details

#define   SECRET_SSID  "Tronet"
#define   SECRET_PASS  "Shira2306"

 
// IoT Guru
// select the device this code is used in 

#define SPARKFUN_SHIELD = 1
#define WEMOS = 2
#define DEVICE = SPARKFUN_SHIELD




#define   USER_ID  "g46NIng-txfaUvdQZj0R6g"
#define   USER_PASS 
#ifdef    WEMOS
#define   DEVICE_ID  "g46NIng-txdn60mgZj4R6g"
#define   DEVICE_KEY  "iORWiO1Zwa3scZg2V9xKMQ"
#elif     SPARKFUN_SHIELD 
#define   DEVICE_ID  "klrv5wfaa6t82ClQfcIR6g"
#define   DEVICE_KEY "s_QOxnkl-bXRZnkJnOdDrQ"
#endif
  

// Nodes:

#define  SHD31d_1 0
#define  SHD31d_2 1
#define  TempInside 2
#define  TempOutside 3
#define  TempPassiveWall 4
#define  TempActiveWall 5
#define  TopFan 6
#define  BottomFan 7
#define BME_1 8
#define BME_CC_1 9
#define numNodes 10



typedef struct node_t {
  char ID[25];
  char key[25];
  
};

static const char *node_names[numNodes]={
"SHD31d_1",
"SHD31d_2",
"TempInside",
"TempOutside",
"TempPassiveWall",
"TempActiveWall",
"TopFan",
"BottomFan",
"EnvCond",
"InTopCell",
};

static const char *node_key[numNodes]={
"uMxOcN7ASozG-uWwW7RHvA",
"twKpJtCmxwK6wmr7KlJCag",
"vLmd_kytYeADibvCQy5EKQ",
"nJiboocbQsCDqI_IYC1MDQ",
"rzy63i1_MoPY8XHx-hNAcQ",
"pSEzr4aovipVcV0aO0NAVA",
"rx7IOMlDJTNzhNixANJCJw",
"rYgyukKGGnqJuE0AAQRC8Q",
"kgB4hu2dO4eeFNuYZNhC6w",
"oBrKtWOL9OHIBFBATqJBJg",
};


static const char *node_ID[numNodes]={
"uaqlk7ObpcS9MjCQZj4R6g",
"uaqlk7ObpcTEKVTwZj4R6g",
"uaqlk7ObpcTte_HwZj4R6g",
"uaqlk7ObpcT5cSnQZj4R6g",
"uaqlk7ObpcTk7T2gZj4R6g",
"g46NIng-txfYt8hwZj4R6g",
"g46NIng-txerxsegZkER6g",
"uaqlk7ObpcTCVSnQZkER6g",
"jWcrox0QP1arY-cAfcIR6g",
"ggSLn0x7f3iF4suAfl4R6g"
};

struct measurement_s {
  float value;
  short device;
  short node;
  short output_pin;
  char *fieldname;
  char *unit;
  float scale;
} ;

#define  SHD31d_1_temp 0
#define  SHD31d_1_humidity 1

#define  SHD31d_2_temp 2
#define  SHD31d_2_humidity 3

#define  TempInside_temp 4
#define  TempOutside_temp 5
#define  TempPassiveWall_temp 6
#define  TempActiveWall_temp 7

#define  TopFan_pwm 8
#define  TopFan_tacho 9
#define  TopFan_vfr 10

#define  BottomFan_pwm 11
#define  BottomFan_tacho 12
#define  BottomFan_vfr 13
#define  EnvCond 14
#define  InTopCell 15

#define  NumFields 16

#define NON_EDITABLE -1


measurement_s measurements_array[]{
	{0, WEMOS, SHD31d_1 , NON_EDITABLE, "temp", "oC",1},
	{0, WEMOS, SHD31d_1 , NON_EDITABLE, "hum", "%",1},
	{0, WEMOS, SHD31d_2 , NON_EDITABLE, "temp", "oC",1},
	{0, WEMOS, SHD31d_2 , NON_EDITABLE, "hum", "%",1},
	{0, WEMOS, TempInside , NON_EDITABLE, "temp", "oC",1},
	{0, WEMOS, TempOutside , NON_EDITABLE, "temp", "oC",1},
	{0, WEMOS, TempPassiveWall , NON_EDITABLE, "temp", "oC",1},
	{0, WEMOS, TempActiveWall , NON_EDITABLE, "temp", "oC",1},
	{0, WEMOS, TopFan , D7, "pwm", "% DC",10.24},
	{0, WEMOS, TopFan , NON_EDITABLE, "tacho", "RPM",1},
	{0, WEMOS, TopFan , NON_EDITABLE, "vfr", "V",1},
	{0, WEMOS, BottomFan , D6, "pwm", "% DC",10.24},
	{0, WEMOS, BottomFan , NON_EDITABLE, "tacho", "RPM",1},
	{0, WEMOS, BottomFan , NON_EDITABLE, "vfr", "V",1}
    {0, SPARKFUN_SHIELD, EnvCond, NON_EDITABLE, "co2","PPM",1},
    {0, SPARKFUN_SHIELD,  EnvCond, NON_EDITABLE, "humidity","%",1},
    {0, SPARKFUN_SHIELD,  EnvCond, NON_EDITABLE, "pressure","kP",1},
    {0, SPARKFUN_SHIELD,  EnvCond, NON_EDITABLE, "temperature","oC",1},
    {0, SPARKFUN_SHIELD,  EnvCond, NON_EDITABLE, "voc","PPB",1}
};

	
