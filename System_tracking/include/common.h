void fatal_error (void);

#define GPS_CONECTADO       //Si esta definido signifa que se utiliza el mismo y se va a resetear el SIM

//  TRAMAS

#define TR_IN       "<!!>"
#define TR_POS_VEL  "<!UVH!>"
#define TR_IM       "<!IMG!>"
#define TR_VID      "<!VID!>"
#define TR_RFID     "<!RFD!>"
#define TR_CONF     "<!CONF!>"
#define AL_CHOQUE   "<!CHQ!>"
#define AL_VUELCO   "<!VLC!>"
#define AL_MAX_VEL  "<!AMV!>"
#define AL_SIN_MOV  "<!ASM!>"
#define AL_DISP     "<!NNN!>"
#define TR_FIN      "hola"

//Trama genércica <!!>MAC,<!VID!>...hola