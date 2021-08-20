//SPI RFID
#define MISO_V	GPIO_NUM_12
#define MOSI_V	GPIO_NUM_13
#define SCK_V	GPIO_NUM_4
#define CS_RFID	GPIO_NUM_17

//Estructuras
struct Tarjetas_RFID
{
	uint32_t	tarjeta;
	char	nombre [20];
}typedef Tarjetas_RFID;


//Definicion Funciones
void ConvIntaChar(uint32_t dato,char* auxRfid);
esp_err_t Comparar(uint32_t tarjAcomparar);
void LectRfid (void *pvParameter);
esp_err_t rfid_start();

