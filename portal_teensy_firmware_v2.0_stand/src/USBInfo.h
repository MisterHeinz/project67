#include <usb_names.h>

#undef MANUFACTURER_NAME
#undef MANUFACTURER_NAME_LEN
#undef PRODUCT_NAME
#undef PRODUCT_NAME_LEN
#undef SERIAL_NUMBER
#undef SERIAL_NUMBER_LEN

#define MANUFACTURER_NAME	{'N','I','O','-','3','0','7',' ','M','A','I'}
#define MANUFACTURER_NAME_LEN	11
#define PRODUCT_NAME		{'P','o','r','t','a','l',' ','C','U'}
#define PRODUCT_NAME_LEN	9

/* 
    Расшифровка серийного номера
    
    0307 — НИО-307 МАИ
    0100 — тип устройства
    00 — деталь внутри устройства
    FFFF — порядковый номер, настраивается отдельно
*/
#define SERIAL_NUMBER   {   \
    '0','3','0','7',        \
    '0','1','0','0',        \
    '0','0',                \
    'F','F','F','F'         \
}

#define SERIAL_NUMBER_LEN	14

struct usb_string_descriptor_struct usb_string_manufacturer_name = {
    2 + MANUFACTURER_NAME_LEN * 2,
    3,
    MANUFACTURER_NAME
};

struct usb_string_descriptor_struct usb_string_product_name = {
    2 + PRODUCT_NAME_LEN * 2,
    3,
    PRODUCT_NAME
};

struct usb_string_descriptor_struct usb_string_serial_number = {
	2 + SERIAL_NUMBER_LEN * 2,
	3,
	SERIAL_NUMBER
};

static void initSerial(uint16_t serialNumber){
    for (int i = 0; i < 4; i++)
    {
        uint8_t symbol = ((serialNumber >> (12 - 4 * i)) & 0xF);
        symbol += symbol > 9 ? 55 : 48;
        usb_string_serial_number.wString[10 + i] = symbol;
    }
}
