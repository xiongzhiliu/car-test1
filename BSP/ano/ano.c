#include "ano.h"
unsigned char DataToSend[100];

void ANO_SendF1(short a, short b, short c, unsigned char d)
{
    unsigned char cnt = 0; // 鍦板潃鍋忕�?
    unsigned char sc = 0;  // 鍜屾牎�?��
    unsigned char ac = 0;  // 闄勫姞鏍￠獙
    unsigned char i = 0;

    DataToSend[cnt++] = 0XAA; // �?у�?
    DataToSend[cnt++] = 0XFF; // 鐩�鏍囧湴鍧�?
    DataToSend[cnt++] = 0XF1; // 鍔熻兘鐮�?
    DataToSend[cnt++] = 7;    // 鏁版嵁闀�?害锛歩nt16 + int16 +int16 + uint8 = 2+2+2+1 = 7

    DataToSend[cnt++] = BYTE0(a);
    DataToSend[cnt++] = BYTE1(a);

    DataToSend[cnt++] = BYTE0(b);
    DataToSend[cnt++] = BYTE1(b);

    DataToSend[cnt++] = BYTE0(c);
    DataToSend[cnt++] = BYTE1(c);

    DataToSend[cnt++] = BYTE0(d);

    for (i = 0; i < DataToSend[3] + 4; i++)
    {
        sc += DataToSend[i]; // 璁＄畻鍜屾牎楠�
        ac += sc;            // 璁＄畻闄�?姞鏍￠獙
    }

    DataToSend[cnt++] = sc; // 渚濇�″彂�?佷袱涓�鏍￠�?
    DataToSend[cnt++] = ac;
    USART0_SendStr(DataToSend, cnt); // �?氳繃涓插彛鍙戦€佹暟缁勭殑鏂瑰紡鍙戦�?佹暣涓�鏁版嵁甯�?
		return;
}

void ANO_SendF2(short a, short b, short c, short d)
{
    unsigned char cnt = 0; // 鍦板潃鍋忕�?
    unsigned char sc = 0;  // 鍜屾牎�?��
    unsigned char ac = 0;  // 闄勫姞鏍￠獙
    unsigned char i = 0;

    DataToSend[cnt++] = 0XAA; // �?у�?
    DataToSend[cnt++] = 0XFF; // 鐩�鏍囧湴鍧�?
    DataToSend[cnt++] = 0XF2; // 鍔熻兘鐮�?
    DataToSend[cnt++] = 8;    // 鏁版嵁闀�?�?4浣嶏�?2涓猻hort�?诲瀷鏁版嵁锛�

    DataToSend[cnt++] = BYTE0(a);
    DataToSend[cnt++] = BYTE1(a);

    DataToSend[cnt++] = BYTE0(b);
    DataToSend[cnt++] = BYTE1(b);

    DataToSend[cnt++] = BYTE0(c);
    DataToSend[cnt++] = BYTE1(c);

    DataToSend[cnt++] = BYTE0(d);
    DataToSend[cnt++] = BYTE1(d);

    for (i = 0; i < DataToSend[3] + 4; i++)
    {
        sc += DataToSend[i];
        ac += sc;
    }
    DataToSend[cnt++] = sc;
    DataToSend[cnt++] = ac;

   // printf("%d", DataToSend[1]);
   //USART0_SendStr(DataToSend, cnt); // �?氳繃涓插彛鍙戦€佹暟缁勭殑鏂瑰紡鍙戦�?佹暣涓�鏁版嵁甯�?
		Bluetooth_SendStr(DataToSend, cnt);
    return;	
}

extern uint8_t blue_node_num;
void ANO_Direct_distance(uint8_t dir,uint8_t num)
{
    unsigned char cnt = 0; // 鍦板潃鍋忕�?
    unsigned char sc = 0;  // 鍜屾牎�?��
    unsigned char ac = 0;  // 闄勫姞鏍￠獙
    unsigned char i = 0;

    //DataToSend[cnt++] = 0XAA; // �?у�?
    DataToSend[cnt++] = 0XFF; // 鐩�鏍囧湴鍧�?
    //DataToSend[cnt++] = 0XF2; // 鍔熻兘鐮�?
    //DataToSend[cnt++] = 10;    // 鏁版嵁闀�?�?4浣嶏�?2涓猻hort�?诲瀷鏁版嵁锛�

    short d = 0;
   
    switch (dir%4)
    {
        case 0:
            d=0x02;
            break;
        case 1:
            d=0x03;
            break;
        case 2:
            d=0x00;
            break;
        case 3:
            d=0x01;
            break;
        default:
            break;
    }
    DataToSend[cnt++] = BYTE0(d);
    //DataToSend[cnt++] = BYTE1(d);
	
		DataToSend[cnt++] = BYTE0(num);

		
//    for (i = 0; i < DataToSend[3] + 4; i++)
//    {
//        sc += DataToSend[i];
//        ac += sc;
//    }
    //DataToSend[cnt++] = sc;
    //DataToSend[cnt++] = ac;

   // printf("%d", DataToSend[1]);
   //USART0_SendStr(DataToSend, cnt); // �?氳繃涓插彛鍙戦€佹暟缁勭殑鏂瑰紡鍙戦�?佹暣涓�鏁版嵁甯�?
		Bluetooth_SendStr(DataToSend, cnt);
    return;	
}