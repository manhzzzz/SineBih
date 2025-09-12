 SSD1306_Init();
SSD1306_GotoXY(0,0);
SSD1306_Puts("HELLO",&Font_11x18,1);
SSD1306_GotoXY(10,30);
SSD1306_Puts("WORLD:)",&Font_11x18,1);
SSD1306_UpdateScreen();
HAL_Delay(2000);

SSD1306_ScrollRight(0x00,0x0f);
HAL_Delay(2000);
SSD1306_ScrollLeft(0x00,0x0f);
SSD1306_Stopscroll();