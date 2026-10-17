/**
 * @file lcd.c
 * @brief 默认通过 SPI1 写 LCD 命令和像素，实现初始化、图形、字库和数字绘制。
 *
 * LCD_Init() 准备屏幕，绘图入口先用 LCD_Address_Set() 选区域，再发送颜色。
 * lcd_buf.c 也调用这里的底层接口；参数和使用条件集中在 lcd.h。
 */
#include "lcd.h"
#include "lcdfont.h"


void LCD_Writ_Bus(uint8_t dat) 
{	
	LCD_CS_Clr();
#if USE_ANALOG_SPI
	// 软件发送时先发最高位，每次循环送出一位，共发送 8 位。
	for(uint8_t i=0;i<8;i++) {			  
		LCD_SCLK_Clr();
		if(dat&0x80) {
		   LCD_MOSI_Set();
		} else {
		   LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat<<=1;
	}
#else
	// 硬件 SPI 阻塞发送一个字节；当前接口未处理 HAL 返回的发送状态。
	HAL_SPI_Transmit(&hspi1, &dat, 1, 0xffff);
#endif	
  LCD_CS_Set();	
}

void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}

void LCD_WR_DATA(uint16_t dat)
{
	// RGB565 像素按高字节、低字节的顺序发送。
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}

void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();// 本次发送命令。
	LCD_Writ_Bus(dat);
	LCD_DC_Set();// 随后的字节恢复为命令参数或像素数据。
}

void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	// 竖屏的行坐标、横屏的列坐标要加 20；四个方向都加上绘图区留白。
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+LCD_MARGIN_X);
		LCD_WR_DATA(x2+LCD_MARGIN_X);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+20+LCD_MARGIN_Y);
		LCD_WR_DATA(y2+20+LCD_MARGIN_Y);
		LCD_WR_REG(0x2c);// 接下来发送本区域的像素。
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+LCD_MARGIN_X);
		LCD_WR_DATA(x2+LCD_MARGIN_X);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+20+LCD_MARGIN_Y);
		LCD_WR_DATA(y2+20+LCD_MARGIN_Y);
		LCD_WR_REG(0x2c);// 接下来发送本区域的像素。
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+20+LCD_MARGIN_X);
		LCD_WR_DATA(x2+20+LCD_MARGIN_X);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+LCD_MARGIN_Y);
		LCD_WR_DATA(y2+LCD_MARGIN_Y);
		LCD_WR_REG(0x2c);// 接下来发送本区域的像素。
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+20+LCD_MARGIN_X);
		LCD_WR_DATA(x2+20+LCD_MARGIN_X);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+LCD_MARGIN_Y);
		LCD_WR_DATA(y2+LCD_MARGIN_Y);
		LCD_WR_REG(0x2c);// 接下来发送本区域的像素。
	}
}

void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	// 地址窗口包含末端，而本函数不画末端行、列，因此各减 1。
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);
	// 按行、列发出相同颜色，填满选中的矩形。
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);// 起点和终点相同，只选中一个像素。
	LCD_WR_DATA(color);
} 

void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1;
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	// 根据终点位置选择向左、向右或保持 x；y 方向同样处理。
	if(delta_x>0)incx=1;
	else if (delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;
	else {incy=-1;delta_y=-delta_y;}
	// 取较大的坐标差作为循环次数的依据，另一方向按累计值决定何时移动。
	if(delta_x>delta_y)distance=delta_x;
	else distance=delta_y;
	// 沿用原有的 > 比较：非零长度时起点会重复画，循环后不补画终点。
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);
		xerr+=delta_x;
		yerr+=delta_y;
		// 累计值超过 distance 时，该方向前进一个像素，并扣除本次用掉的部分。
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}

void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;	  
	// 只计算八分之一圆周，每轮利用上下、左右对称画出八个点。
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);
		LCD_DrawPoint(x0+b,y0-a,color);
		LCD_DrawPoint(x0-a,y0+b,color);
		LCD_DrawPoint(x0-a,y0-b,color);
		LCD_DrawPoint(x0+b,y0+a,color);
		LCD_DrawPoint(x0+a,y0-b,color);
		LCD_DrawPoint(x0+a,y0+b,color);
		LCD_DrawPoint(x0-b,y0+a,color);
		a++;
		if((a*a+b*b)>(r*r))// 新位置超过半径时，向圆心收回一个像素。
		{
			b--;
		}
	}
}

void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	// 每次处理两字节索引，遇到 0 停止；这里不解码 UTF-8，也不自动换行。
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;// 没有对应字号时停止绘制。
		s+=2;
		// 没在字库中找到的字也占一个字宽，保持后续位置不变。
		x+=sizey;
	}
}

void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;// 当前字号的字库项数。
	uint16_t TypefaceNum;// 字模总字节数；每行不足 8 位的部分也占一个字节。
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);
	// 逐项比较索引的前两个字节；没有匹配项时不画任何内容。
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			// 逐字节读取字模，每个字节从低位开始决定要画哪些像素。
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)// mode 为 0：字模位为 1 画字色，为 0 画背景色。
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						// 已写够一行时跳过本字节剩下的位，避免把行尾补位画出来。
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else// 只画字形；字模位为 0 时保留屏上原有颜色。
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);
						x++;
						// 到达行末后回到左侧，下一字节从下一行继续画。
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;// 继续检查后续字库项，匹配后也不会提前退出。
	}
} 

void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;// 当前字号的字库项数。
	uint16_t TypefaceNum;// 字模总字节数；每行不足 8 位的部分也占一个字节。
	uint16_t x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);
	// 逐项比较索引的前两个字节；没有匹配项时不画任何内容。
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			// 逐字节读取字模，每个字节从低位开始决定要画哪些像素。
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)// mode 为 0：字模位为 1 画字色，为 0 画背景色。
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						// 已写够一行时跳过本字节剩下的位，避免把行尾补位画出来。
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else// 只画字形；字模位为 0 时保留屏上原有颜色。
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);
						x++;
						// 到达行末后回到左侧，下一字节从下一行继续画。
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;// 继续检查后续字库项，匹配后也不会提前退出。
	}
} 

void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;// 当前字号的字库项数。
	uint16_t TypefaceNum;// 字模总字节数；每行不足 8 位的部分也占一个字节。
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);
	// 逐项比较索引的前两个字节；没有匹配项时不画任何内容。
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			// 逐字节读取字模，每个字节从低位开始决定要画哪些像素。
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)// mode 为 0：字模位为 1 画字色，为 0 画背景色。
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						// 已写够一行时跳过本字节剩下的位，避免把行尾补位画出来。
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else// 只画字形；字模位为 0 时保留屏上原有颜色。
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);
						x++;
						// 到达行末后回到左侧，下一字节从下一行继续画。
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;// 继续检查后续字库项，匹配后也不会提前退出。
	}
} 

void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;// 当前字号的字库项数。
	uint16_t TypefaceNum;// 字模总字节数；每行不足 8 位的部分也占一个字节。
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);
	// 逐项比较索引的前两个字节；没有匹配项时不画任何内容。
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			// 逐字节读取字模，每个字节从低位开始决定要画哪些像素。
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)// mode 为 0：字模位为 1 画字色，为 0 画背景色。
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						// 已写够一行时跳过本字节剩下的位，避免把行尾补位画出来。
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else// 只画字形；字模位为 0 时保留屏上原有颜色。
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);
						x++;
						// 到达行末后回到左侧，下一字节从下一行继续画。
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;// 继续检查后续字库项，匹配后也不会提前退出。
	}
}

void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;// 字模总字节数，行尾不足 8 位也按一个字节计。
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    // 字库从空格开始排列，减去空格的编号后得到数组下标。
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);
	// 按字号选字库，逐字节取出点阵；调用方须先保证 num 在支持范围内。
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;// 字号不支持，停止读取字模。
		// 每个字节先读低位；不足一个字节的行尾由下面的 break 跳过。
		for(t=0;t<8;t++)
		{
			if(!mode)// 覆盖绘制：每个像素都写入字色或背景色。
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else// 保留背景：只给字模中为 1 的位置画点。
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);
				x++;
				// 画够一个字符宽度后回到下一行左侧。
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	// 每次画一个 ASCII 字符，遇到字符串结尾停止，位置始终向右移动。
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}

uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	// 乘 n 次；n 为 0 时不进入循环，保留初值 1。
	while(n--)result*=m;
	return result;
}

void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	// 从高位向低位取十进制数字；len 小于实际位数时，高位不会画出。
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		// 首个非零数字之前用空格清背景，末位始终显示，保证 0 不会全空。
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 

void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
		int16_t num_int;
		uint8_t t, temp, sizex;
    sizex = sizey / 2;
    // 放大小数后存为 int16_t，小数余下的部分直接舍去；这里不检查数值范围。
    num_int = num * mypow(10, decimal);

    // 首格画负号或空格，数字从右侧开始；两条分支都会把 len 加 1。
    if (num < 0)
    {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);
        num_int = -num_int;
        x += sizex;
        len++;
    }
    else
    {
        LCD_ShowChar(x, y, ' ', fc, bc, sizey, 0);
        num_int = num_int;
        x += sizex;
        len++;
    }

    // 先清理数字区，避免旧数字残留；清理范围还包含字形下面的一行。
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    // 从高位向低位画数；len 已在上面增加，不能只按传入值理解总位数。
    for (t = 0; t < len; t++)
    {
        if (t == (len - decimal))
        {
            // 小数点占一格，跳过这一格，并增加长度以保留后面的小数位。
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}

void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
		int16_t num_int;
		uint8_t t, temp, sizex;
    sizex = sizey / 2;
    // 此入口供非负数使用，放大后须能存入 int16_t；不做四舍五入或范围检查。
    num_int = num * mypow(10, decimal);


		// 与带符号入口对齐，先空出一格；这一格保留原画面。
		num_int = num_int;
		x += sizex;
		len++;

    // 仅清理数字区，包含字形下面的一行，不清理左侧预留位置。
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    // 与带符号入口一样，从高位向低位画数，并在需要的位置插入小数点。
    for (t = 0; t < len; t++)
    {
        if (t == (len - decimal))
        {
            // 小数点占一格，后面的数字向右顺延，仍保留指定的小数位数。
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}

void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint16_t i,j;
	uint32_t k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	// 两层循环共发送 length*width 个像素；k 始终按数组顺序递增。
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			// 一个像素两个字节，数组已按高字节在前准备，不在此处换顺序。
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}

void LCD_Init(void)
{
	
	// 拉低复位脚后再释放，两次等待给模组留出复位和恢复时间。
	LCD_RES_Clr();
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);
	
	LCD_BLK_Set();// 打开背光后再等待 100 ms。
  HAL_Delay(100);
	
	LCD_WR_REG(0x11);// 退出休眠，等 120 ms 后再继续配置。
	HAL_Delay(120);
	// 显示方向要与 LCD_Address_Set() 的坐标偏移保持一致。
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);// 每像素使用 RGB565 两字节颜色。
	LCD_WR_DATA8(0x05);

	// 沿用本模组原有显示参数，设置扫描、电压和刷新相关寄存器。
	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C); 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x33); 
	LCD_WR_DATA8(0x33); 			

	LCD_WR_REG(0xB7);			
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);			
	LCD_WR_DATA8(0x32);
					
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);			
	LCD_WR_DATA8(0x15);
				
	LCD_WR_REG(0xC4);			
	LCD_WR_DATA8(0x20);

	LCD_WR_REG(0xC6);			
	LCD_WR_DATA8(0x0F);

	LCD_WR_REG(0xD0);			
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1); 

	// 两组亮度曲线参数配套使用，保持本模组原有配置。
	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);   

	LCD_WR_REG(0xE1);     
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);
	LCD_WR_REG(0x21);// 按本模组配置开启显示反色，与文字绘制的 mode 无关。

	LCD_WR_REG(0x29);// 打开显示；清屏和界面重画由上层执行。
}

