#ifndef _ADS1256__H
#define _ADS1256__H

/******************ads1248寄存器地址*******************/
// define commands 
#define ADS1256_CMD_WAKEUP   	0x00     //完成SYNC和退出待机模式
#define ADS1256_CMD_RDATA    	0x01     //读数据
#define ADS1256_CMD_RDATAC   	0x03     //连续读数据
#define ADS1256_CMD_SDATAC   	0x0f     //停止连续读数据 
#define ADS1256_CMD_RREG     	0x10     //从寄存器度数据 
#define ADS1256_CMD_WREG     	0x50     //向寄存器写数据 
#define ADS1256_CMD_SELFCAL  	0xf0     //偏移和增益自动校准
#define ADS1256_CMD_SELFOCAL 	0xf1     //偏移自动校准 
#define ADS1256_CMD_SELFGCAL 	0xf2     //增益自动校准 
#define ADS1256_CMD_SYSOCAL  	0xf3     //系统失调校准 
#define ADS1256_CMD_SYSGCAL  	0xf4     //系统增益校准 
#define ADS1256_CMD_SYNC     	0xfc     //同步AD转换 
#define ADS1256_CMD_STANDBY  	0xfd     //待机模式开始 
#define ADS1256_CMD_REST     	0xfe     //复位
 
// define the ADS1256 register values 
#define ADS1256_STATUS       	0x00   
#define ADS1256_MUX         	 0x01   
#define ADS1256_ADCON        	0x02   
#define ADS1256_DRATE        	0x03   
#define ADS1256_IO           	0x04   
#define ADS1256_OFC0         	0x05   
#define ADS1256_OFC1         	0x06   
#define ADS1256_OFC2         	0x07   
#define ADS1256_FSC0         	0x08   
#define ADS1256_FSC1         	0x09   
#define ADS1256_FSC2         	0x0A 
 
 
// define multiplexer codes 
#define ADS1256_MUXP_AIN0   	0x00 
#define ADS1256_MUXP_AIN1   	0x10 
#define ADS1256_MUXP_AIN2   	0x20 
#define ADS1256_MUXP_AIN3   	0x30 
#define ADS1256_MUXP_AIN4   	0x40 
#define ADS1256_MUXP_AIN5   	0x50 
#define ADS1256_MUXP_AIN6   	0x60 
#define ADS1256_MUXP_AIN7   	0x70 
#define ADS1256_MUXP_AINCOM 	0x80 
 
#define ADS1256_MUXN_AIN0   	0x00 
#define ADS1256_MUXN_AIN1   	0x01 
#define ADS1256_MUXN_AIN2   	0x02 
#define ADS1256_MUXN_AIN3   	0x03 
#define ADS1256_MUXN_AIN4   	0x04 
#define ADS1256_MUXN_AIN5   	0x05 
#define ADS1256_MUXN_AIN6   	0x06 
#define ADS1256_MUXN_AIN7   	0x07 
#define ADS1256_MUXN_AINCOM 	0x08   
 
 
// define gain codes 
#define ADS1256_GAIN_1      	0x00 
#define ADS1256_GAIN_2      	0x01 
#define ADS1256_GAIN_4      	0x02 
#define ADS1256_GAIN_8      	0x03 
#define ADS1256_GAIN_16    	 	0x04 
#define ADS1256_GAIN_32     	0x05 
#define ADS1256_GAIN_64         0x06 
//#define ADS1256_GAIN_64         0x07 
 
//define drate codes 
#define ADS1256_DRATE_30000SPS   0xF0 
#define ADS1256_DRATE_15000SPS   0xE0 
#define ADS1256_DRATE_7500SPS  	 0xD0 
#define ADS1256_DRATE_3750SPS    0xC0 
#define ADS1256_DRATE_2000SPS  	 0xB0 
#define ADS1256_DRATE_1000SPS    0xA1 
#define ADS1256_DRATE_500SPS     0x92 
#define ADS1256_DRATE_100SPS     0x82 
#define ADS1256_DRATE_60SPS      0x72 
#define ADS1256_DRATE_50SPS      0x63 
#define ADS1256_DRATE_30SPS      0x53 
#define ADS1256_DRATE_25SPS      0x43 
#define ADS1256_DRATE_15SPS      0x33 
#define ADS1256_DRATE_10SPS      0x23 
#define ADS1256_DRATE_5SPS       0x13 
#define ADS1256_DRATE_2_5SPS     0x03

/******************ads1256引脚设置*******************/

#define RCC_ADS1256Reset                          	RCC_APB2Periph_GPIOB
#define GPIO_RCC_ADS1256Reset_PORT                  GPIOB    
#define GPIO_RCC_ADS1256Reset                       GPIO_Pin_11

#define RCC_ADS1256DRDY                             RCC_APB2Periph_GPIOB
#define GPIO_ADS1256DRDY_PORT                       GPIOB  
#define GPIO_ADS1256DRDY                            GPIO_Pin_10


/******************ads1256函数*******************/

void ADS1256_GPIO_init(void);                                       //DRDY和RESET初始化
void ADS1256_Init(void);                                            //ADS1256初始化                                               //
unsigned int ADS1256ReadData(void);                                 //ADS1256读数据
void ADS1256WREG(unsigned char regaddr,unsigned char databyte);     //设置通道
unsigned int ADS_sum(unsigned char channel);                        //ADS1256反馈测量数据
long double GetDistance(int iq);                               		//处理AD1256反馈数据
#endif




