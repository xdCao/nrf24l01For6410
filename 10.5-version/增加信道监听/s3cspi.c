#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <linux/ioctl.h>
#include <signal.h>
 

typedef unsigned char uint8 ;

#define TX_PLOAD_WIDTH 32 // 20 uint8s TX payload

#define RX_PLOAD_WIDTH 32 // 20 uint8s TX payload

#define DYNPD			0x1C
#define FEATRUE			0x1D

//NRF24L01寄存器指令
#define READ_REG 0x00 // 读寄存器指令
#define WRITE_REG 0x20 // 写寄存器指令
#define RD_RX_PLOAD 0x61 // 读取接收数据指令
#define WR_TX_PLOAD 0xA0 // 写待发数据指令
#define FLUSH_TX 0xE1 // 冲洗发送 FIFO指令
#define FLUSH_RX 0xE2 // 冲洗接收 FIFO指令
#define REUSE_TX_PL 0xE3 // 定义重复装载数据指令
#define NOP 0xFF // 保留


//SPI(nRF24L01)寄存器地址
#define CONFIG 0x00 // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA 0x01 // 自动应答功能设置
#define EN_RXADDR 0x02 // 可用信道设置
#define SETUP_AW 0x03 // 收发地址宽度设置
#define SETUP_RETR 0x04 // 自动重发功能设置
#define RF_CH 0x05 // 工作频率设置
#define RF_SETUP 0x06 // 发射速率、功耗功能设置
#define STATUS 0x07 // 状态寄存器
#define OBSERVE_TX 0x08 // 发送监测功能
#define CD 0x09 // 地址检测 
#define RX_ADDR_P0 0x0A // 频道0接收数据地址
#define RX_ADDR_P1 0x0B // 频道1接收数据地址
#define RX_ADDR_P2 0x0C // 频道2接收数据地址
#define RX_ADDR_P3 0x0D // 频道3接收数据地址
#define RX_ADDR_P4 0x0E // 频道4接收数据地址
#define RX_ADDR_P5 0x0F // 频道5接收数据地址
#define TX_ADDR 0x10 // 发送地址寄存器
#define RX_PW_P0 0x11 // 接收频道0接收数据长度
#define RX_PW_P1 0x12 // 接收频道0接收数据长度
#define RX_PW_P2 0x13 // 接收频道0接收数据长度
#define RX_PW_P3 0x14 // 接收频道0接收数据长度
#define RX_PW_P4 0x15 // 接收频道0接收数据长度
#define RX_PW_P5 0x16 // 接收频道0接收数据长度
#define FIFO_STATUS 0x17 // FIFO栈入栈出状态寄存器设置
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define LEVEL_1  0x26 //0dbm
#define LEVEL_2  0x24 //-6dbm
#define LEVEL_3  0x22 //-12dbm
#define LEVEL_4  0x20 //-18dbm

#define DEVFILE "/dev/gpios"

static void pabort(const char *s)
{
	perror(s);
	abort();
}


void init_NRF24L01(int fd,int m_fd,int cs);
void set_rxmode(int fd,int cs);
void read_reg(int fd);
int SPI_Transfer(int fd, uint8_t *TxBuf, uint8_t *RxBuf, int len);
int send_packet(int fd,int m_fd,int cs,uint8_t *tx_buf,uint8_t level);
int SPI_Read(int fd,uint8_t *RxBuf, int len);
int SPI_Write(int fd,uint8_t *TxBuf, int len);
void recv_packet(int fd,int m_fd,uint8_t *rxbuf,int cs);
void sig_handler(int sig);
int listen_carrier(int fd,int cs);
int init();

static const char *device0 = "/dev/spidev0.0";
static const char *device1 = "/dev/spidev1.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

int fd0;
int fd1;
int m_fd;
uint8_t recvBuf0[33]={0,};
uint8_t recvBuf1[33]={0,};

void sig_handler(int sig)  
{  
    if(sig == SIGIO)  
    {  
        printf("Receive io signal from kernel!\n");  

		uint8_t rdsta0[]={READ_REG+STATUS,0};
		uint8_t rdstaTmp0[2]={0, };
		SPI_Transfer(fd0,rdsta0,rdstaTmp0,2);
		if (rdstaTmp0[1]==0x40)
		{
			printf("spi0spi0spi0spi0spi0spi0spi0spi0spi0spi0spi0spi0收到数据\n");
			recv_packet(fd0,m_fd,recvBuf0,0);
	        uint8_t rdLen[]={READ_REG+0x60,0};
			uint8_t rdLenTmp[2]={0, };
			SPI_Transfer(fd0,rdLen,rdLenTmp,2);
			int i;
			for(i=0;i<rdLenTmp[1]+1;i++){
				printf("%c ", recvBuf0[i]);
			}
			printf("\n");

			uint8_t rflush[]={FLUSH_RX,0xFF};
			uint8_t rflushTmp[2]={0, };
			SPI_Transfer(fd0,rflush,rflushTmp,2);

			printf("write status:\n");
			uint8_t sta[]={WRITE_REG+STATUS,0xFF};
			uint8_t staTmp[2]={0, };
			SPI_Transfer(fd0,sta,staTmp,2);
		}


        uint8_t rdsta1[]={READ_REG+STATUS,0};
		uint8_t rdstaTmp1[2]={0, };
		SPI_Transfer(fd1,rdsta1,rdstaTmp1,2);
		if (rdstaTmp1[1]==0x40)
		{
			printf("spi1spi1spi1spi1spi1spi1spi1spi1spi1spi1spi1spi1spi1收到数据\n");
			recv_packet(fd1,m_fd,recvBuf1,1);
	        uint8_t rdLen[]={READ_REG+0x60,0};
			uint8_t rdLenTmp[2]={0, };
			SPI_Transfer(fd1,rdLen,rdLenTmp,2);
			int i;
			for(i=0;i<rdLenTmp[1]+1;i++){
				printf("%c ", recvBuf1[i]);
			}
			printf("\n");

			uint8_t rflush[]={FLUSH_RX,0xFF};
			uint8_t rflushTmp[2]={0, };
			SPI_Transfer(fd1,rflush,rflushTmp,2);

			printf("write status:\n");
			uint8_t sta[]={WRITE_REG+STATUS,0xFF};
			uint8_t staTmp[2]={0, };
			SPI_Transfer(fd1,sta,staTmp,2);
			
	
		}


        
    }  
} 


int main(int argc, char *argv[])
{



	int ret = 0;

	ret=init();

	uint8_t wr0[]={WR_TX_PLOAD,
		 0x11,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,
		 0x22,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,
		 0x33,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,
		 0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		};

	while(1){

			// ret=send_packet(fd0,m_fd,0,wr0,LEVEL_1);

			// set_rxmode(fd0,0);

			// usleep(10);

			// ret=send_packet(fd0,m_fd,0,wr0,LEVEL_4);

			// set_rxmode(fd0,0);

			// usleep(10);
	
			// ret=send_packet(fd1,m_fd,1,wr0,LEVEL_2);

			// set_rxmode(fd1,1);

			// usleep(10);

			// ret=send_packet(fd1,m_fd,1,wr0,LEVEL_3);

			// set_rxmode(fd1,1);

			// usleep(10);
			listen_carrier(fd0,0);

			usleep(30000);

			listen_carrier(fd1,1);

			usleep(30000);
			
	}

	close(fd0);
	close(fd1);

	return ret;
}

int init(){

	int ret=0;

	signal(SIGIO, sig_handler); 
 
	fd0 = open(device0, O_RDWR);
	fd1 = open(device1, O_RDWR);

	m_fd = open(DEVFILE,O_RDWR);

	fcntl(m_fd, F_SETOWN, getpid());  
    fcntl(m_fd, F_SETFL, fcntl(m_fd, F_GETFL) | FASYNC);


	ret = ioctl(fd0, SPI_IOC_WR_MODE, &mode);
	ret = ioctl(fd0, SPI_IOC_RD_MODE, &mode);

	ret = ioctl(fd0, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ret = ioctl(fd0, SPI_IOC_RD_BITS_PER_WORD, &bits);

	ret = ioctl(fd0, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	ret = ioctl(fd0, SPI_IOC_RD_MAX_SPEED_HZ, &speed);


	ret = ioctl(fd1, SPI_IOC_WR_MODE, &mode);
	ret = ioctl(fd1, SPI_IOC_RD_MODE, &mode);

	ret = ioctl(fd1, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ret = ioctl(fd1, SPI_IOC_RD_BITS_PER_WORD, &bits);

	ret = ioctl(fd1, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	ret = ioctl(fd1, SPI_IOC_RD_MAX_SPEED_HZ, &speed);




	init_NRF24L01(fd0,m_fd,0);
	init_NRF24L01(fd1,m_fd,1);

	set_rxmode(fd0,0);
	set_rxmode(fd1,1);

	return ret;
}

int listen_carrier(int fd,int cs){
	if (cs==0)
	{
		printf("进行信道监听00000000000000000000000000000000000000:\n");
	}
	else if(cs==1)
	{
		printf("进行信道监听11111111111111111111111111111111111111:\n");
	}
	uint8_t rdCD[]={CD,0};
	uint8_t rdCDTmp[2]={0, };
	SPI_Transfer(fd,rdCD,rdCDTmp,2);
	if (rdCDTmp[1]==0x01)
	{
		return 1;
	}else if (rdCDTmp[1]==0x00)
	{
		return 0;
	}
}


void init_NRF24L01(int fd,int m_fd,int cs){

	if(cs==0){

		ioctl(m_fd, 0, 1);//拉低

		uint8_t dynpd[]={WRITE_REG+DYNPD,0x01};
		uint8_t dynpdTmp[2]={0, };
		SPI_Transfer(fd,dynpd,dynpdTmp,2);

		uint8_t feature[]={WRITE_REG+FEATRUE,0x07};
		uint8_t featureTmp[2]={0, };
		SPI_Transfer(fd,feature,featureTmp,2);
		
		printf("write TxAddr:\n");
		uint8_t wrTxAddr[]={WRITE_REG+TX_ADDR,0x34,0x43,0x10,0x10,0x01};
		uint8_t wrTxAddrTmp[6]={0, };
		SPI_Transfer(fd,wrTxAddr,wrTxAddrTmp,6);

		printf("write RxAddr:\n");
		uint8_t wrRxAddr[]={WRITE_REG+RX_ADDR_P0,0x34,0x43,0x10,0x10,0x01};
		uint8_t wrRxAddrTmp[6]={0, };
		SPI_Transfer(fd,wrRxAddr,wrRxAddrTmp,6);

		printf("write address width:\n");
		uint8_t aw[]={WRITE_REG+SETUP_AW,0x3};
		uint8_t awTmp[2]={0, };
		SPI_Transfer(fd,aw,awTmp,2);

		printf("write EN_AA:\n");
		uint8_t wrENAA[]={WRITE_REG+EN_AA,0x01};
		uint8_t wrENAATmp[2]={0, };
		SPI_Transfer(fd,wrENAA,wrENAATmp,2);

		printf("write EN_RXADDR:\n");
		uint8_t wrEnRxAddr[]={WRITE_REG+EN_RXADDR,0x01};
		uint8_t wrEnRxAddrTmp[2]={0, };
		SPI_Transfer(fd,wrEnRxAddr,wrEnRxAddrTmp,2);

		printf("write RF_CH:\n");
		uint8_t wrRfCH[]={WRITE_REG+RF_CH,95};
		uint8_t wrRfCHTmp[2]={0, };
		SPI_Transfer(fd,wrRfCH,wrRfCHTmp,2);

		printf("write RX_PW_P0:\n");
		uint8_t wrRXPWP0[]={WRITE_REG+RX_PW_P0,32};
		uint8_t wrRXPWP0Tmp[2]={0, };
		SPI_Transfer(fd,wrRXPWP0,wrRXPWP0Tmp,2);

		printf("write rf_setup:\n");
		uint8_t wrSetup[]={WRITE_REG+RF_SETUP,0x26};
		uint8_t wrSetupTmp[2]={0, };
		SPI_Transfer(fd,wrSetup,wrSetupTmp,2);

		printf("write return:\n");
		uint8_t retr[]={WRITE_REG+SETUP_RETR,0x0f};
		uint8_t retrTmp[2]={0, };
		SPI_Transfer(fd,retr,retrTmp,2);

		uint8_t rflush[]={FLUSH_RX,0xFF};
		uint8_t rflushTmp[2]={0, };
		SPI_Transfer(fd,rflush,rflushTmp,2);

		printf("write status:\n");
		uint8_t sta[]={WRITE_REG+STATUS,0xFF};
		uint8_t staTmp[2]={0, };
		SPI_Transfer(fd,sta,staTmp,2);

		ioctl(m_fd, 1, 1);

		read_reg(fd);

	}else if (cs==1){

		ioctl(m_fd, 0, 8);

		uint8_t dynpd[]={WRITE_REG+DYNPD,0x01};
		uint8_t dynpdTmp[2]={0, };
		SPI_Transfer(fd,dynpd,dynpdTmp,2);

		uint8_t feature[]={WRITE_REG+FEATRUE,0x07};
		uint8_t featureTmp[2]={0, };
		SPI_Transfer(fd,feature,featureTmp,2);
		
		printf("write TxAddr:\n");
		uint8_t wrTxAddr[]={WRITE_REG+TX_ADDR,0x51,0x53,0x35,0x45,0x22};
		uint8_t wrTxAddrTmp[6]={0, };
		SPI_Transfer(fd,wrTxAddr,wrTxAddrTmp,6);

		printf("write RxAddr:\n");
		uint8_t wrRxAddr[]={WRITE_REG+RX_ADDR_P0,0x51,0x53,0x35,0x45,0x22};
		uint8_t wrRxAddrTmp[6]={0, };
		SPI_Transfer(fd,wrRxAddr,wrRxAddrTmp,6);

		printf("write address width:\n");
		uint8_t aw[]={WRITE_REG+SETUP_AW,0x3};
		uint8_t awTmp[2]={0, };
		SPI_Transfer(fd,aw,awTmp,2);

		printf("write EN_AA:\n");
		uint8_t wrENAA[]={WRITE_REG+EN_AA,0x01};
		uint8_t wrENAATmp[2]={0, };
		SPI_Transfer(fd,wrENAA,wrENAATmp,2);

		printf("write EN_RXADDR:\n");
		uint8_t wrEnRxAddr[]={WRITE_REG+EN_RXADDR,0x01};
		uint8_t wrEnRxAddrTmp[2]={0, };
		SPI_Transfer(fd,wrEnRxAddr,wrEnRxAddrTmp,2);

		printf("write RF_CH:\n");
		uint8_t wrRfCH[]={WRITE_REG+RF_CH,60};
		uint8_t wrRfCHTmp[2]={0, };
		SPI_Transfer(fd,wrRfCH,wrRfCHTmp,2);

		printf("write RX_PW_P0:\n");
		uint8_t wrRXPWP0[]={WRITE_REG+RX_PW_P0,32};
		uint8_t wrRXPWP0Tmp[2]={0, };
		SPI_Transfer(fd,wrRXPWP0,wrRXPWP0Tmp,2);

		printf("write rf_setup:\n");
		uint8_t wrSetup[]={WRITE_REG+RF_SETUP,0x26};
		uint8_t wrSetupTmp[2]={0, };
		SPI_Transfer(fd,wrSetup,wrSetupTmp,2);

		printf("write return:\n");
		uint8_t retr[]={WRITE_REG+SETUP_RETR,0x0f};
		uint8_t retrTmp[2]={0, };
		SPI_Transfer(fd,retr,retrTmp,2);

		uint8_t rflush[]={FLUSH_RX,0xFF};
		uint8_t rflushTmp[2]={0, };
		SPI_Transfer(fd,rflush,rflushTmp,2);

		printf("write status:\n");
		uint8_t sta[]={WRITE_REG+STATUS,0xFF};
		uint8_t staTmp[2]={0, };
		SPI_Transfer(fd,sta,staTmp,2);

		ioctl(m_fd, 1, 8);

		read_reg(fd);
	}
}



void read_reg(int fd){

	printf("read TxAddr:\n");
	uint8_t rdTxAddr[]={READ_REG+TX_ADDR,0,0,0,0,0};
	uint8_t rdTxAddrTmp[6]={0, };
	SPI_Transfer(fd,rdTxAddr,rdTxAddrTmp,6);

	printf("read RxAddr:\n");
	uint8_t rdRxAddr[]={READ_REG+RX_ADDR_P0,0,0,0,0,0};
	uint8_t rdRxAddrTmp[6]={0, };
	SPI_Transfer(fd,rdRxAddr,rdRxAddrTmp,6);

	printf("read EN_AA:\n");
	uint8_t rdENAA[]={READ_REG+EN_AA,0};
	uint8_t rdENAATmp[2]={0, };
	SPI_Transfer(fd,rdENAA,rdENAATmp,2);

	printf("read EN_RXADDR:\n");
	uint8_t rdEnRxAddr[]={READ_REG+EN_RXADDR,0};
	uint8_t rdEnRxAddrTmp[2]={0, };
	SPI_Transfer(fd,rdEnRxAddr,rdEnRxAddrTmp,2);

	printf("read RF_CH:\n");
	uint8_t rdRfCH[]={READ_REG+RF_CH,0};
	uint8_t rdRfCHTmp[2]={0, };
	SPI_Transfer(fd,rdRfCH,rdRfCHTmp,2);

	printf("read RX_PW_P0:\n");
	uint8_t rdRXPWP0[]={READ_REG+RX_PW_P0,0};
	uint8_t rdRXPWP0Tmp[2]={0, };
	SPI_Transfer(fd,rdRXPWP0,rdRXPWP0Tmp,2);

	printf("read rf_setup:\n");
	uint8_t rdSetup[]={READ_REG+RF_SETUP,0};
	uint8_t rdSetupTmp[2]={0, };
	SPI_Transfer(fd,rdSetup,rdSetupTmp,2);

}



void set_rxmode(int fd,int cs){
	if (cs==0)
	{
		ioctl(m_fd, 0, 1);
	}else if(cs==1){
		ioctl(m_fd, 0, 8);
	}
	usleep(10);

	printf("write config recv:\n");
	uint8_t con[]={WRITE_REG+CONFIG,0x0b};
	uint8_t conTmp[2]={0, };
	SPI_Transfer(fd,con,conTmp,2);

	if (cs==0)
	{
		ioctl(m_fd, 1, 1);
	}else if (cs==1)
	{
		ioctl(m_fd, 1, 8);
	}

	usleep(6000);
}



void recv_packet(int fd,int m_fd,uint8_t *rxbuf,int cs){

	uint8_t rdData[33]={RD_RX_PLOAD,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	SPI_Transfer(fd,rdData,rxbuf,33);

}




int send_packet(int fd,int m_fd,int cs,uint8_t *tx_buf,uint8_t level){

	int ret=-1;

	if (cs==0)
	{
		ioctl(m_fd, 0, 1);
	}else if(cs==1){
		ioctl(m_fd, 0, 8);
	}
	usleep(10);

	printf("write rf_setup:\n");
	uint8_t wrSetup[]={WRITE_REG+RF_SETUP,level};
	uint8_t wrSetupTmp[2]={0, };
	SPI_Transfer(fd,wrSetup,wrSetupTmp,2);

	printf("read rf_setup 新的发射功率:\n");
	uint8_t rdSetup[]={READ_REG+RF_SETUP,0};
	uint8_t rdSetupTmp[2]={0, };
	SPI_Transfer(fd,rdSetup,rdSetupTmp,2);

	printf("write config send:\n");
	uint8_t con[]={WRITE_REG+CONFIG,0x1a};
	uint8_t conTmp[2]={0, };
	ret=SPI_Transfer(fd,con,conTmp,2);


	if(cs==0){
		printf("spi 0 发送数据\n");
		// uint8_t wr0[]={WR_TX_PLOAD,
		//  0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,
		//  0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,
		//  0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,
		//  0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		// };
		int i;
		for (i = 1; i < 32; i++)
		{
			if (i % 8 == 0)
				printf("\n");
			printf("0x%02X ", tx_buf[i]);
		}
		uint8_t wr0Tmp[33]={0, };
		ret=SPI_Transfer(fd,tx_buf,wr0Tmp,33);
	}else if(cs==1){
		printf("spi 1 发送数据\n");
		// uint8_t wr1[]={WR_TX_PLOAD,
		//  0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,
		//  0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,
		//  0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,
		//  0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b
		// };
		int i;
		for (i = 1; i < 32; i++)
		{
			if (i % 8 == 0)
				printf("\n");
			printf("0x%02X ", tx_buf[i]);
		}
		uint8_t wr1Tmp[33]={0, };
		ret=SPI_Transfer(fd,tx_buf,wr1Tmp,33);
		printf("是否发送成功？%d\n", ret);
	}
	usleep(10);

	

	if (cs==0)
	{
		ioctl(m_fd, 1, 1);
	}else if(cs==1){
		ioctl(m_fd, 1, 8);
	}

	usleep(200);

	printf("observe tx:\n");
	uint8_t obs[]={READ_REG+OBSERVE_TX,0};
	uint8_t obsTmp[2]={0, };
	SPI_Transfer(fd,obs,obsTmp,2);
	
	printf("read status:\n");
	uint8_t rdsta[]={READ_REG+STATUS,0};
	uint8_t rdstaTmp[2]={0, };
	ret=SPI_Transfer(fd,rdsta,rdstaTmp,2);

	if (cs==0)
	{
		ioctl(m_fd, 0, 1);
	}else if(cs==1){
		ioctl(m_fd, 0, 8);
	}
	usleep(10);

	printf("write status:\n");
	uint8_t sta[]={WRITE_REG+STATUS,0xFF};
	uint8_t staTmp[2]={0, };
	ret=SPI_Transfer(fd,sta,staTmp,2);

	uint8_t tflush[]={FLUSH_TX,0xFF};
	uint8_t tflushTmp[2]={0, };
	SPI_Transfer(fd,tflush,tflushTmp,2);

	if (cs==0)
	{
		ioctl(m_fd, 1, 1);
	}else if(cs==1){
		ioctl(m_fd, 1, 8);
	}

	usleep(10);


	return ret;

}


int SPI_Transfer(int fd,uint8_t *TxBuf, uint8_t *RxBuf, int len)
{
	int ret;

	struct spi_ioc_transfer tr ={
		.tx_buf = (unsigned long) TxBuf,
		.rx_buf = (unsigned long) RxBuf,
		.len =len,
		.cs_change=0,
		.delay_usecs = delay,
	};


	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	
	int i;
	printf("RxData:");
	for (i = 0; i < len; i++)
	{
		if (i % 8 == 0)
			printf("\n");
		printf("0x%02X ", RxBuf[i]);
	}

	printf("\n");

	return ret;
}


int SPI_Write(int fd,uint8_t *TxBuf, int len)
{
	int ret;

	ret = write(fd, TxBuf, len);
	if (ret < 0)
		pabort("SPI Write errorn");
	else
	{
		printf("write:\n");
		int i;
		for (i = 0; i < len; i++)
		{
			if (i % 8 == 0)
				printf("\n");
			printf("0x%02X ", TxBuf[i]);
		}
		printf("\n");
	}
	return ret;
}

int SPI_Read(int fd,uint8_t *RxBuf, int len)
{
	int ret;
	ret = read(fd, RxBuf, len);
	if (ret < 0)
		pabort("SPI Read errorn");
	else{
		printf("read:\n");
		int i;
		for (i = 0; i < len; i++)
		{
			if (i % 8 == 0)
				printf("\n");
			printf("0x%02X ", RxBuf[i]);
		}
		printf("\n");
	}
	return ret;
}






  

