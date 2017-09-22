#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/unistd.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-bank-p.h>
#include <linux/device.h> 
#include <asm/io.h> 
#include <linux/irq.h> 
#include <asm/irq.h>
#include <linux/irqflags.h> 
#include <linux/irqreturn.h> 
#include <linux/irqnr.h> 
#include <linux/interrupt.h> 
#include <asm/signal.h> 
#include <asm-generic/siginfo.h>

#define DEVICE_NAME "gpios"

#define GPPCON *((volatile unsigned long *)S3C64XX_GPPCON)
#define GPPDAT *((volatile unsigned long *)S3C64XX_GPPDAT)
#define EINT78CON *((volatile unsigned long *)S3C64XX_EINT78CON)
#define EINT78MASK *((volatile unsigned long *)S3C64XX_EINT78MASK)
#define EINT78PEND *((volatile unsigned long *)S3C64XX_EINT78PEND)

static struct fasync_struct *async; //声明fasync_struct 

volatile unsigned long VIC1ADDRESS;

void init_GPIO_INT();

static int data_fasync(int fd, struct file *filp, int mode)  
{  
    printk("application  fasync!\n");  
    return fasync_helper(fd, filp, mode, &async);         //注册上层调用进程的信息，上层调用fcntl设置FASYNC会调用这个系统调用  
}  


static long s3c6410_gppio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch(cmd) {
		unsigned tmp;
	case 0:
	case 1:
          
		tmp = readl(S3C64XX_GPPDAT);
            
		if(cmd==0) //close light
                  { 
			tmp &= (~(1<<arg));
                  }
		else  //open light
                  { 
			tmp |= (1<<arg);
                  }

                writel(tmp,S3C64XX_GPPDAT);

		printk (DEVICE_NAME": %d %d\n", arg, cmd);
		return 0;
	default:
		return -EINVAL;
	}
}

static struct file_operations dev_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= s3c6410_gppio_ioctl,
	.fasync = data_fasync,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};

static irqreturn_t exirq_get0(int irq,void *dev_id){
	printk("irq=%d\r\n",irq);
	printk("nrf24l01's spi0 interrupt is invoked\r\r");
	kill_fasync(&async, SIGIO, POLL_IN);  //向打开设备文件的进程发出SIGIO信号
	// EINT78PEND &= 0xFEFFFFFF;
	EINT78PEND &= 0xFDFFFFFF;
	return IRQ_HANDLED;
}


static irqreturn_t exirq_get1(int irq,void *dev_id){
	printk("irq=%d\r\n",irq);
	printk("nrf24l01's spi1 interrupt is invoked\r\r");
	kill_fasync(&async, SIGIO, POLL_IN);  //向打开设备文件的进程发出SIGIO信号
	// EINT78PEND &= 0xF7FFFFFF;
	EINT78PEND &= 0xEFFFFFFF;
	return IRQ_HANDLED;
}

static int __init dev_init(void)
{
	int ret;
        
       unsigned tmp;

       //gpp1,8 pull up
	tmp = readl(S3C64XX_GPPPUD);
	tmp &= (0xCFFF3);
	tmp |= 0x20008;
	writel(tmp,S3C64XX_GPPPUD);

	//gpp1,8 output mode
	tmp =readl(S3C64XX_GPPCON);
	tmp &= (0xCFFF3);
	tmp |= 0x10004;
	writel(tmp,S3C64XX_GPPCON);
	
	//gpp1,8 output 0
	tmp = __raw_readl(S3C64XX_GPPDAT);
	tmp |= 0x102;
	writel(tmp,S3C64XX_GPPDAT);  

	ret = misc_register(&misc);

	init_GPIO_INT();

	int reqIrq0;
	reqIrq0 = request_irq(IRQ_EINT_GROUP(8,9),exirq_get0,IRQF_TRIGGER_FALLING|IRQF_SHARED,"nrf24l01 SPI0",10005);
	printk("spi0 reqIrq=%d\r\n",reqIrq0);

	int reqIrq1;
	reqIrq1 = request_irq(IRQ_EINT_GROUP(8,12),exirq_get1,IRQF_TRIGGER_FALLING|IRQF_SHARED,"nrf24l01 SPI1",10007);
	printk("spi1 reqIrq=%d\r\n",reqIrq1);

	printk ("\n@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	printk (DEVICE_NAME"\tinitialized\n");
	printk ("\n@@@@@@@@@@@@@@@@@@@@@@@@@@\n");

	return ret;
}

static void __exit dev_exit(void)
{
	misc_deregister(&misc);
}


void init_GPIO_INT(){

	printk("init irq start\n");

	printk("%s\n", GPPCON);

	//配置GPP9,GPP12为外部中断,测试通过没问题
	GPPCON &= 0xFCF3FFFF;
	GPPCON |= 0x030C0000;


	printk("配置GPP9,GPP12为外部中断\n");
	printk("%X\n",GPPCON);

	//配置9,12下降沿触发,配置没问题，看到底哪种
	EINT78CON &= 0x00FFFFFF;
	// EINT78CON |= 0x22000000;
	EINT78CON |= 0x00000000;

	printk("配置下降沿触发\n");
	printk("%X\n",EINT78CON);

	EINT78MASK &= 0x00000000;
	EINT78MASK |= 0xDBFFFFFF;

	printk("解除掩码\n");
	printk("%X\n",EINT78MASK);

	printk("初始化中断\n");
	// EINT78PEND &= 0xFEFFFFFF;
	// EINT78PEND &= 0xF7FFFFFF;
	EINT78PEND &= 0xFDFFFFFF;
	EINT78PEND &= 0xEFFFFFFF;

}




module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("xdCao");