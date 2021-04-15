/*
 * drivers/fan_speed_detect.c
 *
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/preempt.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/rmap.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <asm/siginfo.h>
#include <asm/signal.h>
#include <linux/clk/sunxi.h>
#include <mach/sunxi-smc.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <mach/gpio.h>


#ifndef strict_strtoul
#define strict_strtoul kstrtoul
#endif

#define FSD_DEV_MAJOR               (235)
#define FSD_DEV_MINOR               (0)
#define GPIO_PA_INT_REG_BASE        (0x01c20800)
#define GPIO_PA_INT_REG_SIZE        (0x240)
#define REG_PA_CFG0                 (0x0)
#define REG_PA_CFG1                 (0x4)
#define REG_PA_CFG2                 (0x8)
#define REG_PD_CFG1                 (0x4 + 0x24*3)
#define REG_PA_DATA                 (0x10)
#define REG_PG_CFG0                 (0xd8)
#define REG_PG_CFG1                 (0xdc)
#define REG_PG_DATA                 (0xe8)
#define REG_PG_DRV0                 (0xec)
#define REG_PG_PULL                 (0xf4)
#define REG_PG_INT_CFG0             (0x220)
#define REG_PG_INT_CFG1             (0x224)
#define REG_PG_INT_CTL              (0x230)
#define REG_PG_INT_STA              (0x234)
#define REG_PG_INT_DEB              (0x238)
#define REG_PA_INT_CFG0             (0x200)
#define REG_PA_INT_CFG1             (0x204)
#define REG_PA_INT_CFG2             (0x208)
#define REG_PA_INT_CFG3             (0x20c)
#define REG_PA_INT_CTL              (0x210)
#define REG_PA_INT_STA              (0x214)
#define REG_PA_INT_DEB              (0x218)
#define HW_PWM_REG_BASE             (0x01c21400)
#define HW_PWM_REG_SIZE             (0xc)
#define REG_PWM_CH_CTRL             (0x0)
#define REG_PWM_CH0_PERIOD          (0x4)
#define REG_PWM_CH1_PERIOD          (0x8)
#define REG_PL_CFG1                 (0x4)

#define PWM_DUTY_DEFAULT            (90)    //* percentage of pwm high level.
#define GPIO_PWM_PERIOD_CNT         (50)

#define FAN_COUNT                   (2)

#define HW_PWM_CLOCK                (24*1000*1000)
#define HW_PWM_ENTIRE_CYCLE_NUM     (960) //25K = 24M/960
#define get_bit(x,y)                ((x) >> (y)&1)

struct fsd_dev
{
    struct cdev        cdev;            /* char device struct */
    struct device*     dev;             /* ptr to class device struct */
    struct class*      class;           /* class for auto create device node */
    struct resource*   regs;            /* registers resource */
    volatile char*     regsaddr ;       /* registers address */
    volatile char*     regsaddr_r ;     /* cpus gpio registers address */
    struct resource*   hw_pwm_regs;     /* hw pwm registers resource */
    volatile char*     hw_pwm_regsaddr; /* hardware pwm registers address */

    struct timer_list  fan_detect_timer;
    unsigned int       fan0_intr_cnt;
    unsigned int       fan1_intr_cnt;
    unsigned int       fan2_intr_cnt;
    unsigned int       fan3_intr_cnt;
    /* Fan speed here is raw speed not real speed, should be multiplied by a integer
     * in user space according to fan's spec. */
    unsigned int       fan0_speed;
    unsigned int       fan1_speed;
    unsigned int       fan2_speed;
    unsigned int       fan3_speed;
    int                fan_detect_open;
    int                fan0_virq;
    int                fan1_virq;
    int                fan2_virq;
    int                fan3_virq;

    int                use_hw_pwm;
    int                hw_pwm_duty;     /* from 0 to 100 */
    int                hw_pwm_cycles;   /* will be set to 1000 */

    struct hrtimer     pwm_timer;
    unsigned long      pwm_timer_period_us;
    int                pwm_period_cnt;        /* pwm period */
    int                pwm_duty[FAN_COUNT];   /* percentage of the pwm high level */
    int                pwm_duty_cnt[FAN_COUNT];    /* how long time the pwm high level */
    int                pwm_time_cnt[FAN_COUNT];    /* counter for the timer */

    int                fsd_dev_major;
    int                fsd_dev_minor;
    int                hw_version;
};

static struct fsd_dev* fsd_devp = NULL;

static int get_gpio_pg4_input_value(struct fsd_dev* p)
{
    unsigned int tmp;

    // set PG Config0 register for PG4.
    tmp = readl(p->regsaddr+REG_PG_CFG0);
    tmp &= 0xfff0ffff;    // set PG4 as input pin.
    writel(tmp, (p->regsaddr+REG_PG_CFG0));

    // set PG pull register for PG4 to pull down.
    tmp = readl(p->regsaddr+REG_PG_PULL);
    tmp &= 0xfffffcff;
    tmp |= 0x00000200;
    writel(tmp, (p->regsaddr+REG_PG_PULL));

    // set PG Config0 register for PG4.
    tmp = readl(p->regsaddr+REG_PG_DRV0);
    tmp &= 0xfffffcff;    // set PG4 drive level to level 0.
    writel(tmp, (p->regsaddr+REG_PG_DRV0));

    udelay(100);

    // read PG4 input value from PG data register.
    tmp = readl(p->regsaddr+REG_PG_DATA);
    tmp &= 0x10;

    if(tmp != 0)
        return 1;
    else
        return 0;
}

static int get_gpio_pg5_input_value(struct fsd_dev* p)
{
    unsigned int tmp;

    // set PG Config0 register for PG5.
    tmp = readl(p->regsaddr+REG_PG_CFG0);
    tmp &= 0xff0fffff;    // set pg5 as input pin.
    writel(tmp, (p->regsaddr+REG_PG_CFG0));

    // set PG pull register for PG5 to pull down.
    tmp = readl(p->regsaddr+REG_PG_PULL);
    tmp &= 0xfffff3ff;
    tmp |= 0x00000800;
    writel(tmp, (p->regsaddr+REG_PG_PULL));

    // set PG Config0 register for PG5.
    tmp = readl(p->regsaddr+REG_PG_DRV0);
    tmp &= 0xfffff3ff;    // set pg5 drive level to level 0.
    writel(tmp, (p->regsaddr+REG_PG_DRV0));

    udelay(100);

    // read pg5 input value from PG data register.
    tmp = readl(p->regsaddr+REG_PG_DATA);
    tmp &= 0x20;

    if(tmp != 0)
        return 1;
    else
        return 0;
}

static int fsd_ext_irq_enable(struct fsd_dev* devp)
{
    unsigned int tmp;

    // set PA Config2 register for PA17 and PA21 as external interrupt signal.
    tmp = readl(devp->regsaddr+REG_PA_CFG2);
    tmp &= 0xff0fff0f;    // clean config bits for PA17 and PA21
    tmp |= 0x600060;      // config PA17 and PA21 as EXT INT signal;
    writel(tmp, (devp->regsaddr+REG_PA_CFG2));

    // set PA INT Config2 register for PA17 and PA21 as external interrupt signal.
    tmp = readl(devp->regsaddr+REG_PA_INT_CFG2);
    tmp &= 0xff0fff0f;    // clean config bits for PA17 and PA21
    tmp |= 0x100010;      // set up PA17 and PA21 EXT INT triggered by negtive edge;
    writel(tmp, (devp->regsaddr+REG_PA_INT_CFG2));

    // set sample resolution for ext intr;
    tmp = readl(devp->regsaddr+REG_PA_INT_DEB);
    tmp &= 0x00;
    writel(tmp, (devp->regsaddr+REG_PA_INT_DEB));

    // enable ext intr.
    tmp = readl(devp->regsaddr+REG_PA_INT_CTL);
    tmp |= 0x00220000;    // enable PA17 and PA21 ext intr.
    writel(tmp, (devp->regsaddr+REG_PA_INT_CTL));

    if (devp->hw_version == 3)
    {
        // set PG1 and PG10 as external interrupt signal.
        tmp = readl(devp->regsaddr+REG_PG_CFG0);
        tmp &= 0xffffff0f;
        tmp |= 0x60;
        writel(tmp, (devp->regsaddr+REG_PG_CFG0));
        tmp = readl(devp->regsaddr+REG_PG_CFG1);
        tmp &= 0xfffff0ff;
        tmp |= 0x600;
        writel(tmp, (devp->regsaddr+REG_PG_CFG1));

        // set PG1 and PG10 as external interrupt signal.
        tmp = readl(devp->regsaddr+REG_PG_INT_CFG0);
        tmp &= 0xffffff0f;
        tmp |= 0x10;
        writel(tmp, (devp->regsaddr+REG_PG_INT_CFG0));
        tmp = readl(devp->regsaddr+REG_PG_INT_CFG1);
        tmp &= 0xfffff0ff;
        tmp |= 0x100;
        writel(tmp, (devp->regsaddr+REG_PG_INT_CFG1));

        // set sample resolution for ext intr;
        tmp = readl(devp->regsaddr+REG_PG_INT_DEB);
        tmp &= 0x00;
        writel(tmp, (devp->regsaddr+REG_PG_INT_DEB));

        // enable ext intr.
        tmp = readl(devp->regsaddr+REG_PG_INT_CTL);
        tmp |= 0x402;
        writel(tmp, (devp->regsaddr+REG_PG_INT_CTL));
    }
    return 0;
}

static int fsd_ext_irq_disable(struct fsd_dev* devp)
{
    unsigned int tmp;

    // set PA Config2 register for PA17 and PA21 as external interrupt signal.
    tmp = readl(devp->regsaddr+REG_PA_CFG2);
    tmp &= 0xff0fff0f;    // clean config bits for PA17 and PA21
    tmp |= 0x700070;      // set up PA17 and PA21 as IO disabled;
    writel(tmp, (devp->regsaddr+REG_PA_CFG2));

    // disable ext intr.
    tmp = readl(devp->regsaddr+REG_PA_INT_CTL);
    tmp &= 0xffddffff;   // disable PA17 and PA21 ext intr.
    writel(tmp, (devp->regsaddr+REG_PA_INT_CTL));

    if (devp->hw_version == 3)
    {
        tmp = readl(devp->regsaddr+REG_PG_CFG0);
        tmp &= 0xffffff0f;
        tmp |= 0x70;
        writel(tmp, (devp->regsaddr+REG_PG_CFG0));
        tmp = readl(devp->regsaddr+REG_PG_CFG1);
        tmp &= 0xfffff0ff;
        tmp |= 0x700;
        writel(tmp, (devp->regsaddr+REG_PG_CFG1));

        // disable ext intr.
        tmp = readl(devp->regsaddr+REG_PG_INT_CTL);
        tmp &= 0xfffffbfd;
        writel(tmp, (devp->regsaddr+REG_PG_INT_CTL));
    }
    return 0;
}

/*
 * ioremap and request iomem
 */
static int fsd_iomem_register(struct fsd_dev* devp)
{
#if 0
    char*            addr;
    int              ret;
    struct resource* res;

//    ret = check_mem_region(GPIO_PA_INT_REG_BASE, GPIO_PA_INT_REG_SIZE);
    res = request_mem_region(GPIO_PA_INT_REG_BASE, GPIO_PA_INT_REG_SIZE, "gpip_pa_int_cfg regs");
    if (res == NULL)
    {
        pr_err("%s: cannot reserve region for register\n", __func__);
        goto err;
    }
    devp->regs = res;

    addr = ioremap(GPIO_PA_INT_REG_BASE, GPIO_PA_INT_REG_SIZE);
    if (!addr)
    {
        printk("%s: cannot map region for register\n", __func__);
        goto err;
    }

    devp->regsaddr = addr;

    ret = check_mem_region(HW_PWM_REG_BASE, HW_PWM_REG_SIZE);
    res = request_mem_region(HW_PWM_REG_BASE, HW_PWM_REG_SIZE, "hw_pwm regs");
    if (res == NULL)
    {
        pr_err("%s: cannot reserve region for register\n", __func__);
        goto err;
    }
    devp->hw_pwm_regs = res;

    addr = ioremap(HW_PWM_REG_BASE, HW_PWM_REG_SIZE);
    if (!addr)
    {
        printk("%s: cannot map region for register\n", __func__);
        goto err;
    }

    devp->hw_pwm_regsaddr = addr;

    return 0;

err:
    if (devp->regs)
    {
        release_resource(devp->regs);
        devp->regs = NULL;
    }

    if(devp->hw_pwm_regs)
    {
        release_resource(devp->hw_pwm_regs);
    }

    return -1;
#else
    devp->regsaddr = (volatile char *)0xf1c20800;
    devp->regsaddr_r = (volatile char *)0xf1f02c00;
    /* use cpus hw pwm */
    devp->hw_pwm_regsaddr = (volatile char *)0xf1f03800;

    return 0;
#endif
}

/*
 * unmap/release iomem
 */
static void fsd_iomem_unregister(struct fsd_dev* devp)
{
#if 0
    iounmap(devp->regsaddr);
    devp->regsaddr = NULL;
    if (devp->regs)
    {
        release_resource(devp->regs);
        devp->regs = NULL;
    }

    iounmap(devp->hw_pwm_regsaddr);
    devp->hw_pwm_regsaddr = NULL;
    if (devp->hw_pwm_regs)
    {
        release_resource(devp->hw_pwm_regs);
        devp->hw_pwm_regs = NULL;
    }
#endif
}

/*
 * interrupt service routine
 * To wake up wait queue
 */
static irqreturn_t fsd_irq_handle(int irq, void *dev_id)
{
    struct fsd_dev* p = (struct fsd_dev*)dev_id;

    if(irq == p->fan0_virq)
        p->fan0_intr_cnt++;
    if(irq == p->fan1_virq)
        p->fan1_intr_cnt++;
    if(irq == p->fan2_virq)
        p->fan2_intr_cnt++;
    if(irq == p->fan3_virq)
        p->fan3_intr_cnt++;

    return IRQ_HANDLED;
}


void fan_detect_timer_handler(unsigned long arg)
{
    struct fsd_dev* p = (struct fsd_dev*)arg;

    p->fan0_speed = p->fan0_intr_cnt;
    p->fan1_speed = p->fan1_intr_cnt;
    p->fan2_speed = p->fan2_intr_cnt;
    p->fan3_speed = p->fan3_intr_cnt;
    p->fan0_intr_cnt = 0;
    p->fan1_intr_cnt = 0;
    p->fan2_intr_cnt = 0;
    p->fan3_intr_cnt = 0;

    p->fan_detect_timer.expires = jiffies + 1*HZ;
    p->fan_detect_timer.data    = (unsigned long)p;
    p->fan_detect_timer.function = fan_detect_timer_handler;
    add_timer(&p->fan_detect_timer);
}

void fan_detect_init(struct fsd_dev* p)
{
    int err;
    if(p->fan_detect_open == 0)
    {
        p->fan0_virq = gpio_to_irq(GPIOA(17));
        err = devm_request_irq(p->dev, p->fan0_virq, fsd_irq_handle, IRQF_TRIGGER_FALLING, "PA17_EINT", p);
        if (err)
            pr_err("Request fan irq %d error.\n", p->fan0_virq);
        p->fan1_virq = gpio_to_irq(GPIOA(21));
        err = devm_request_irq(p->dev, p->fan1_virq, fsd_irq_handle, IRQF_TRIGGER_FALLING, "PA21_EINT", p);
        if (err)
            pr_err("Request fan irq %d error.\n", p->fan1_virq);
        if (p->hw_version == 3)
        {
            p->fan2_virq = gpio_to_irq(GPIOG(1));
            err = devm_request_irq(p->dev, p->fan2_virq, fsd_irq_handle, IRQF_TRIGGER_FALLING, "PG1_EINT", p);
            if (err)
                pr_err("Request fan irq %d error.\n", p->fan2_virq);
            p->fan3_virq = gpio_to_irq(GPIOG(10));
            err = devm_request_irq(p->dev, p->fan3_virq, fsd_irq_handle, IRQF_TRIGGER_FALLING, "PG10_EINT", p);
            if (err)
                pr_err("Request fan irq %d error.\n", p->fan3_virq);
        }
        p->fan_detect_open = 1;

        //* set a timer to generate a detect task.
        init_timer(&fsd_devp->fan_detect_timer);
        fsd_devp->fan_detect_timer.expires = jiffies + 1*HZ;
        fsd_devp->fan_detect_timer.data    = (unsigned long)fsd_devp;
        fsd_devp->fan_detect_timer.function = fan_detect_timer_handler;
        add_timer(&fsd_devp->fan_detect_timer);

        fsd_ext_irq_enable(p);
    }
}

void fan_detect_release(struct fsd_dev* p)
{
    if(p->fan_detect_open == 1)
    {
        fsd_ext_irq_disable(p);

        devm_free_irq(p->dev, p->fan0_virq, p);
        devm_free_irq(p->dev, p->fan1_virq, p);
        if (p->hw_version == 3)
        {
            devm_free_irq(p->dev, p->fan2_virq, p);
            devm_free_irq(p->dev, p->fan3_virq, p);
        }
        del_timer(&fsd_devp->fan_detect_timer);
        p->fan_detect_open = 0;
    }
}

static void set_gpio_value(struct fsd_dev* dev, int port, int index, int value)
{
    unsigned int tmp;

    if (port != 0 && port != 3)
        BUG();
    if (value != 0 && value != 1)
        BUG();

    tmp = readl(dev->regsaddr + REG_PA_DATA + port * 0x24);
    if (value == 1)
        tmp |= 1 << index;
    else
        tmp &= ~(1 << index);
    writel(tmp, (dev->regsaddr + REG_PA_DATA + port * 0x24));
}

static void set_fan_output(struct fsd_dev* dev, int index, int value)
{
    switch (index) {
    case 0:
        set_gpio_value(dev, 0, 6, value); //PA6
        break;

    case 1:
        set_gpio_value(dev, 3, 8, value); //PD8
        break;

    default:
        BUG();
    }
}

static void handle_pwm(struct fsd_dev* p, int index)
{
    int i = index;

    if (p->pwm_time_cnt[i] == p->pwm_duty_cnt[i])
        set_fan_output(p, i, 0);
    else if (p->pwm_time_cnt[i] == 0)
        set_fan_output(p, i, 1);

    p->pwm_time_cnt[i]++;
    if(p->pwm_time_cnt[i] >= p->pwm_period_cnt)
        p->pwm_time_cnt[i] = 0;
}

static enum hrtimer_restart pwm_timer_handler(struct hrtimer *timer)
{
    int i;
    ktime_t period_time;

    for (i = 0; i < FAN_COUNT; i++) {
        handle_pwm(fsd_devp, i);
    }

    period_time = ns_to_ktime(fsd_devp->pwm_timer_period_us * 1000);
    hrtimer_forward_now(timer, period_time);

    return HRTIMER_RESTART;
}

static void pa6_enable_output_high(struct fsd_dev* p)
{
    unsigned int tmp;

    // set PA Config0 register for PA6 to be GPIO output.
    tmp = readl(p->regsaddr+REG_PA_CFG0);
    tmp &= 0xf0ffffff;    // clean config bits for PA6
    tmp |= 0x01000000;    // config PA6 as gpio output;
    writel(tmp, (p->regsaddr+REG_PA_CFG0));

    // set gpio PA6 to high level;
    tmp = readl(p->regsaddr+REG_PA_DATA);
    tmp |= 1<<6;
    writel(tmp, (p->regsaddr+REG_PA_DATA));
}

static void pa6_disable_gpio(struct fsd_dev* p)
{
    unsigned int tmp;

    // set PA Config0 register for PA6 to be disabled.
    tmp = readl(p->regsaddr+REG_PA_CFG0);
    tmp &= 0xf0ffffff;    // clean config bits for PA6
    tmp |= 0x0f000000;    // disable PA6;
    writel(tmp, (p->regsaddr+REG_PA_CFG0));
}

static void pd8_enable_output_high(struct fsd_dev* p)
{
    unsigned int tmp;

    // set PD8 to be GPIO output
    tmp = readl(p->regsaddr+REG_PD_CFG1);
    tmp &= 0xfffffff0;
    tmp |= 0x00000001;
    writel(tmp, (p->regsaddr+REG_PD_CFG1));

    // set PD8 to high level
    tmp = readl(p->regsaddr+REG_PD_CFG1);
    tmp |= 1<<0;
    writel(tmp, (p->regsaddr+REG_PD_CFG1));
}

static void pd8_disable_gpio(struct fsd_dev* p)
{
    unsigned int tmp;

    tmp = readl(p->regsaddr+REG_PD_CFG1);
    tmp &= 0xfffffff0;
    tmp |= 0x0000000f;
    writel(tmp, (p->regsaddr+REG_PD_CFG1));
}

static void set_pl10_as_pwm(struct fsd_dev* p)
{
    unsigned int tmp;

    // set PL10 Config1 register for PL10 to be SPWM0.
    tmp = readl(p->regsaddr_r+REG_PL_CFG1);
    tmp &= 0xfffff0ff;
    tmp |= 0x00000200;
    writel(tmp, (p->regsaddr_r+REG_PL_CFG1));
}

static void set_hw_pwm_duty(struct fsd_dev* p, int pwm_duty)
{
    unsigned int tmp;
    unsigned int i;
    unsigned int cycle_num;
    unsigned int active_cyble_num;

    //* check bit 28 to see whether the pwm0 period register is busy.
    i = 0;
    tmp = readl(p->hw_pwm_regsaddr+REG_PWM_CH_CTRL);
    while((tmp & 0x10000000) != 0 && i<1000)
    {
        i++;
        udelay(10);
        tmp = readl(p->hw_pwm_regsaddr+REG_PWM_CH_CTRL);
    }

    p->hw_pwm_duty = pwm_duty;

    // set PWM0 Period register, entire cycle numbers = 1000(0x03e7+1), and initial the entire active cycle numbers to be 1000(0x3e8).
    cycle_num = (unsigned int)p->hw_pwm_cycles;
    active_cyble_num = (unsigned int)((p->hw_pwm_duty * cycle_num) / 100);

    tmp = readl(p->hw_pwm_regsaddr+REG_PWM_CH0_PERIOD);
    tmp = (cycle_num - 1)<<16 | (active_cyble_num);
    writel(tmp, (p->hw_pwm_regsaddr+REG_PWM_CH0_PERIOD));
}

static void enable_hw_pwm(struct fsd_dev* p)
{
    unsigned int tmp;

    // set PWM Channel Control register to enable PWM0.
    tmp = readl(p->hw_pwm_regsaddr+REG_PWM_CH_CTRL);
    tmp &= 0xffff8000;
    tmp |= 0x0000007f;  //* enable pwm0 and set prescale to 1, so it is 24MH per cycle.
    writel(tmp, (p->hw_pwm_regsaddr+REG_PWM_CH_CTRL));

    set_hw_pwm_duty(p, 100);
}

static void disable_hw_pwm(struct fsd_dev* p)
{
    unsigned int tmp;

    // set PWM Channel Control register to disable PWM0.
    tmp = readl(p->hw_pwm_regsaddr+REG_PWM_CH_CTRL);
    tmp &= 0xffff8000;
    writel(tmp, (p->hw_pwm_regsaddr+REG_PWM_CH_CTRL));
}

static void set_fan_hw_version(struct fsd_dev* p)
{
    bool pg4_in;
    bool pg5_in;

    pg4_in = get_gpio_pg4_input_value(p);
    pg5_in = get_gpio_pg5_input_value(p);

    p->hw_version = ((pg5_in << 1) | (pg4_in)) & 0xff;

    return;
}

void fan_pwm_init(struct fsd_dev* p)
{
    ktime_t ktime;
    int i;

    set_fan_hw_version(p);
    printk("Hw versoin: %d PG: %d %d\n", p->hw_version, get_bit(p->hw_version, 1),
        get_bit(p->hw_version, 0));

    p->use_hw_pwm = 1;
    printk("Use %s PWM for fan\n", p->use_hw_pwm ? "hardware" : "software");
    if(p->use_hw_pwm)
    {
        set_pl10_as_pwm(p);
        p->hw_pwm_duty = 100;
        p->hw_pwm_cycles = HW_PWM_ENTIRE_CYCLE_NUM;
        enable_hw_pwm(p);
    }
    else
    {
        pa6_enable_output_high(p);
        pd8_enable_output_high(p);

        p->pwm_period_cnt = GPIO_PWM_PERIOD_CNT;
        for (i = 0; i < FAN_COUNT; i++) {
            p->pwm_duty[i] = PWM_DUTY_DEFAULT;
            p->pwm_duty_cnt[i] = p->pwm_period_cnt * p->pwm_duty[i] / 100;
            p->pwm_time_cnt[i] = 0;
        }

        p->pwm_timer_period_us = 200;
        printk("Set the period of fan's htimer to %ldus\n", p->pwm_timer_period_us);

        ktime = ktime_set(0, p->pwm_timer_period_us * 1000);
        hrtimer_init( &p->pwm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        p->pwm_timer.function = &pwm_timer_handler;

        hrtimer_start( &p->pwm_timer, ktime, HRTIMER_MODE_REL );
    }
}

void fan_pwm_release(struct fsd_dev* p)
{
    if(p->use_hw_pwm)
    {
        disable_hw_pwm(p);
    }
    else
    {
        hrtimer_cancel(&p->pwm_timer);
        pa6_disable_gpio(p);
        pd8_disable_gpio(p);
    }
}

static int fsd_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int fsd_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static const struct file_operations fsd_fops =
{
    .owner          = THIS_MODULE,
    .open           = fsd_open,
    .release        = fsd_release,
};


ssize_t fan0_show(struct class* class, struct class_attribute* attr, char* buf)
{
    unsigned int speed;
    speed = fsd_devp->fan0_speed;

    sprintf(buf, "%u\n", speed);
    return strlen(buf);
}


ssize_t fan1_show(struct class* class, struct class_attribute* attr, char* buf)
{
    unsigned int speed;
    speed = fsd_devp->fan1_speed;

    sprintf(buf, "%u\n", speed);
    return strlen(buf);
}

ssize_t fan2_show(struct class* class, struct class_attribute* attr, char* buf)
{
    unsigned int speed;
    speed = fsd_devp->fan2_speed;

    sprintf(buf, "%u\n", speed);
    return strlen(buf);
}

ssize_t fan3_show(struct class* class, struct class_attribute* attr, char* buf)
{
    unsigned int speed;
    speed = fsd_devp->fan3_speed;

    sprintf(buf, "%u\n", speed);
    return strlen(buf);
}

ssize_t pwm0_duty_show(struct class* class, struct class_attribute* attr, char* buf)
{
    if(fsd_devp->use_hw_pwm)
        sprintf(buf, "%u\n", fsd_devp->hw_pwm_duty);
    else
        sprintf(buf, "%u\n", fsd_devp->pwm_duty[0]);

    return strlen(buf);
}

ssize_t pwm1_duty_show(struct class* class, struct class_attribute* attr, char* buf)
{
    if(fsd_devp->use_hw_pwm)
        sprintf(buf, "%u\n", fsd_devp->hw_pwm_duty);
    else
        sprintf(buf, "%u\n", fsd_devp->pwm_duty[1]);

    return strlen(buf);
}

ssize_t pwm_duty_show(struct class* class, struct class_attribute* attr, char* buf)
{
    if(fsd_devp->use_hw_pwm)
        sprintf(buf, "%u\n", fsd_devp->hw_pwm_duty);
    else
        sprintf(buf, "%u\n", fsd_devp->pwm_duty[0]);

    return strlen(buf);
}

static int set_pwm_duty(int index, const char* pwm_duty_str, int count)
{
    int pwm_duty;
    int ret;

    ret = strict_strtoul(pwm_duty_str, 10, (long unsigned int*)&pwm_duty);
    if(ret != 0)
        return -EINVAL;

    if (pwm_duty < 0 || pwm_duty > 100) {
        printk("invalid duty %d.\n", pwm_duty);
        return -1;
    }

    if(fsd_devp->use_hw_pwm)
    {
        set_hw_pwm_duty(fsd_devp, pwm_duty);
    }
    else
    {
        fsd_devp->pwm_duty[index] = pwm_duty;
        fsd_devp->pwm_duty_cnt[index] = (fsd_devp->pwm_period_cnt * pwm_duty + 99)
                / 100;
        fsd_devp->pwm_time_cnt[index] = 0;
    }

    return count;
}

ssize_t pwm0_duty_store(struct class* class, struct class_attribute* attr, const char* buf, size_t count)
{
    return set_pwm_duty(0, buf, count);
}

ssize_t pwm1_duty_store(struct class* class, struct class_attribute* attr, const char* buf, size_t count)
{
    return set_pwm_duty(1, buf, count);
}

ssize_t pwm_duty_store(struct class* class, struct class_attribute* attr, const char* buf, size_t count)
{
    set_pwm_duty(0, buf, count);
    set_pwm_duty(1, buf, count);
    return count;
}

ssize_t fan_detect_start_store(struct class* class, struct class_attribute* attr, const char* buf, size_t count)
{
    fan_detect_init(fsd_devp);
    return count;
}

ssize_t fan_detect_stop_store(struct class* class, struct class_attribute* attr, const char* buf, size_t count)
{
    fan_detect_release(fsd_devp);
    return count;
}

static struct class_attribute fsd_clase_attrs[] =
{
    __ATTR(fan0speed, 0664, fan0_show, NULL),
    __ATTR(fan1speed, 0664, fan1_show, NULL),
    __ATTR(fan2speed, 0664, fan2_show, NULL),
    __ATTR(fan3speed, 0664, fan3_show, NULL),
    __ATTR(pwm0_duty, 0664, pwm0_duty_show, pwm0_duty_store),
    __ATTR(pwm1_duty, 0664, pwm1_duty_show, pwm1_duty_store),
    __ATTR(pwm_duty, 0664, pwm_duty_show, pwm_duty_store),
    __ATTR(start, 0664, NULL, fan_detect_start_store),
    __ATTR(stop, 0664, NULL, fan_detect_stop_store),
    __ATTR_NULL,
};

static struct class fsd_class =
{
    .name = "fanspeed",
    .class_attrs = fsd_clase_attrs,
};


static int __init fsd_dev_init(void)
{
    int   ret;
    int   devno;
    dev_t dev = 0;

    fsd_devp = kzalloc(sizeof(struct fsd_dev), GFP_KERNEL);
    if (fsd_devp == NULL)
    {
        pr_err("%s: malloc memory for fan speed detect device error\n", __func__);
        return -ENOMEM;
    }

    fsd_devp->fsd_dev_major = FSD_DEV_MAJOR;
    fsd_devp->fsd_dev_minor = FSD_DEV_MINOR;

    /* register char device */
    dev = MKDEV(fsd_devp->fsd_dev_major, fsd_devp->fsd_dev_minor);
    if (fsd_devp->fsd_dev_major)
    {
        ret = register_chrdev_region(dev, 1, "fanspeed");
    }
    else
    {
        ret = alloc_chrdev_region(&dev, fsd_devp->fsd_dev_minor, 1, "fsd_dev");
        fsd_devp->fsd_dev_major = MAJOR(dev);
        fsd_devp->fsd_dev_minor = MINOR(dev);
    }
    if (ret < 0)
    {
        printk("%s: can't get major %d", __func__, fsd_devp->fsd_dev_major);
        return -EFAULT;
    }


    /* create char device */
    devno = MKDEV(fsd_devp->fsd_dev_major, fsd_devp->fsd_dev_minor);
    cdev_init(&fsd_devp->cdev, &fsd_fops);
    fsd_devp->cdev.owner = THIS_MODULE;
    fsd_devp->cdev.ops   = &fsd_fops;
    ret = cdev_add(&fsd_devp->cdev, devno, 1);
    if (ret)
    {
        printk("%s: add fan speed detect device error\n", __func__);
        ret = -EINVAL;
        goto err0;
    }

    ret = class_register(&fsd_class);
    if(ret<0)
    {
        printk("%s: register fan speed detect class failed\n", __func__);
        ret = -EINVAL;
        goto err1;
    }
    fsd_devp->class = &fsd_class;

    fsd_devp->dev = device_create(fsd_devp->class, NULL, devno, NULL, "fanspeed");
    if (IS_ERR(fsd_devp->dev))
    {
        printk("%s: create fan speed detect_dev device failed\n", __func__);
        ret = -EINVAL;
        goto err2;
    }

    if(fsd_iomem_register(fsd_devp))
    {
        printk("%s: register fsd io mem fail.", __func__);
        goto err3;
    }

    fan_pwm_init(fsd_devp);

    return 0;
err3:
    device_destroy(fsd_devp->class, dev);
err2:
    class_unregister(fsd_devp->class);
err1:
    cdev_del(&fsd_devp->cdev);
err0:
    unregister_chrdev_region(dev, 1);
    if (fsd_devp)
    {
        kfree(fsd_devp);
    }

    return ret;
}
module_init(fsd_dev_init);

static void __exit fsd_dev_exit(void)
{
    dev_t dev;

    if (fsd_devp == NULL)
    {
        pr_err("%s: invalid fsd_devp\n", __func__);
        return;
    }

    fan_detect_release(fsd_devp);
    fan_pwm_release(fsd_devp);

    fsd_iomem_unregister(fsd_devp);

    dev = MKDEV(fsd_devp->fsd_dev_major, fsd_devp->fsd_dev_minor);
    device_destroy(fsd_devp->class, dev);
    class_unregister(fsd_devp->class);
    cdev_del(&fsd_devp->cdev);

    unregister_chrdev_region(dev, 1);

    kfree(fsd_devp);
    fsd_devp = NULL;
}
module_exit(fsd_dev_exit);

#if 0
#if defined(CONFIG_OF)
static struct of_device_id fan_match[] = {
    {.compatible = "allwinner,fan",},
    {}
}
MODULE_DEVICE_TABLE(of, fan_match)
#endif

static struct platform_driver fan_driver {
    .probe = fan_probe,
    .remove = fan_remove,
    .driver = {
        .name = "fanspeed",
        .owner = THIS_MODULE,
#if defined(CONFIG_OF)
        .of_match_table = fan_match,
#endif
    }
};


static int __init fan_dev_init(void)
{
    return paltform_driver_register(&fan_driver);
}

static void __exit fan_dev_exit(void)
{
    platform_driver_unregister(&fan_driver);
}

module_init(fan_dev_init);
module_exit(fan_dev_exit);
#endif

MODULE_AUTHOR("Chen Xiaochuan");
MODULE_DESCRIPTION("Use GPIO External Interrupt To To Detect Fan Speed.");
MODULE_LICENSE("GPL");

