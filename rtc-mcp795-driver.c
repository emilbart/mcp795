#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/rtc.h>
#include <linux/of.h>
#include <linux/bcd.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of_device.h>

/* Read data from EE memory array beginning at selected address */
#define MCP795_INSTR_EEREAD     0x03
/* Write data to EE memory array beginning at selected address */
#define MCP795_INSTR_EEWRITE    0x02
/* Reset the write enable latch (disable write operations) */
#define MCP795_INSTR_EEWRDI     0x04
/* Set the write enable latch (enable write operations) */
#define MCP795_INSTR_EEWREN     0x06
/* Read STATUS register */
#define MCP795_INSTR_SRREAD     0x05
/* Write STATUS register */
#define MCP795_INSTR_SRWRITE    0x01
/* Read RTCC/SRAM array beginning at selected address */
#define MCP795_INSTR_READ       0x13
/* Write RTCC/SRAM data to memory array beginning at selected address */
#define MCP795_INSTR_WRITE      0x12
/* Unlock ID Locations */
#define MCP795_INSTR_UNLOCK     0x14
/* Write to the ID Locations */
#define MCP795_INSTR_IDWRITE    0x32
/* Read the ID Locations */
#define MCP795_INSTR_IDREAD     0x33
/* Clear RAM Location to 0 */
#define MCP795_INSTR_CLRRAM     0x54

/* Hundredths of Seconds */
#define MCP795_REG_TENTH_SEC   0x00
/* Seconds */
#define MCP795_REG_SECONDS     0x01
/* Minutes */
#define MCP795_REG_MINUTES     0x02
/* Hour */
#define MCP795_REG_HOUR        0x03
/* Day */
#define MCP795_REG_DAY         0x04
/* Date */
#define MCP795_REG_DATE        0x05
/* Month */
#define MCP795_REG_MONTH       0x06
/* Year */
#define MCP795_REG_YEAR        0x07
/* Control register */
#define MCP795_REG_CONTROL     0x08
/* Calibration register*/
#define MCP795_REG_CALIBRATION 0x09
#define MCP795_DATE_REG_LEN 8

#define MCP795_BIT_AM       0x20
#define MCP795_BIT_12H_MODE 0x40
#define MCP795_BIT_CALSGN   0x80

#define MCP795_BIT_ST       0x80

#define MCP795_BIT_DAY      0x07
#define MCP795_BIT_VBATEN   0x08
#define MCP795_BIT_VBAT     0x10
#define MCP795_BIT_OSCON    0x20

#define MCP795_BIT_LP       0x20

#define MCP795_BIT_OUT      0x80
#define MCP795_BIT_SQWE     0x40
#define MCP795_BIT_ALM1     0x20
#define MCP795_BIT_ALM0     0x10
#define MCP795_BIT_EXTOSC   0x08
#define MCP795_BIT_RS       0x07

/* Write-In-Process bit mask */
#define MCP795_BIT_WIP      0x01

/* page size in bytes of NVRAM */
#define MCP795_NVRAM_PAGE_SIZE 8

struct mcp795 {
	struct spi_device *spi;
	struct rtc_device *rtc;
	unsigned int nvram_size;
};

static unsigned bcd2hour(u8 val)
{
    if (val & MCP795_BIT_12H_MODE) {
        unsigned hour = 0;

        hour = bcd2bin(val & 0x0F);
        if (!(val & MCP795_BIT_AM)) {
            hour += 12;
        }
        return hour;
    }
    return bcd2bin(val & 0x3F);
}

static u8 hour2bcd(bool hr12, int hour, u8 prevVal)
{
    return (prevVal & 0xE0) | bin2bcd(hour);
}

static int
mcp795_rtc_read(struct device *dev, u8 cmd, u8 addr, u8 *buf, u16 count)
{
    struct spi_device *spi = to_spi_device(dev);
    u8 txdata[2];

    txdata[0] = cmd;
    txdata[1] = addr;
    return spi_write_then_read(spi, txdata, sizeof(txdata), buf, count);
}

static int
mcp795_rtc_write(struct device *dev, u8 cmd, u8 addr, u8 *data, u16 count)
{
    struct spi_device *spi = to_spi_device(dev);
    u8 txdata[2 + count];

    txdata[0] = cmd;
    txdata[1] = addr;
    memcpy(&txdata[2], data, count);
    return spi_write(spi, txdata, sizeof(txdata));
}

static int mcp795_rtcc_set_bits(struct device *dev, u8 addr, u8 mask, u8 state)
{
    int ret;
    u8 value;

    ret = mcp795_rtc_read(dev, MCP795_INSTR_READ, addr, &value, sizeof(value));
    if (ret) {
        return ret;
    }
    if ((value & mask) != state)
    {
        value &= (~mask);
        value |= state;;
        ret = mcp795_rtc_write(dev, MCP795_INSTR_WRITE, addr,
                &value, sizeof(value));
    }
    return ret;
}

static int mcp795_nvram_write_enable(struct device *dev)
{
    struct spi_device *spi = to_spi_device(dev);
    u8 txdata;

    txdata = MCP795_INSTR_EEWREN;
    return spi_write(spi, &txdata, sizeof(txdata));
}

static int mcp795_uid_unlock(struct device *dev)
{
    struct spi_device *spi = to_spi_device(dev);
    u8 txdata[2] = {MCP795_INSTR_UNLOCK, 0x55};
    int ret;

    ret = spi_write(spi, &txdata, sizeof(txdata));
    if (ret < 0) {
        return ret;
    }
    txdata[1] = 0xAA;
    return spi_write(spi, &txdata, sizeof(txdata));
}

static int mcp795_nvram_read_status(struct device *dev, u8 *status)
{
    struct spi_device *spi = to_spi_device(dev);
    u8 txdata;

    txdata = MCP795_INSTR_SRREAD;
    return spi_write_then_read(spi, &txdata, sizeof(txdata), status, 1);
}

static int mcp795_initialize(struct mcp795 *mcp795)
{
    struct device *dev = &mcp795->spi->dev;
    u8  buf[10];
    int ret;

    ret = mcp795_rtcc_set_bits(dev, MCP795_REG_SECONDS,
                               MCP795_BIT_ST, MCP795_BIT_ST);
    if (ret)
        return ret;
    ret = mcp795_rtcc_set_bits(dev, MCP795_REG_HOUR, MCP795_BIT_12H_MODE, 0);
    if (ret)
        return ret;
    ret = mcp795_rtcc_set_bits(dev, MCP795_REG_DAY,
            MCP795_BIT_VBAT | MCP795_BIT_VBATEN, MCP795_BIT_VBATEN);
    if (ret)
        return ret;
    ret = mcp795_rtcc_set_bits(dev, MCP795_REG_CONTROL,
            MCP795_BIT_SQWE | MCP795_BIT_ALM1 | MCP795_BIT_ALM0, 0);
    if (ret)
        return ret;

    ret = mcp795_rtc_read(dev, MCP795_INSTR_READ, MCP795_REG_TENTH_SEC,
            buf, sizeof(buf));
    if (ret < 0) {
        dev_err(dev, "mcp795_initialize: failed to read data from device\n");
        return ret;
    }
    dev_info(dev, "mcp795_initialize:\n"
            "ST=%d\n"
            "CALSGN=%d\n"
            "12h=%d\n"
            "AM=%d\n"
            "OSCON=%d\n"
            "VBAT=%d\n"
            "VBATEN=%d\n"
            "LP=%d\n"
            "OUT=%d\n"
            "SQWE=%d\n"
            "ALM1=%d\n"
            "ALM0=%d\n"
            "EXTOSC=%d\n"
            "RS=0x%x\n"
            "calibration=0x%x\n",
            (buf[1] & MCP795_BIT_ST) != 0,
            (buf[3] & MCP795_BIT_CALSGN) != 0,
            (buf[3] & MCP795_BIT_12H_MODE) != 0,
            (buf[3] & MCP795_BIT_AM) != 0,
            (buf[4] & MCP795_BIT_OSCON) != 0,
            (buf[4] & MCP795_BIT_VBAT) != 0,
            (buf[4] & MCP795_BIT_VBATEN) != 0,
            (buf[6] & MCP795_BIT_LP) != 0,
            (buf[8] & MCP795_BIT_OUT) != 0,
            (buf[8] & MCP795_BIT_SQWE) != 0,
            (buf[8] & MCP795_BIT_ALM1) != 0,
            (buf[8] & MCP795_BIT_ALM0) != 0,
            (buf[8] & MCP795_BIT_EXTOSC) != 0,
            buf[8] & MCP795_BIT_RS,
            buf[9]
            );
    return 0;
}

static int mcp795_get_time(struct device *dev, struct rtc_time *time)
{
    u8  buf[MCP795_DATE_REG_LEN];
    int ret;

    /* Use write-then-read to get all the date/time registers
     * since dma from stack is nonportable
     */
    ret = mcp795_rtc_read(dev, MCP795_INSTR_READ, MCP795_REG_TENTH_SEC,
            buf, sizeof(buf));
    if (ret < 0) {
        dev_err(dev, "mcp795_get_time: failed to read data from device\n");
        return ret;
    }
    time->tm_sec    = bcd2bin(buf[MCP795_REG_SECONDS] & (~MCP795_BIT_ST));
    time->tm_min    = bcd2bin(buf[MCP795_REG_MINUTES]);
    time->tm_hour   = bcd2hour(buf[MCP795_REG_HOUR]);
    time->tm_wday   = buf[MCP795_REG_DAY] & MCP795_BIT_DAY;
    time->tm_mday   = bcd2bin(buf[MCP795_REG_DATE]);
    time->tm_mon    = bcd2bin(buf[MCP795_REG_MONTH]) & (~MCP795_BIT_LP);
    time->tm_year   = bcd2bin(buf[MCP795_REG_YEAR]) + 100;

    dev_info(dev, "Read from mcp795: %04d-%02d-%02d %02d:%02d:%02d wday=%d\n",
            time->tm_year + 1900, time->tm_mon, time->tm_mday,
            time->tm_hour, time->tm_min, time->tm_sec, time->tm_wday);

    return rtc_valid_tm(time);
}

static int mcp795_set_time(struct device *dev, struct rtc_time *time)
{
    u8 data[MCP795_DATE_REG_LEN];
    int ret;

    dev_info(dev, "Set to mcp795: %04d-%02d-%02d %02d:%02d:%02d wday=%d\n",
            time->tm_year + 1900, time->tm_mon, time->tm_mday,
            time->tm_hour, time->tm_min, time->tm_sec, time->tm_wday);

    ret = mcp795_rtc_read(dev, MCP795_INSTR_READ, MCP795_REG_TENTH_SEC,
            data, sizeof(data));
    if (ret) {
        return ret;
    }
    data[MCP795_REG_TENTH_SEC] = 0;
    data[MCP795_REG_SECONDS] =
            (data[MCP795_REG_SECONDS] & MCP795_BIT_ST) | bin2bcd(time->tm_sec);
    data[MCP795_REG_MINUTES] = bin2bcd(time->tm_min);
    data[MCP795_REG_HOUR] =
            hour2bcd(false, time->tm_hour, data[MCP795_REG_HOUR]);
    data[MCP795_REG_DAY] =
            (data[MCP795_REG_DAY] & ~MCP795_BIT_DAY) | time->tm_wday;
    data[MCP795_REG_DATE] = bin2bcd(time->tm_mday);
    data[MCP795_REG_MONTH] =
            (data[MCP795_REG_MONTH] & MCP795_BIT_LP) | bin2bcd(time->tm_mon);
    data[MCP795_REG_YEAR] = bin2bcd(time->tm_year - 100);

    return mcp795_rtc_write(dev, MCP795_INSTR_WRITE, MCP795_REG_TENTH_SEC,
            data, sizeof(data));
}

int mcp795_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
    dev_info(dev, "%s\n", __FUNCTION__);

    alarm->time.tm_sec = 50;
    alarm->time.tm_min = 50;
    alarm->time.tm_hour = 4;
    alarm->time.tm_mday = -1;
    alarm->time.tm_mon = -1;
    alarm->time.tm_year = -1;
    /* next three fields are unused by Linux */
    alarm->time.tm_wday = -1;
    alarm->time.tm_mday = -1;
    alarm->time.tm_isdst = -1;

    return 0;
}

int mcp795_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    return 0;
}

int mcp795_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    return 0;
}

static const struct rtc_class_ops mcp795_rtc_ops = {
    .read_time          = mcp795_get_time,
    .set_time           = mcp795_set_time,
    .read_alarm         = mcp795_read_alarm,
    .set_alarm          = mcp795_set_alarm,
    .alarm_irq_enable   = mcp795_alarm_irq_enable
};

static irqreturn_t mcp795_irq(int irq, void *p)
{
    disable_irq(irq);
    return IRQ_HANDLED;
}

static bool mcp795_nvram_is_wip(struct device *dev)
{
    unsigned long timeout;
    unsigned long retries = 0;
    bool busy = true;

    timeout = jiffies + msecs_to_jiffies(25);
    do {
        int ret;
        u8 status;

        ret = mcp795_nvram_read_status(dev, &status);
        if (ret < 0) {
            dev_err(dev, "failed to read status register, ret=%d\n", ret);
            continue;
        }
        if (!(status & MCP795_BIT_WIP)) {
            busy = false;
            break;
        }
        msleep(1);
    } while ((retries++ < 3) || time_before_eq(jiffies, timeout));
    return busy;
}

static ssize_t mcp795_nvram_read(struct file *file, struct kobject *kobj,
        struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct mcp795 *mcp795 = dev_get_drvdata(dev);
    int ret;

    if (unlikely(off >= mcp795->nvram_size)) {
        return 0;
    }
    if (count >= mcp795->nvram_size) {
        count = mcp795->nvram_size;
    }
    if ((off + count) > mcp795->nvram_size) {
        count = mcp795->nvram_size - off;
    }
    if (unlikely(!count)) {
        return count;
    }
    if (mcp795_nvram_is_wip(dev)) {
        return -ETIMEDOUT;
    }
    ret = mcp795_rtc_read(dev, MCP795_INSTR_EEREAD, off, buf, count);
    if (ret < 0) {
        dev_err(dev, "nvram read error %d\n", ret);
    }
    return (ret < 0) ? ret : count;
}

static ssize_t mcp795_nvram_write(struct file *file, struct kobject *kobj,
        struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct mcp795 *mcp795 = dev_get_drvdata(dev);
    size_t written = 0;
    size_t page_count;
    int ret;

    if (unlikely(off >= mcp795->nvram_size)) {
        return -EFBIG;
    }
    if (count >= mcp795->nvram_size) {
        count = mcp795->nvram_size;
    }
    if ((off + count) > mcp795->nvram_size) {
        count = mcp795->nvram_size - off;
    }
    if (unlikely(!count)) {
        return count;
    }
    page_count = count;
    if (off > 0) {
        if (count > MCP795_NVRAM_PAGE_SIZE) {
            page_count = MCP795_NVRAM_PAGE_SIZE - (off % 8);
        }
    }
    do {
        if (page_count > MCP795_NVRAM_PAGE_SIZE) {
            page_count = MCP795_NVRAM_PAGE_SIZE;
        }
        if (mcp795_nvram_is_wip(dev)) {
            ret = ETIMEDOUT;
            break;
        }
        ret = mcp795_nvram_write_enable(dev);
        if (ret < 0) {
            dev_err(dev, "nvram write enable command failed %d\n", ret);
            break;
        }
        ret = mcp795_rtc_write(dev, MCP795_INSTR_EEWRITE, off, &buf[written], page_count);
        if (ret < 0) {
            dev_err(dev, "nvram write error %d\n", ret);
            break;
        }
        written += page_count;
        count -= page_count;
        off += page_count;
        page_count = count;
    } while (count > 0);
    return (written > 0) ? written : ret;
}

static struct bin_attribute mcp795_nvram_attr = {
    .attr.name  = "mcp795_nvram_attr",
    .attr.mode  = S_IRUGO | S_IWUSR,
    .read       = mcp795_nvram_read,
    .write      = mcp795_nvram_write,
};

static ssize_t mcp795_uid_read(struct file *file, struct kobject *kobj,
        struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    int ret;
    int i;

    if (count < 8) {
        return 0;
    }
    dev_warn(dev, "unique ID read\n");
    ret = mcp795_rtc_read(dev, MCP795_INSTR_IDREAD, 0, buf, 8);
    if (ret < 0) {
        dev_err(dev, "unique ID read error %d\n", ret);
    }
    for (i = 0; i < 8; i++)
        dev_err(dev, "[%d]=%02X\n", i, buf[i]);

    return (ret < 0) ? ret : count;
}

static ssize_t mcp795_uid_write(struct file *file, struct kobject *kobj,
        struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    u64 val;
    int ret;

    ret = kstrtoull((const char *)buf, 0, &val);
    if (ret)
        return ret;
    dev_warn(dev, "unique ID write val=0x%016llX\n", val);

    ret = mcp795_nvram_write_enable(dev);
    if (ret < 0) {
        dev_err(dev, "unique ID write enable command failed %d\n", ret);
    }
    ret = mcp795_uid_unlock(dev);
    if (ret < 0) {
        dev_err(dev, "nvram write unlock failed %d\n", ret);
    }
    ret = mcp795_rtc_write(dev, MCP795_INSTR_IDWRITE, 0, (u8 *)&val, sizeof(val));
    if (ret < 0) {
        dev_err(dev, "unique ID write error %d\n", ret);
    }
    return count;
}

static struct bin_attribute mcp795_uid_attr = {
    .attr.name  = "mcp795_uid_attr",
    .attr.mode  = S_IRUGO | S_IWUSR,
    .read       = mcp795_uid_read,
    .write      = mcp795_uid_write,
    .size       = 16,
};

static const struct of_device_id mcp795_of_match[] = {
    { .compatible = "maxim,mcp795",     .data = (void *)128 },
    { .compatible = "maxim,mcp7952",    .data = (void *)256 },
    { }
};

MODULE_DEVICE_TABLE(of, mcp795_of_match);

static int mcp795_probe(struct spi_device *spi)
{
    const struct of_device_id *of_dev_id;
    struct platform_device *pdev;
    struct mcp795 *mcp795;
    int ret;

    dev_info(&spi->dev, "mcp795_probe spi->irq=%d\n", spi->irq);

    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 3000000;
    ret = spi_setup(spi);
    if (ret) {
        dev_err(&spi->dev, "mcp795_probe: failed to setup SPI device\n");
        return ret;
    }
    mcp795 = devm_kzalloc(&spi->dev, sizeof(struct mcp795), GFP_KERNEL);
    if (!mcp795) {
        dev_err(&spi->dev, "mcp795_probe: devm_kzalloc failed\n");
        return -ENOMEM;
    }
    of_dev_id = of_match_device(mcp795_of_match, &spi->dev);
    if (!of_dev_id)
        return -ENODEV;
    mcp795->nvram_size = (unsigned int)of_dev_id->data;

    mcp795->spi = spi;
    spi_set_drvdata(spi, mcp795);
    ret = mcp795_initialize(mcp795);
    if (ret) {
        dev_err(&spi->dev, "mcp795_probe: failed to initialize RTC\n");
        return ret;
    }
    device_set_wakeup_capable(&spi->dev, true);
    mcp795->rtc = devm_rtc_device_register(&spi->dev, "rtc-mcp795-driver",
            &mcp795_rtc_ops, THIS_MODULE);
    if (IS_ERR(mcp795->rtc)) {
        dev_err(&spi->dev, "mcp795_probe: failed to register RTC device\n");
        return PTR_ERR(mcp795->rtc);
    }
    // spi->irq = gpio_to_irq(25);
    pdev = to_platform_device(&spi->dev);
    spi->irq = platform_get_irq(pdev, 0);
    dev_info(&spi->dev, "mcp795_probe spi->irq=%d\n", spi->irq);
    if (spi->irq) {
        ret = devm_request_irq(&spi->dev, spi->irq, mcp795_irq,
                                0, dev_name(&mcp795->rtc->dev), mcp795);
        if (ret) {
            dev_err(&spi->dev, "mcp795_probe: failed to request IRG %d\n",
                    spi->irq);
        }
    }
    mcp795_nvram_attr.size = mcp795->nvram_size;
    ret = sysfs_create_bin_file(&spi->dev.kobj, &mcp795_nvram_attr);
    if (ret) {
        dev_err(&spi->dev, "mcp795_probe: unable to create sysfs file: %s\n",
                mcp795_nvram_attr.attr.name);
    }
    ret = sysfs_create_bin_file(&spi->dev.kobj, &mcp795_uid_attr);
    if (ret) {
        dev_err(&spi->dev, "mcp795_probe: unable to create sysfs file: %s\n",
                mcp795_uid_attr.attr.name);
    }
    return 0;
}

static int mcp795_remove(struct spi_device *spi)
{
    dev_info(&spi->dev, "mcp_795 remove\n");
    sysfs_remove_bin_file(&spi->dev.kobj, &mcp795_nvram_attr);
    sysfs_remove_bin_file(&spi->dev.kobj, &mcp795_uid_attr);
    return 0;
}

static struct spi_driver mcp795_driver = {
    .probe  = mcp795_probe,
    .remove = mcp795_remove,
    .driver = {
        .name = "mcp795",
        .owner = THIS_MODULE,
        .of_match_table = mcp795_of_match,
    },
};

module_spi_driver(mcp795_driver);

MODULE_DESCRIPTION("mcp795 rtc driver");
MODULE_AUTHOR("Emil Bartczak");
MODULE_LICENSE("GPL");
