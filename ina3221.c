#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#define INA3221_CONFIG			    0x00
#define INA3221_SHUNT_VOL_CHAN1		0x01
#define INA3221_BUS_VOL_CHAN1		0x02
#define INA3221_SHUNT_VOL_CHAN2		0x03
#define INA3221_BUS_VOL_CHAN2		0x04
#define INA3221_SHUNT_VOL_CHAN3		0x05
#define INA3221_BUS_VOL_CHAN3		0x06
#define INA3221_CRIT_CHAN1		    0x07
#define INA3221_WARN_CHAN1		    0x08
#define INA3221_CRIT_CHAN2		    0x09
#define INA3221_WARN_CHAN2		    0x0A
#define INA3221_CRIT_CHAN3		    0x0B
#define INA3221_WARN_CHAN3		    0x0C
#define INA3221_MASK_ENABLE		    0x0F

struct ina3221_chan_pdata {
	const char *chan_name;
	u16 warn_conf_limits;
	u16 crit_conf_limits;
	u16 shunt_resistor;
	u16 mask;
};

struct ina3221_platform_data {
	struct ina3221_chan_pdata cpdata[INA3221_NUMBER_OF_RAILS];
};


struct ina3221_chip {
	struct device *dev;
	struct i2c_client *client;
	struct ina3221_platform_data *pdata;
	struct mutex mutex;
	// int shutdown_complete;
	// int is_suspended;
	// int mode;
	// int alert_enabled;
	// struct notifier_block nb_hot;
	// struct notifier_block nb_cpufreq;
};


static int ina3221_read_value(struct ina3221_chip *chip, int channel, u8 dev_reg, int *val){
    int ret, reading;
    struct i2c_client *client = chip->client;
    u8 reg;
    reg = dev_reg + (channel * 2);
    mutex_lock(&chip->mutex);
    // read
    reading = i2c_smbus_read_word_data(client, reg);
    // error handling
	if (ret < 0) {
		dev_err(chip->dev, "Value read on channel %d failed: %d\n", channel, ret);
		goto exit;
	}
    val = be16_to_cpu(reading);

exit:
	mutex_unlock(&chip->mutex);
	return ret;
}


static int ina3221_read_raw(struct iio_dev *indio_dev, \
struct iio_chan_spec const *chan, int *val, int *val2, long mask){
    struct ina3221_chip *chip = iio_priv(indio_dev);
    struct device *dev = chip->dev;
    int type = chan->type;
    int channel = chan->channel;
    int address = chan->address;
    int ret = 0;

    if (channel >= 3) {
        dev_err(dev, "Invalid channel Id %d\n", channel);
        return -EINVAL;
    }
    if (mask != IIO_CHAN_INFO_PROCESSED) {
        dev_err(dev, "Invalid mask 0x%08lx\n", mask);
        return -EINVAL;
    }

    switch(type){
        case IIO_VOLTAGE:
            switch(address){
                case 0:
                    //read shunt volt
                    ina3221_read_value(chip, channel, INA3221_SHUNT_VOL_CHAN1, val);
                    break;
                case 1:
                    //read bus volt
                    ina3221_read_value(chip, channel, INA3221_BUS_VOL_CHAN1, val);
                    break;
            }
            break;
        case IIO_CURRENT:
            switch(address){
                case 0:
                    //read curr crit
                    ina3221_read_value(chip, channel, INA3221_CRIT_CHAN1, val);
                    break;
                case 1:
                    //read warn crit
                    ina3221_read_value(chip, channel, INA3221_WARN_CHAN1, val);
                    break;
            }
            break;
        default:
            ret = -EINVAL;
            break;
        }
        if(!ret){
            ret = IIO_VAL_INT;
        }
        return ret;
}

#define channel_type(_type, _add, _channel, _name)			\
{								\
	.type = _type,						\
	.indexed = 1,						\
	.address = _add,					\
	.channel = _channel,					\
	.extend_name = _name,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
		//.event_spec = ina3221_alarm,\
		.num_event_specs = ARRAY_SIZE(ina3221_alarm),
}

#define channel_spec(chan)						\
    channel_type(IIO_VOLTAGE, 0, chan, "shunt voltage"),			\
  	channel_type(IIO_VOLTAGE, 1, chan, "bus voltage"),			\
  	channel_type(IIO_CURRENT, 0, chan, "crit limit"),     \
    channel_type(IIO_CURRENT, 1, chan, "warn limit"),   \
    channel_type(IIO_RESISTANCE, 0, chan, "shunt resistor" )


static const struct iio_chan_spec ina3221_channels_spec[] = {
  	channel_spec(0),
  	channel_spec(1),
  	channel_spec(2),
};

static const struct iio_info ina3221_info = {
  //	.attrs = &ina3221_groups,
  	.driver_module = THIS_MODULE,
  	.read_raw = &ina3221_read_raw,
};

static struct ina3221_platform_data *ina3221_get_platform_data_dt(
  	struct i2c_client *client)
{
  	struct ina3221_platform_data *pdata;
  	struct device *dev = &client->dev;
  	struct device_node *np = dev->of_node;
  	struct device_node *child;
  	u32 reg;   //change datatype
  	int ret;
  	u32 pval;   // change datatype    change u32
  	int valid_channel = 0;

  	if (!np) {
  		dev_err(&client->dev, "Only DT supported\n");
  		return ERR_PTR(-ENODEV);
  	}

  	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
  	if (!pdata) {
  		dev_err(&client->dev, "pdata allocation failed\n");
  		return ERR_PTR(-ENOMEM);
  	}


  	for_each_child_of_node(np, child) {
  		ret = of_property_read_u32(child, "reg", &reg);
  		if (ret || reg >= 3) {
  			dev_err(dev, "reg property invalid on node %s\n",
  				child->name);
  			continue;
  		}

  		pdata->cpdata[reg].chan_name =  of_get_property(child,
  						"ti,chan-name", NULL);
  		if (!pdata->cpdata[reg].chan_name) {
  			dev_err(dev, "channel name is not provided on node %s\n",
  				child->full_name);
  			continue;
  		}

  		ret = of_property_read_u32(child, "ti,current-warning-limit-ma",
  				&pval);
  		if (!ret)
  			pdata->cpdata[reg].warn_conf_limits = pval;
  		else
  			pdata->cpdata[reg].warn_conf_limits = U32_MINUS_1;

  		ret = of_property_read_u32(child,
  				"ti,current-critical-limit-ma", &pval);
  		if (!ret)
  			pdata->cpdata[reg].crit_conf_limits = pval;
  		else
  			pdata->cpdata[reg].crit_conf_limits = U32_MINUS_1;

  		ret = of_property_read_u32(child, "ti,shunt-resistor-mohm",
  				&pval);
  		if (!ret)
  			pdata->cpdata[reg].shunt_resistor = pval;

  		valid_channel++;
  	}

  	if (!valid_channel)
  		return ERR_PTR(-EINVAL);

  	return pdata;
  }


static const struct i2c_device_id ina3221_id[] = {
  	{.name = "ina3221x",},
  	{},
};
MODULE_DEVICE_TABLE(i2c, ina3221_id);

static struct i2c_driver ina3221_driver = {
  	.driver = {
  		.name	= "ina3221x",
  		.owner = THIS_MODULE,
  		.pm = &ina3221_pm_ops,
  	},
  	.probe		= ina3221_probe,
  	.remove		= ina3221_remove,
  	.shutdown	= ina3221_shutdown,
  	.id_table	= ina3221_id,
};

module_i2c_driver(ina3221_driver);


MODULE_DESCRIPTION("TI INA3221 3-Channel Shunt and Bus Voltage Monitor");
MODULE_AUTHOR("Deepak Nibade <dnibade@nvidia.com>");
MODULE_AUTHOR("Timo Alho <talho@nvidia.com>");
MODULE_AUTHOR("Anshul Jain <anshulj@nvidia.com>");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
