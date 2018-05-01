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
#define INA3221_RESET			0x8000
#define INA3221_NUMBER_OF_CHANNEL    3
#define U32_MINUS_1	((u32) -1)


struct ina3221_chan_pdata {
	const char *chan_name;
	u32 warn_conf_limits;
	u32 crit_conf_limits;
	u32 shunt_resistor;
	u16 mask;
};

struct ina3221_platform_data {
	struct ina3221_chan_pdata cpdata[INA3221_NUMBER_OF_CHANNEL];
};


struct ina3221_chip {
	struct device *dev;
	struct i2c_client *client;
	struct ina3221_platform_data *pdata;
	struct mutex mutex;

};


static int to_shunt_volt(int val){
	int volt_uv;
	volt_uv = sign_extend32(val >> 3, 12) * 40;
	return volt_uv;
}

static int to_bus_volt(int val){
	int volt_mv;
	volt_mv = sign_extend32(val >> 3, 12) * 8;
	return volt_mv;
}

static int show_shunt_resistor(struct ina3221_chip *chip, int channel){
	int resistor_mo;
	resistor_mo = chip->pdata->cpdata[channel].shunt_resistor;
	return resistor_mo;
}


static int shunt_volt_to_current(int volt_uv, int resistor_mo){
	int current_ma;

	current_ma = DIV_ROUND_CLOSEST(volt_uv, resistor_mo * 1000);

	return current_ma;
}

static int show_crit_limit(struct ina3221_chip *chip, int channel){
	int current_ma;

 	current_ma= chip->pdata->cpdata[channel].crit_conf_limits;
	return current_ma;
}

static int show_warn_limit(struct ina3221_chip *chip, int channel){
	int current_ma;

 	current_ma= chip->pdata->cpdata[channel].warn_conf_limits;
	return current_ma;
}



static int ina3221_read_value(struct ina3221_chip *chip, int channel, u16 dev_reg){
	int ret;
    struct i2c_client *client = chip->client;
    u16 reg;
    reg = dev_reg + (channel * 2);
	printk("read from: %u\n",reg);

    mutex_lock(&chip->mutex);
    ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(chip->dev, "Value read on channel %d failed: %d\n", channel, ret);
		goto exit;
	}
	printk("read value: %d\n", ret);

exit:
	mutex_unlock(&chip->mutex);
	return ret;
}


static int ina3221_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask){
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
    switch(type){
        case IIO_VOLTAGE:
            switch(address){
                case 0:
                    //read shunt volt
                    ret = ina3221_read_value(chip, channel, INA3221_SHUNT_VOL_CHAN1);
					*val = to_shunt_volt(ret);
                    break;
                case 1:
                    //read bus volt
                    ret = ina3221_read_value(chip, channel, INA3221_BUS_VOL_CHAN1);
					*val = to_bus_volt(ret);
                    break;
            }
            break;
        case IIO_CURRENT:
        	switch(address){
                case 0:
					ret = ina3221_read_value(chip, channel, INA3221_SHUNT_VOL_CHAN1);
					ret = to_shunt_volt(ret);
					*val = shunt_volt_to_current(ret, show_shunt_resistor(chip, channel));
                    break;
                case 1:
					ret = show_shunt_resistor(chip, channel);
					*val = ret;
                    break;
            }
            break;
		case IIO_POWER:
		 	switch(address){
				case 0:
					ret = show_crit_limit(chip, channel);
					*val = ret;
					printk("limit: %d\n", *val);
					break;
				case 1:
					ret = show_warn_limit(chip, channel);
					*val = ret;
					printk("limit: %d\n", *val);
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

static int to_volt_limit(u32 conf_limits, u32 shunt_resistor){
	int volt_limit_uv;
	volt_limit_uv = conf_limits * shunt_resistor * 1000 * 1000;
	volt_limit_uv = DIV_ROUND_CLOSEST(volt_limit_uv, 5) & 0xfff8;
	return volt_limit_uv;

}


// crit warn
static int ina3221_set_alert_reg(struct ina3221_chip *chip, int channel, u16 dev_reg, int volt_limit_uv){
	int ret;
	struct i2c_client *client = chip->client;

	u16 reg;
	reg = dev_reg + (channel * 2);
	mutex_lock(&chip->mutex);
	ret = i2c_smbus_write_word_data(client, reg, cpu_to_be16(volt_limit_uv));
	if (ret < 0) {
		dev_err(chip->dev, "Unable to set limit on channel %d failed: %d\n", channel, ret);
	}
	mutex_unlock(&chip->mutex);
	return ret;
}





static int ina3221_set_limits(struct ina3221_chip *chip)
{
	struct ina3221_chan_pdata *cpdata;
	int i, volt_limit_uv;
	int ret = 0;

	for (i = 0; i < INA3221_NUMBER_OF_CHANNEL; i++) {
		cpdata = &chip->pdata->cpdata[i];

		if (cpdata->crit_conf_limits != U32_MINUS_1) {
			volt_limit_uv = to_volt_limit(cpdata->crit_conf_limits, cpdata->shunt_resistor);
			ret = ina3221_set_alert_reg(chip, i, INA3221_CRIT_CHAN1, volt_limit_uv);
			if (ret < 0)
				break;
		}

		if (cpdata->warn_conf_limits != U32_MINUS_1) {
			volt_limit_uv = to_volt_limit(cpdata->warn_conf_limits, cpdata->shunt_resistor);
			ret = ina3221_set_alert_reg(chip, i, INA3221_CRIT_CHAN1, volt_limit_uv);
			if (ret < 0)
				break;
		}
	}
	return ret;
}




static int ina3221_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask){
    struct ina3221_chip *chip = iio_priv(indio_dev);
    struct device *dev = chip->dev;

    int type = chan->type;
    int channel = chan->channel;
    int address = chan->address;
	struct ina3221_chan_pdata *cpdata = &chip->pdata->cpdata[channel];
	int volt_limit_uv;
    int ret = 0;

    if (channel >= 3) {
        dev_err(dev, "Invalid channel Id %d\n", channel);
        return -EINVAL;
    }

	switch(type){
        case IIO_POWER:
            switch(address){
                case 0:
					volt_limit_uv = to_volt_limit(cpdata->crit_conf_limits, cpdata->shunt_resistor);
					ret = ina3221_set_alert_reg(chip, channel, INA3221_CRIT_CHAN1, volt_limit_uv);
                    break;
                case 1:
					volt_limit_uv = to_volt_limit(cpdata->warn_conf_limits, cpdata->shunt_resistor);
					ret = ina3221_set_alert_reg(chip, channel, INA3221_CRIT_CHAN1, volt_limit_uv);
                    break;
    return -EINVAL;
}

#define channel_type(_type, _add, _channel, _name, _mask){ \
    .type = _type, \
    .indexed = 1, \
    .address = _add, \
    .channel = _channel, \
    .extend_name = _name, \
    .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
}

#define channel_spec(chan)						\
    channel_type(IIO_VOLTAGE, 0, chan, "shunt"),			\
  	channel_type(IIO_VOLTAGE, 1, chan, "bus"),			\
	channel_type(IIO_CURRENT, 0, chan, "current"), \
	channel_type(IIO_CURRENT, 1, chan, "shunt_resistor"),  \
  	channel_type(IIO_POWER, 0, chan, "crit_limit"),     \
    channel_type(IIO_POWER, 1, chan, "warn_limit")


static const struct iio_chan_spec ina3221_channels_spec[] = {
  	channel_spec(0),
  	channel_spec(1),
  	channel_spec(2),
};

static const struct iio_info ina3221_info = {
  	.driver_module = THIS_MODULE,
  	.read_raw = &ina3221_read_raw,
	.write_raw = &ina3221_write_raw,

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
  		if (!ret){
  			pdata->cpdata[reg].warn_conf_limits = pval;
		}
  		else{
  			pdata->cpdata[reg].warn_conf_limits = U32_MINUS_1;
		}
  		ret = of_property_read_u32(child,
  				"ti,current-critical-limit-ma", &pval);
  		if (!ret){
  			pdata->cpdata[reg].crit_conf_limits = pval;
		}
		else{
  			pdata->cpdata[reg].crit_conf_limits = U32_MINUS_1;
		}
  		ret = of_property_read_u32(child, "ti,shunt-resistor-mohm",
  				&pval);
  		if (!ret){
			pdata->cpdata[reg].shunt_resistor = pval;
		}
		else{
	  		pdata->cpdata[reg].shunt_resistor = U32_MINUS_1;
		}
  		valid_channel++;
  	}

  	if (!valid_channel){
  		return ERR_PTR(-EINVAL);
	}
  	return pdata;
}


static int ina3221_probe (struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ina3221_chip *chip;
	struct iio_dev *indio_dev;
	struct ina3221_platform_data *pdata;
	int ret;

	pdata = ina3221_get_platform_data_dt(client);
	if (IS_ERR(pdata)) {
		ret = PTR_ERR(pdata);
		dev_err(&client->dev, "platform data processing failed %d\n",
			ret);
		return ret;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!indio_dev) {
		dev_err(&client->dev, "iio allocation fails\n");
		return -ENOMEM;
	}

	chip = iio_priv(indio_dev);
	chip->dev = &client->dev;
	chip->client = client;
	i2c_set_clientdata(client, indio_dev);
	chip->pdata = pdata;
	mutex_init(&chip->mutex);



	indio_dev->info = &ina3221_info;
	indio_dev->channels = ina3221_channels_spec;
	indio_dev->num_channels = ARRAY_SIZE(ina3221_channels_spec);
	indio_dev->name = id->name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	ret = devm_iio_device_register(chip->dev, indio_dev);
	if (ret < 0) {
		dev_err(chip->dev, "iio registration fails with error %d\n",
			ret);
		return ret;
	}

	/* reset ina3221 */
	ret = i2c_smbus_write_word_data(client, INA3221_CONFIG,
		__constant_cpu_to_be16((INA3221_RESET)));
	if (ret < 0) {
		dev_err(&client->dev, "ina3221 reset failure status: 0x%x\n",
			ret);
		return ret;
	}
	ret = ina3221_set_limits(chip);
	if (ret < 0) {
		dev_info(&client->dev, "Not able to set warn and crit limits!\n");
	}

	ret = ina3221_set_limits(chip);

	return ret;
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
  	},
  	.probe		= ina3221_probe,
  	.id_table	= ina3221_id,
};

module_i2c_driver(ina3221_driver);


MODULE_DESCRIPTION("TI INA3221 3-Channel Shunt and Bus Voltage Monitor");
MODULE_AUTHOR("Deepak Nibade <dnibade@nvidia.com>");
MODULE_AUTHOR("Timo Alho <talho@nvidia.com>");
MODULE_AUTHOR("Anshul Jain <anshulj@nvidia.com>");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
