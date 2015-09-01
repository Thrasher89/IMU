/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @Stephan Zuercher stephan.zuercher@stud.hslu.ch
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/uart.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <lib/conversion/rotation.h>

#include "stim300ctl.h"
#include <nuttx/sched.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/scheduling_priorities.h>



#define STIM300_SERIAL_PORT_DEVICE_PATH			PX4IO_SERIAL_DEVICE
#define STIM300_SERIAL_PORT_DEVICE_BAUDRATE		921600
#define STIM300_DEFAULT_RATE				100


#define STIM300_DEVICE_PATH_ACCEL			"/dev/STIM300_accel"
#define STIM300_DEVICE_PATH_GYRO			"/dev/STIM300_gyro"
#define STIM300_DEVICE_PATH_ACCEL_EXT			"/dev/STIM300_accel_ext"
#define STIM300_DEVICE_PATH_GYRO_EXT			"/dev/STIM300_gyro_ext"

#define STIM300_ACCEL_DEFAULT_RANGE_G			8

#define STIM300_GYRO_DEFAULT_RANGE_G			8

#define STIM300_ONE_G					9.80665f


class STIM300;
STIM300 *g_dev_ptr = nullptr;

class STIM300_gyro;

class STIM300 : public device::UART, STIM300CTL
{
public:
	STIM300(const char *uart_dev_path,unsigned uart_speed, const char *path_accel, const char *path_gyro, enum Rotation rotation);
	virtual ~STIM300();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	void			print_info();

	void			goServiceMode(const char *);
	void			stream(int i);

protected:
	virtual int		probe();

	friend class STIM300_gyro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	STIM300_gyro		*_gyro;
	uint8_t			_product;	/** product code */

	struct hrt_call		_call;
	unsigned		_call_interval;

	RingBuffer		*_accel_reports;

	struct accel_scale	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	orb_id_t		_accel_orb_id;
	int			_accel_class_instance;

	RingBuffer		*_gyro_reports;

	struct gyro_scale	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;

	unsigned		_sample_rate;
	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;

	unsigned		debugCnt=0;


	enum Rotation		_rotation;

	/**
	 * Start automatic measurement.
	 */
	void			start();


	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 */
	void			reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();


	/**
	 * Set the STIM300 measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the STIM300 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			gyro_self_test();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(uint16_t desired_sample_rate_hz);

	/* do not allow to copy this class due to pointer data members */
	STIM300(const STIM300&);
	STIM300 operator=(const STIM300&);
};

/**
 * Helper class implementing the gyro driver node.
 */
class STIM300_gyro : public device::CDev
{
public:
	STIM300_gyro(STIM300 *parent, const char *path);
	~STIM300_gyro();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class STIM300;

	void			parent_poll_notify();

private:
	STIM300			*_parent;
	orb_advert_t		_gyro_topic;
	orb_id_t		_gyro_orb_id;
	int			_gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	STIM300_gyro(const STIM300_gyro&);
	STIM300_gyro operator=(const STIM300_gyro&);
};

/** driver 'main' command */
extern "C" { __EXPORT int stim300_main(int argc, char *argv[]); }

void
STIM300::goServiceMode(const char * input){

}

STIM300::STIM300(const char *uart_dev_path,unsigned uart_speed, const char *path_accel, const char *path_gyro, enum Rotation rotation):
	UART("STIM300",path_accel, uart_dev_path,uart_speed),
	STIM300CTL((device::UART*) this),
	_gyro(new STIM300_gyro(this, path_gyro)),
	_product(0),
	_call{},
	_call_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(-1),
	_accel_orb_id(nullptr),
	_accel_class_instance(-1),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_sample_rate(1000),
	_accel_reads(perf_alloc(PC_COUNT, "STIM300_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "STIM300_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "STIM300_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "STIM300_bad_transfers")),
	_rotation(rotation)
{
	// disable debug() calls
	_debug_enabled = true;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
}

STIM300::~STIM300()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	/* free any existing reports */
	if (_accel_reports != nullptr)
		delete _accel_reports;
	if (_gyro_reports != nullptr)
		delete _gyro_reports;

	if (_accel_class_instance != -1)
		unregister_class_devname(ACCEL_DEVICE_PATH, _accel_class_instance);

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	printf("STIM300 in destructor");
}

int
STIM300::init()
{
	int ret;


	/* allocate basic report buffers */
	_accel_reports = new RingBuffer(2, sizeof(accel_report));
	if (_accel_reports == nullptr)
		goto out;

	_gyro_reports = new RingBuffer(2, sizeof(gyro_report));
	if (_gyro_reports == nullptr)
		goto out;

	reset();

	/* Initialize offsets and scales */
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();
	/* if probe/setup failed, bail now */
	if (ret != OK) {
		debug("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_DEVICE_PATH);

	//measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	switch (_accel_class_instance) {
		case CLASS_DEVICE_PRIMARY:
			_accel_orb_id = ORB_ID(sensor_accel0);
			break;
		
		case CLASS_DEVICE_SECONDARY:
			_accel_orb_id = ORB_ID(sensor_accel1);
			break;

		case CLASS_DEVICE_TERTIARY:
			_accel_orb_id = ORB_ID(sensor_accel2);
			break;

	}

	_accel_topic = orb_advertise(_accel_orb_id, &arp);

	if (_accel_topic < 0) {
		warnx("ADVERT FAIL");
	}


	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_gyro_reports->get(&grp);

	switch (_gyro->_gyro_class_instance) {
		case CLASS_DEVICE_PRIMARY:
			_gyro->_gyro_orb_id = ORB_ID(sensor_gyro0);
			break;

		case CLASS_DEVICE_SECONDARY:
			_gyro->_gyro_orb_id = ORB_ID(sensor_gyro1);
			break;

		case CLASS_DEVICE_TERTIARY:
			_gyro->_gyro_orb_id = ORB_ID(sensor_gyro2);
			break;

	}

	_gyro->_gyro_topic = orb_advertise(_gyro->_gyro_orb_id, &grp);

	if (_gyro->_gyro_topic < 0) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}

void STIM300::reset()
{
	/*
	// if the STIM300 is initialised after the l3gd20 and lsm303d
	// then if we don't do an irqsave/irqrestore here the STIM300
	// frequenctly comes up in a bad state where all transfers
	// come as zero
	irqstate_t state;
	state = irqsave();

	write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	up_udelay(10000);

	// Wake up device and select GyroZ clock. Note that the
	// STIM300 starts up in sleep mode, and it can take some time
	// for it to come out of sleep
	write_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	up_udelay(1000);

	// Disable I2C bus (recommended on datasheet)
	write_reg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
        irqrestore(state);

	usleep(1000);

	// SAMPLE RATE
	_set_sample_rate(_sample_rate);
	usleep(1000);

	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this ruins quality and does not improve the
	// system response
	_set_dlpf_filter(STIM300_DEFAULT_ONCHIP_FILTER_FREQ);
	usleep(1000);
	// Gyro scale 2000 deg/s ()
	write_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	usleep(1000);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	_gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

	// product-specific scaling
	switch (_product) {
	case STIM300ES_REV_C4:
	case STIM300ES_REV_C5:
	case STIM300_REV_C4:
	case STIM300_REV_C5:
		// Accel scale 8g (4096 LSB/g)
		// Rev C has different scaling than rev D
		write_reg(MPUREG_ACCEL_CONFIG, 1 << 3);
		break;

	case STIM300ES_REV_D6:
	case STIM300ES_REV_D7:
	case STIM300ES_REV_D8:
	case STIM300_REV_D6:
	case STIM300_REV_D7:
	case STIM300_REV_D8:
	case STIM300_REV_D9:
	case STIM300_REV_D10:
	// default case to cope with new chip revisions, which
	// presumably won't have the accel scaling bug		
	default:
		// Accel scale 8g (4096 LSB/g)
		write_reg(MPUREG_ACCEL_CONFIG, 2 << 3);
		break;
	}

	// Correct accel scale factors of 4096 LSB/g
	// scale to m/s^2 ( 1g = 9.81 m/s^2)
	_accel_range_scale = (STIM300_ONE_G / 4096.0f);
	_accel_range_m_s2 = 8.0f * STIM300_ONE_G;

	usleep(1000);

	// INT CFG => Interrupt on Data Ready
	write_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
	usleep(1000);
	write_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	usleep(1000);

	// Oscillator set
	// write_reg(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
	usleep(1000);
	*/

}

int
STIM300::probe()
{

	/* look for a product ID we recognise */
	/*
	_product = read_reg(MPUREG_PRODUCT_ID);

	// verify product revision
	switch (_product) {
	case STIM300ES_REV_C4:
	case STIM300ES_REV_C5:
	case STIM300_REV_C4:
	case STIM300_REV_C5:
	case STIM300ES_REV_D6:
	case STIM300ES_REV_D7:
	case STIM300ES_REV_D8:
	case STIM300_REV_D6:
	case STIM300_REV_D7:
	case STIM300_REV_D8:
	case STIM300_REV_D9:
	case STIM300_REV_D10:
		debug("ID 0x%02x", _product);
		return OK;
	}

	debug("unexpected ID 0x%02x", _product);
	return -EIO;
	*/
	return OK;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
STIM300::_set_sample_rate(uint16_t desired_sample_rate_hz)
{
	/*
  uint8_t div = 1000 / desired_sample_rate_hz;
  if(div>200) div=200;
  if(div<1) div=1;
  write_reg(MPUREG_SMPLRT_DIV, div-1);
  _sample_rate = 1000 / div;
  */
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
STIM300::_set_dlpf_filter(uint16_t frequency_hz)
{
	/*
	uint8_t filter;

        if (frequency_hz == 0) {
		filter = BITS_DLPF_CFG_2100HZ_NOLPF;
        } else if (frequency_hz <= 5) {
		filter = BITS_DLPF_CFG_5HZ;
	} else if (frequency_hz <= 10) {
		filter = BITS_DLPF_CFG_10HZ;
	} else if (frequency_hz <= 20) {
		filter = BITS_DLPF_CFG_20HZ;
	} else if (frequency_hz <= 42) {
		filter = BITS_DLPF_CFG_42HZ;
	} else if (frequency_hz <= 98) {
		filter = BITS_DLPF_CFG_98HZ;
	} else if (frequency_hz <= 188) {
		filter = BITS_DLPF_CFG_188HZ;
	} else if (frequency_hz <= 256) {
		filter = BITS_DLPF_CFG_256HZ_NOLPF2;
	} else {
		filter = BITS_DLPF_CFG_2100HZ_NOLPF;
	}
	write_reg(MPUREG_CONFIG, filter);
	*/
}

ssize_t
STIM300::read(struct file *filp, char *buff, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);

	printf("in read\n");
	exit(-9);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		//measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty())
		return -EAGAIN;

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buff);
	int transferred = 0;
	while (count--) {
		if (!_accel_reports->get(arp))
			break;
		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));
}

int
STIM300::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		//measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
STIM300::accel_self_test()
{
	if (self_test())
		return 1;

	/* inspect accel offsets */
	if (fabsf(_accel_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f)
		return 1;

	if (fabsf(_accel_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f)
		return 1;

	if (fabsf(_accel_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f)
		return 1;

	return 0;
}

int
STIM300::gyro_self_test()
{
	if (self_test())
		return 1;

	/* evaluate gyro offsets, complain if offset -> zero or larger than 6 dps */
	if (fabsf(_gyro_scale.x_offset) > 0.1f || fabsf(_gyro_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.y_offset) > 0.1f || fabsf(_gyro_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.z_offset) > 0.1f || fabsf(_gyro_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f)
		return 1;

	return 0;
}

ssize_t
STIM300::gyro_read(struct file *filp, char *buff, size_t buflen)
{
	unsigned count = buflen / sizeof(gyro_report);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_gyro_reports->flush();
		//measure();
	}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty())
		return -EAGAIN;

	perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buff);
	int transferred = 0;
	while (count--) {
		if (!_gyro_reports->get(grp))
			break;
		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(gyro_report));
}

int
STIM300::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	printf("in ioctl %d\n",cmd);
	switch (cmd) {

	case SENSORIOCRESET:
		reset();
		return OK;

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, 2000);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, STIM300_DEFAULT_RATE);

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 500)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call.period = _call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		 //TODO SERIAL baudrate
		 /*
		if (_call_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_interval;
		*/

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;
		
		irqstate_t flags = irqsave();
		if (!_accel_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);
		
		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _accel_reports->size();

	case ACCELIOCGSAMPLERATE:
		return _sample_rate;

	case ACCELIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case ACCELIOCGLOWPASS:
		return -EINVAL;

	case ACCELIOCSLOWPASS:
		// set hardware filtering
		//TODO
		_set_dlpf_filter(arg);
		return -EINVAL;
		return OK;

	case ACCELIOCSSCALE:
		{
			/* copy scale, but only if off by a few percent */
			struct accel_scale *sc = (struct accel_scale *) arg;
			float sum = sc->x_scale + sc->y_scale + sc->z_scale;
			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;
			} else {
				return -EINVAL;
			}
		}

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		/* not implemented */
		return -EINVAL;
	case ACCELIOCGRANGE:
		return (unsigned long)((_accel_range_m_s2)/STIM300_ONE_G + 0.5f);

	case ACCELIOCSELFTEST:
		return accel_self_test();

	default:
		/* give it to the superclass */
		return UART::ioctl(filp, cmd, arg);
	}
}

int
STIM300::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

		/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
	case SENSORIOCSQUEUEDEPTH:
	case SENSORIOCGQUEUEDEPTH:
	case GYROIOCGSAMPLERATE:
	case GYROIOCSSAMPLERATE:
	case GYROIOCGLOWPASS:
	case GYROIOCSLOWPASS:
		/* send to parent (stim300)*/
		ioctl(filp,cmd, arg);

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		/*TODO*/
		return -EINVAL;
	case GYROIOCGRANGE:
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return gyro_self_test();

	default:
		/* give it to the superclass */
		return UART::ioctl(filp, cmd, arg);
	}
}

int
STIM300::set_range(unsigned max_g)
{
#if 0
	uint8_t rangebits;
	float rangescale;

	if (max_g > 16) {
		return -ERANGE;

	} else if (max_g > 8) {		/* 16G */
		rangebits = OFFSET_LSB1_RANGE_16G;
		rangescale = 1.98;

	} else if (max_g > 4) {		/* 8G */
		rangebits = OFFSET_LSB1_RANGE_8G;
		rangescale = 0.99;

	} else if (max_g > 3) {		/* 4G */
		rangebits = OFFSET_LSB1_RANGE_4G;
		rangescale = 0.5;

	} else if (max_g > 2) {		/* 3G */
		rangebits = OFFSET_LSB1_RANGE_3G;
		rangescale = 0.38;

	} else if (max_g > 1) {		/* 2G */
		rangebits = OFFSET_LSB1_RANGE_2G;
		rangescale = 0.25;

	} else {			/* 1G */
		rangebits = OFFSET_LSB1_RANGE_1G;
		rangescale = 0.13;
	}

	/* adjust sensor configuration */
	modify_reg(ADDR_OFFSET_LSB1, OFFSET_LSB1_RANGE_MASK, rangebits);
	_range_scale = rangescale;
#endif
	return OK;
}

void
STIM300::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();


	/* start polling at the specified rate */
	//hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&STIM300::measure_trampoline, this);
	task_spawn_cmd("stim serial read", SCHED_FIFO, SCHED_PRIORITY_FAST_DRIVER, 1500, (main_t)&STIM300::measure_trampoline, nullptr);
	//STIM300::measure();
}

void
STIM300::stop()
{
	hrt_cancel(&_call);
}

void
STIM300::measure_trampoline(void *arg)
{
	printf("in trampoline\n");

	//STIM300 *dev = reinterpret_cast<STIM300 *>(arg);

	/* make another measurement */
	g_dev_ptr->measure();
}

void
STIM300::measure()
{
	int ret;
	/* do UART init (and probe) first */
	ret = UART::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		debug("UART setup failed");
	}

	ret = STIM300CTL::init();
	if (ret != OK) {
		debug("STIM300CTL init failed");
	}
#pragma pack(push, 1)
	/**
	 * Report conversation within the STIM300, including command byte and
	 * interrupt status.
	 */
#pragma pack(pop)

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report;



	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the STIM300 in one pass.
	 */

	/*
	mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;

        // sensor transfer at high clock speed
        set_frequency(STIM300_HIGH_BUS_SPEED);
	*/

/*
	if (OK != transfer(nullptr, ((uint8_t *)&mpu_report), sizeof(mpu_report)))
		return;
		*/

	/*
	 * Convert from big to little endian
	 */
	/*
	report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
	report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
	report.accel_z = int16_t_from_bytes(mpu_report.accel_z);

	report.temp = int16_t_from_bytes(mpu_report.temp);

	report.gyro_x = int16_t_from_bytes(mpu_report.gyro_x);
	report.gyro_y = int16_t_from_bytes(mpu_report.gyro_y);
	report.gyro_z = int16_t_from_bytes(mpu_report.gyro_z);
	*/


//for(int z=0;z<30000;z++){

	int k =0;

	while(1){


//	sleep(1);
	//usleep(100);//0.5ms

		/*
streamSensorData2fd(0);
continue;
*/

	meas_result res = getSensorData();
	report.accel_x = res.gyroX*STIM300_ONE_G;
	report.accel_y = res.gyroY*STIM300_ONE_G;
	report.accel_z = res.gyroZ*STIM300_ONE_G;

	if(k==0) {
		k=1;
	} 
	else{
		k=0;
		continue;
	}
	

	report.temp = 2;

	report.gyro_x = res.gyroX;
	report.gyro_y = res.gyroY;
	report.gyro_z = res.gyroZ;

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {
		// all zero data - probably a UART bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return;
	}
	    

	/*
	 * Swap axes and negate y
	 */
	/*
	int16_t accel_xt = report.accel_y;
	int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

	int16_t gyro_xt = report.gyro_y;
	int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);
	*/

	/*
	 * Apply the swap
	 */
	/*
	report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.gyro_x = gyro_xt;
	report.gyro_y = gyro_yt;
	*/

	/*
	 * Report buffers.
	 */
	accel_report		arb;
	gyro_report		grb;

	/*
	 * Adjust and scale results to m/s^2.
	 */
	grb.timestamp = arb.timestamp = hrt_absolute_time();
        grb.error_count = arb.error_count = 0; // not reported

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */


	/* NOTE: Axes have been swapped to match the board a few lines above. */

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;
	arb.x= report.accel_x;
	arb.y= report.accel_y;
	arb.z= report.accel_z;

	/*
	arb.x = ((report.accel_x * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	arb.y = ((report.accel_y * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	arb.z = ((report.accel_z * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;
	*/



	// apply user specified rotation
	rotate_3f(_rotation, arb.x, arb.y, arb.z);

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	arb.temperature_raw = report.temp;
	arb.temperature = (report.temp) / 361.0f + 35.0f;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	grb.x = ((report.gyro_x * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	grb.y = ((report.gyro_y * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	grb.z = ((report.gyro_z * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x=grb.x_raw;
	grb.y=grb.y_raw;
	grb.z=grb.z_raw;
	

	// apply user specified rotation
	rotate_3f(_rotation, grb.x, grb.y, grb.z);

	grb.scaling = _gyro_range_scale;
	grb.range_rad_s = _gyro_range_rad_s;

	grb.temperature_raw = report.temp;
	grb.temperature = (report.temp) / 361.0f + 35.0f;
	grb.temperature = 44;
	arb.x= res.accelX*STIM300_ONE_G;
	arb.y= res.accelY*STIM300_ONE_G;
	arb.z= res.accelZ*STIM300_ONE_G;
	grb.x= res.gyroX;
	grb.y= res.gyroY;
	grb.z= res.gyroZ;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);
	_gyro->parent_poll_notify();

	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(_accel_orb_id, _accel_topic, &arb);
	}

	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(_gyro->_gyro_orb_id, _gyro->_gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}
}

void
STIM300::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
}

STIM300_gyro::STIM300_gyro(STIM300 *parent, const char *path) :
	CDev("STIM300_gyro", path),
	_parent(parent),
	_gyro_topic(-1),
	_gyro_orb_id(nullptr),
	_gyro_class_instance(-1)
{
}

STIM300_gyro::~STIM300_gyro()
{
	if (_gyro_class_instance != -1)
		unregister_class_devname(GYRO_DEVICE_PATH, _gyro_class_instance);
}

int
STIM300_gyro::init()
{
	int ret;

	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		debug("gyro init failed");
		return ret;
	}

	_gyro_class_instance = register_class_devname(GYRO_DEVICE_PATH);

	return ret;
}

void
STIM300_gyro::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
STIM300_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->gyro_read(filp, buffer, buflen);
}

int
STIM300_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return _parent->gyro_ioctl(filp, cmd, arg);
}

/**
 * Local functions in support of the shell command.
 */
namespace stim300
{

STIM300	*g_dev_int; // on internal bus
STIM300	*g_dev_ext; // on external bus

void	start(bool, enum Rotation);
void	test(bool);
void	reset(bool);
void	info(bool);
void	usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;
	const char *path_accel = external_bus?STIM300_DEVICE_PATH_ACCEL_EXT:STIM300_DEVICE_PATH_ACCEL;
	const char *path_gyro  = external_bus?STIM300_DEVICE_PATH_GYRO_EXT:STIM300_DEVICE_PATH_GYRO;

	if (g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
		errx(0, "already started");

	/* create the driver */
        if (external_bus) {
		errx(1,"use internal bus, but it where external");
	} else {
		g_dev_ptr = new STIM300(STIM300_SERIAL_PORT_DEVICE_PATH,STIM300_SERIAL_PORT_DEVICE_BAUDRATE, path_accel, path_gyro, rotation);
		// PX4IO_SERIAL_DEVICE = /dev/ttys2
	}


	if (g_dev_ptr == nullptr)
		goto fail;

	if (OK != g_dev_ptr->init())
		goto fail;


	// set the poll rate to default, starts automatic data collection 
	fd = open(path_accel, O_RDONLY);
	printf("done opening: %d: %s\n",fd,path_gyro);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

        close(fd);

	exit(0);
fail:

	if (g_dev_ptr != nullptr) {
            delete (g_dev_ptr);
            g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
	const char *path_accel = external_bus?STIM300_DEVICE_PATH_ACCEL_EXT:STIM300_DEVICE_PATH_ACCEL;
	const char *path_gyro  = external_bus?STIM300_DEVICE_PATH_GYRO_EXT:STIM300_DEVICE_PATH_GYRO;
	accel_report a_report;
	gyro_report g_report;
	ssize_t sz;

	/* get the driver */
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'STIM300 start')",
		    path_accel);

	/* get the driver */
	int fd_gyro = open(path_gyro, O_RDONLY);

	if (fd_gyro < 0)
		err(1, "%s open failed", path_gyro);

	/* reset to manual polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0)
		err(1, "reset to manual polling");

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	warnx("single read");
	warnx("time:     %lld", a_report.timestamp);
	warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
	warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
	warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
	warnx("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
	warnx("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
	warnx("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
	warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
	      (double)(a_report.range_m_s2 / STIM300_ONE_G));

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

	warnx("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);
	warnx("temp:  \t%d\traw 0x%0x", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);


	/* XXX add poll-rate tests here too */

	reset(external_bus);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
	const char *path_accel = external_bus?STIM300_DEVICE_PATH_ACCEL_EXT:STIM300_DEVICE_PATH_ACCEL;
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

        close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
        STIM300 **g_dev_ptr = external_bus?&g_dev_ext:&g_dev_int;
	if (*g_dev_ptr == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
stim300_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XR:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
		default:
			stim300::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")){
		stim300::start(external_bus, rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test"))
		stim300::test(external_bus);

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset"))
		stim300::reset(external_bus);

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info"))
		stim300::info(external_bus);

	/*
	if (!strcmp(verb, "s")){
		g_dev_ptr->goServiceMode(argv[optind+1]);
	}
	*/

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
