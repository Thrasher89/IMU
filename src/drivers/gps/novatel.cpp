/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
 *   Author: Carlo Zgraggen <carlo.zgraggen@hslu.ch>
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
 * Novatel protocol implementation. Following Novatel OEM V protocol specifications.
 *
 *
 ****************************************************************************/
/**
 * edited: Stephan Zuercher stephan.zuercher@stud.hslu.ch
 */


/* @file novatel.cpp */


#include <assert.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>


#include <string.h>
#include <time.h>
#include <unistd.h>

#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_gps_heading.h>
#include <uORB/topics/satellite_info.h>


#include <uORB/topics/satellite_info.h>

#include <drivers/drv_hrt.h>


#include "novatel.h"


NOVATEL::NOVATEL(const int &fd, struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info,struct vehicle_gps_heading_s *gps_heading ) :
_fd(fd),
_gps_position(gps_position),
_gps_heading(gps_heading),
_satellite_info(satellite_info),
_novatel_revision(0)
{
	decode_init();
}

NOVATEL::~NOVATEL()
{
}

int
NOVATEL::configure(unsigned &baudrate)
{

	// set baudrate first
	if (GPS_Helper::set_baudrate(_fd, NOVATEL_BAUDRATE) != 0)
		return -1;

	baudrate = NOVATEL_BAUDRATE;

	/*
	printf("write config\n");
	// Write config messages, don't wait for an answer
	if (strlen(NOVATEL_UNLOGALL) != write(_fd, NOVATEL_UNLOGALL, strlen(NOVATEL_UNLOGALL))) {
		warnx("novatel: config write failed");
		return -1;
	}
	usleep(3000);
	if (strlen(NOVATEL_HEADING_5HZ) != write(_fd, NOVATEL_HEADING_5HZ, strlen(NOVATEL_HEADING_5HZ))) {
		warnx("novatel: config write failed");
		return -1;
	}
	usleep(1000);
	if (strlen(NOVATEL_BESTPOS_5HZ) != write(_fd, NOVATEL_BESTPOS_5HZ, strlen(NOVATEL_BESTPOS_5HZ))) {
		warnx("novatel: config write failed");
		return -1;
	}
	usleep(1000);

	if (strlen(NOVATEL_BESTVEL_5HZ) != write(_fd, NOVATEL_BESTVEL_5HZ, strlen(NOVATEL_BESTVEL_5HZ))) {
		warnx("novatel: config write failed");
		return -1;
	}
	usleep(1000);

	if (strlen(SBAS_ON) != write(_fd, SBAS_ON, strlen(SBAS_ON))) {
		warnx("novatel: config write failed");
		return -1;
	}
	usleep(10000);

	if (strlen(MAGVAR_ZERO) != write(_fd, MAGVAR_ZERO, strlen(MAGVAR_ZERO))) {
		warnx("novatel: config write failed");
		return -1;
	}
	usleep(10000);
*/


	return 0;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
NOVATEL::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	static uint8_t buf[512];

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	static int j = 0;
	static ssize_t count = 0;

	while (true) {
		/* first read whatever is left */
		if (j < count) {
			/* pass received bytes to the packet decoder */
			//warnx("buf: %d", count);
			while (j < count) {
				if (parse_char(buf[j]) > 0) {
				fflush(stdout);
					int ret = handle_message();
					if (ret == 1) //return 1;			// pos update
						return 1;
					else if (ret == 2) return 1;	// vel update
					else if (ret == 3) return 2;	// sat info update
					else if (ret == 4) return 4;	// heading update
					else  return -1;		// no known message
				}

				/* in case we keep trying but only get crap from GPS */
				if (time_started + timeout * 1000 < hrt_absolute_time()) {
					return -1;
				}

				j++;
			}

			/* everything is read */
			j = count = 0;


		}

		/* then poll for new data */
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (ret == 0) {
			/* Timeout */
			return -1;

		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				//usleep(NOVATEL_WAIT_BEFORE_READ_USEC);
				count = ::read(_fd, buf, sizeof(buf));
			}
		}

	} //end while
}

void
NOVATEL::decode_init(void)
{
	_rx_count = 0;
	_decode_state = NOVATEL_DECODE_UNINIT;
	_rx_header_lgth = 0;
	_rx_message_id = 0;
	_rx_message_lgth = 0;
	_rx_crc = 0;
	_rx_message_time_status = NOVATEL_TIME_STATUS_UNKNOWN;
	_rx_message_week = 0;
	_rx_message_ms = 0;
}


int
NOVATEL::parse_char(uint8_t b)
{
	switch (_decode_state) {
		/* First, look for start 'AA' */
		case NOVATEL_DECODE_UNINIT:
			if (b == NOVATEL_SYNC1) {
				_decode_state = NOVATEL_DECODE_GOT_SYNC1;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				//warnx("[GPS] missed start byte: %d", b);
			}
			break;
		/* Second, look for sync2 */
		case NOVATEL_DECODE_GOT_SYNC1:
			if (b == NOVATEL_SYNC2) {
				_decode_state = NOVATEL_DECODE_GOT_SYNC2;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				/* Second start symbol was wrong, reset state machine */
				decode_init();
				//warnx("[GPS] second start byte wrong");
			}
			break;
		/* Third, look for sync3 */
		case NOVATEL_DECODE_GOT_SYNC2:
			if (b == NOVATEL_SYNC3) {
				_decode_state = NOVATEL_DECODE_GOT_SYNC3;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				/* Third start symbol was wrong, reset state machine */
				decode_init();
				//warnx("[GPS] third start byte wrong");
			}
			break;
		/* Get the length of the header */
		case NOVATEL_DECODE_GOT_SYNC3:
			//warnx("GOT Sync 3 byte");
			_decode_state = NOVATEL_DECODE_GOT_HEADER_LGTH;
			_rx_header_lgth = b;
			_rx_buffer[_rx_count] = b;
			_rx_count++;
			break;
		/* Get the rest of the header */
		case NOVATEL_DECODE_GOT_HEADER_LGTH:
			if (_rx_count < _rx_header_lgth) {
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				//warnx("GOT Header length %d", _rx_header_lgth);
				_decode_state = NOVATEL_DECODE_GOT_MESSAGE_LGTH;
				_rx_message_id = _rx_buffer[4] + (_rx_buffer[5] << 8);
				_rx_message_lgth = _rx_buffer[8] + (_rx_buffer[9] << 8);
				_gps_time.timeState = _rx_message_time_status = (novatel_gps_time_status) _rx_buffer[13];	//convert to enum
				_gps_time.week = _rx_message_week = _rx_buffer[14] + (_rx_buffer[15] << 8);
				_gps_time.ms= _rx_message_ms = _rx_buffer[16] + (_rx_buffer[17] << 8) + (_rx_buffer[18] << 16) + (_rx_buffer[19] << 24);
				_gps_time.mc_time = hrt_absolute_time();
				_rx_buffer[_rx_count] = b;
				_rx_count++;
				//warnx("GOT Message length %d", _rx_message_lgth);

			}
			break;
		/* Get the message */
		case NOVATEL_DECODE_GOT_MESSAGE_LGTH:
			if (_rx_count < (_rx_header_lgth + _rx_message_lgth)) { //check due to buffer overflow
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				_decode_state = NOVATEL_DECODE_GOT_CRC1;
				_rx_crc = b;
			}
			break;
		/* Get the rest of the crc */
		case NOVATEL_DECODE_GOT_CRC1:
			_decode_state = NOVATEL_DECODE_GOT_CRC2;
			_rx_crc += b << 8;
			break;
		case NOVATEL_DECODE_GOT_CRC2:
			_decode_state = NOVATEL_DECODE_GOT_CRC3;
			_rx_crc += b << 16;
			break;
		case NOVATEL_DECODE_GOT_CRC3:
			_rx_crc += b << 24;

			/* compare checksum */
			_calculated_crc = calculate_block_crc32(_rx_count, _rx_buffer);
			if ( _calculated_crc == _rx_crc) {
				return 1;
			} else {
				warnx("novatel: Checksum wrong. calculated crc: %x, rx crc: %x", _calculated_crc, _rx_crc);
				decode_init();
				return -1;
			}
			break;
		default:
			return -1;
			break;
	}
	return 0;
}

int
NOVATEL::handle_message()
{
	int ret = 0;

	gps_novatel_bestpos_packet_t *packet_bestpos;
	gps_novatel_bestvel_packet_t *packet_bestvel;
	gps_novatel_psrdop_packet_t  *packet_psrdop;
	gps_novatel_heading_packet_t  *packet_heading;
	//gps_novatel_psrpos_packet_t *packet_psrpos;

	switch (_rx_message_id) {
		case NOVATEL_BESTPOS:
			packet_bestpos = (gps_novatel_bestpos_packet_t *) (&(_rx_buffer[_rx_header_lgth]));

			_gps_position->timestamp_position = hrt_absolute_time();
			_gps_position->lat = (int32_t) (packet_bestpos->lat * 1e7);
			_gps_position->lon = (int32_t) (packet_bestpos->lon * 1e7);
			_gps_position->alt = (int32_t) (packet_bestpos->hgt * 1e3);


			_gps_position->timestamp_variance = hrt_absolute_time();
			_gps_position->c_variance_rad = (packet_bestpos->sigma_hgt) * (packet_bestpos->sigma_hgt);	// var = sigma^2
			_gps_position->fix_type = (packet_bestpos->postype >= NOVATEL_FIXTYPE_SINGLE) ? 3 : 0;	// there is no 2d fix

			_gps_position->satellites_used = packet_bestpos->sat_tracked;

			if(_rx_message_time_status >= NOVATEL_TIME_STATUS_FINEADJUSTING ){
				_gps_position->timestamp_time = hrt_absolute_time();
				_gps_position->time_gps_usec = ((uint64_t) _rx_message_ms) * 1000;
			}
			_rate_count_lat_lon++;
			_got_posllh = true;

			_best_pos.time=_gps_time;
			_best_pos.data=*packet_bestpos;
			if(_report_best_pos>0)
				orb_publish(ORB_ID(novatel_bestpos), _report_best_pos, &_best_pos);
			else
				_report_best_pos=orb_advertise(ORB_ID(novatel_bestpos), &_best_pos);

			ret = 1;
			break;
		case NOVATEL_BESTVEL:
			packet_bestvel = (gps_novatel_bestvel_packet_t *) (&(_rx_buffer[_rx_header_lgth]));

			_gps_position->timestamp_velocity = hrt_absolute_time();
			_gps_position->vel_m_s = (float) packet_bestvel->hor_spd;
			_gps_position->vel_n_m_s = ((float) packet_bestvel->hor_spd) * cosf((	(float) packet_bestvel->trk_gnd) * M_DEG_TO_RAD_F);
			_gps_position->vel_e_m_s = ((float) packet_bestvel->hor_spd) * sinf(((float) packet_bestvel->trk_gnd) * M_DEG_TO_RAD_F);
			_gps_position->vel_d_m_s = -((float) packet_bestvel->vert_spd);
			// configure receiver with "magvar correction 0 0" (default) to be sure that course over ground = heading
			_gps_position->cog_rad = (((float) packet_bestvel->trk_gnd) < 180.0f) ? (( (float) packet_bestvel->trk_gnd) * M_DEG_TO_RAD_F) : ((( (float) packet_bestvel->trk_gnd) - 360.0f) * M_DEG_TO_RAD_F);	// 0-360 to -pi..+pi
			_gps_position->vel_ned_valid = (packet_bestvel->veltype >= NOVATEL_FIXTYPE_SINGLE) ? true : false;
			_gps_position->s_variance_m_s = 0;									//does not exist in novatel protocol

			if(_rx_message_time_status >= NOVATEL_TIME_STATUS_FINEADJUSTING ){
				_gps_position->timestamp_time = hrt_absolute_time();
				_gps_position->time_gps_usec = _rx_message_ms * 1000;
			}

			_rate_count_vel++;
			_got_velned = true;

			_best_vel.time=_gps_time;
			_best_vel.data=*packet_bestvel;
			if(_report_best_vel>0)
				orb_publish(ORB_ID(novatel_bestvel), _report_best_vel, &_best_vel);
			else
				_report_best_vel=orb_advertise(ORB_ID(novatel_bestvel), &_best_vel);


			ret = 2;
			break;
		case NOVATEL_PSRDOP:
			packet_psrdop = (gps_novatel_psrdop_packet_t *) (&(_rx_buffer[_rx_header_lgth]));

			_gps_position->eph = packet_psrdop->hdop;
			_gps_position->epv = sqrt((packet_psrdop->pdop * packet_psrdop->pdop) - (packet_psrdop->hdop * packet_psrdop->hdop));

			ret = 3;	// dirty, 0 means no message handeled.
			break;
		case NOVATEL_HEADING:
			packet_heading = (gps_novatel_heading_packet_t *) (&(_rx_buffer[_rx_header_lgth]));

			_gps_heading->hdg=packet_heading->heading;
			_gps_heading->hdg_deviation=packet_heading->hdg_std_dev;
			_gps_heading->ptch=packet_heading->pitch;
			_gps_heading->ptch_deviation=packet_heading->ptch_std_dev;

			if(_rx_message_time_status >= NOVATEL_TIME_STATUS_FINEADJUSTING ){
				_gps_heading->timestamp_time = hrt_absolute_time();
				_gps_heading->time_gps_usec = _rx_message_ms * 1000;
			}

			_heading.time=_gps_time;
			_heading.data=*packet_heading;
			if(_report_heading>0)
				orb_publish(ORB_ID(novatel_heading), _report_heading, &_heading);
			else
				_report_heading=orb_advertise(ORB_ID(novatel_heading), &_heading);

			ret = 4;
			break;
		default:
			warnx("NOVATEL: Unknown message received: %d\n", _rx_message_id);
			ret = -1;
			break;

	}
	decode_init();
	return ret;
}


unsigned long
NOVATEL::calculate_block_crc32(unsigned long message_lgth, unsigned char *data)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while ( message_lgth-- != 0 )
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ *data++ ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC;
}

unsigned long
NOVATEL::CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for ( j = 8 ; j > 0; j-- )
	{
		if ( ulCRC & 1 )
			ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

