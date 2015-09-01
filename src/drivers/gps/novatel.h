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
 ****************************************************************************/

/**
 * edited: Stephan Zuercher stephan.zuercher@stud.hslu.ch
 */

/* @file novatel.h */

#ifndef NOVATEL_H_
#define NOVATEL_H_



#include "gps_helper.h"

#define NOVATEL_SYNC1 0xAA
#define NOVATEL_SYNC2 0x44
#define NOVATEL_SYNC3 0x12
/* MessageIDs (the ones that are used) */
#define NOVATEL_BESTPOS 42
#define NOVATEL_BESTVEL 99
#define NOVATEL_PSRPOS	47
#define NOVATEL_PSRDOP	174
#define NOVATEL_HEADING 971

#define CRC32_POLYNOMIAL 0xEDB88320


#define NOVATEL_UNLOGALL		"UNLOGALL\r\n"
#define NOVATEL_BESTPOS_5HZ		"log bestposb ontime 0.2\r\n"
#define NOVATEL_BESTVEL_5HZ		"log bestvelb ontime 0.2\r\n"
#define NOVATEL_HEADING_5HZ		"log headingb ONCHANGED\r\n"
#define SBAS_ON	        		"sbascontrol enable auto\r\n"
#define MAGVAR_ZERO				"magvar correction 0 0\r\n"

#define NOVATEL_BAUDRATE 115200
#define NOVATEL_WAIT_BEFORE_READ_USEC 1


#define NOVATEL_FIXTYPE_NONE 0 				// No solution
#define NOVATEL_FIXTYPE_FIXEDPOS 1 			// Position has been fixed by the FIX POSITION command
#define NOVATEL_FIXTYPE_FIXEDHEIGHT 2 		// Position has been fixed by the FIX HEIGHT/AUTO command
#define NOVATEL_FIXTYPE_DOPPLER_VELOCITY 8  // Velocity computed using instantaneous Doppler
#define NOVATEL_FIXTYPE_SINGLE 16 			// Single point position
#define NOVATEL_FIXTYPE_PSRDIFF 17 			// Pseudorange differential solution
#define NOVATEL_FIXTYPE_WAAS 18 			// Solution calculated using corrections from an SBAS
#define NOVATEL_FIXTYPE_PROPAGATED 19 		// Propagated by a Kalman filter without new observations
#define NOVATEL_FIXTYPE_OMNISTAR 20 		// a OmniSTAR VBS position (L1 sub-metre)
#define NOVATEL_FIXTYPE_L1_FLOAT 32 		// Floating L1 ambiguity solution
#define NOVATEL_FIXTYPE_IONOFREE_FLOAT 33 	// Floating ionospheric-free ambiguity solution
#define NOVATEL_FIXTYPE_NARROW_FLOAT 34 	// Floating narrow-lane ambiguity solution
#define NOVATEL_FIXTYPE_L1_INT 48 			// Integer L1 ambiguity solution
#define NOVATEL_FIXTYPE_WIDE_INT 49 		// Integer wide-lane ambiguity solution
#define NOVATEL_FIXTYPE_NARROW_INT 50 		// Integer narrow-lane ambiguity solution
#define NOVATEL_FIXTYPE_RTK_DIRECT_INS 51 	// RTK status where the RTK filter is directly initialized from the INS filter
#define NOVATEL_FIXTYPE_INS 52 				// INS calculated position corrected for the antenna
#define NOVATEL_FIXTYPE_INS_PSRSP 53 		// INS pseudorange single point solution - no DGPS corrections
#define NOVATEL_FIXTYPE_INS_PSRDIFF 54 		// INS pseudorange differential solution
#define NOVATEL_FIXTYPE_INS_RTKFLOAT 55 	// INS RTK floating point ambiguities solution
#define NOVATEL_FIXTYPE_INS_RTKFIXED 56 	// INS RTK fixed ambiguities solution
#define NOVATEL_FIXTYPE_OMNISTAR_HP 64 		// OmniSTAR HP position
#define NOVATEL_FIXTYPE_OMNISTAR_XP 65 		// OmniSTAR XP position
#define NOVATEL_FIXTYPE_CDGPS 66 			// Position solution using CDGPS correction



typedef enum {
	NOVATEL_DECODE_UNINIT = 0,
	NOVATEL_DECODE_GOT_SYNC1,
	NOVATEL_DECODE_GOT_SYNC2,
	NOVATEL_DECODE_GOT_SYNC3,
	NOVATEL_DECODE_GOT_HEADER_LGTH,
	NOVATEL_DECODE_GOT_MESSAGE_LGTH,
	NOVATEL_DECODE_GOT_CRC1,
	NOVATEL_DECODE_GOT_CRC2,
	NOVATEL_DECODE_GOT_CRC3
} novatel_decode_state_t;

typedef enum {
	NOVATEL_TIME_STATUS_UNKNOWN = 20,
	NOVATEL_TIME_STATUS_APPROXIMATE = 60,
	NOVATEL_TIME_STATUS_COARSEADJUSTING = 80,
	NOVATEL_TIME_STATUS_COARSE = 100,
	NOVATEL_TIME_STATUS_COARSESTEERING = 120,
	NOVATEL_TIME_STATUS_FREEWHEELING = 130,
	NOVATEL_TIME_STATUS_FINEADJUSTING = 140,
	NOVATEL_TIME_STATUS_FINE = 160,
	NOVATEL_TIME_STATUS_FINESTEERING = 180,
	NOVATEL_TIME_STATUS_SATTIME = 200
} novatel_gps_time_status;


/** the structures of the binary packets */
#pragma pack(push, 1)

typedef struct {
	uint32_t solstat; /**< Solution status */
	uint32_t postype; /**< Position type */
	double_t lat; /**< Latitude, deg */
	double_t lon; /**< Longitude, deg */
	double_t hgt; /**< Height above mean sea level, m */
	float undulation; /**< Undulation - the relationship between the geoid and the ellipsoid (m) of the chosen datum */
	uint32_t datum_id_nr; /**< Datum ID number */
	float sigma_lat; /**< Latitude standard deviation, deg */
	float sigma_lon; /**< Longitude standard deviation, deg */
	float sigma_hgt; /**< Height standard deviation, m */
	uint32_t stn_id; /**< Base station ID */
	float dgps_age; /**< Differential age, s */
	float sol_age; /**< Solution age, s */
	uint8_t sat_tracked; /**< Number of satellite vehicles tracked */
	uint8_t sat_used; /**< Number of satellite vehicles used in solution */
	uint8_t ggL1; /**< Number of GPS plus GLONASS L1 used in solution */
	uint8_t ggL1L2; /**< Number of GPS plus GLONASS L1 and L2 used in solution */
	uint8_t reserved_1; /**< Reserved */
	uint8_t ext_sol_stat; /**< Extended solution status */
	uint8_t reserved_2; /**< Reserved */
	uint8_t sig_mask; /**< Signals used mak - if 0, signals used in solution are unknown */
} gps_novatel_bestpos_packet_t;

typedef struct {
	uint32_t solstat; /**< Solution status */
	uint32_t veltype; /**< Velocity type */
	float latency; /**< A measure of the latency in the velocity time tag in seconds. It should be subtracted from the time to give improved results */
	float dgps_age; /**< Differential age, s */
	double_t hor_spd; /**< Horizontal speed over ground, in meters per second */
	double_t trk_gnd; /**< Actual direction of motion over ground (track over ground) with respect to True North, in degrees */
	double_t vert_spd; /**< Vertical speed, in meters per second, where positive values indicate increasing altitude (up) and negative values indicate decreasing altitude (down) */
	float reserved; /**< Reserved */
} gps_novatel_bestvel_packet_t;


typedef struct {
	uint64_t mc_time;
	uint16_t week;
	uint32_t ms;
	uint8_t timeState;
} gps_novatel_time_t;

typedef struct {
	uint32_t solstat;
	uint32_t postype;
	float length;
	float heading;
	float pitch;
	float reserved;
	float hdg_std_dev;
	float ptch_std_dev;
	uint8_t stationName[4];
	uint8_t num_sat_track;
	uint8_t num_sat_solution;
	uint8_t num_sat_above_elevation;
	uint8_t num_sat_above_L2;
	uint8_t sol_src;
	uint8_t ext_sol_stat;
	uint8_t Galileo_BeiDou_mask;
	uint8_t GPS_mask; 
} gps_novatel_heading_packet_t;


typedef struct {
	float gdop; /**< Geometric dilution of precision */
	float pdop; /**< Position dilution of precision */
	float hdop; /**< Horizontal dilution of precision */
	float htdop; /**< Horizontal position and time dilution of precision*/
	float tdop; /**< Time dilution of precision */
	float cutoff; /**< Elevation cut-off angle*/
	long	Number_PRN; /**< Number of satellites PRNs to follow*/
	unsigned long PRN1;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN2;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN3;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN4;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN5;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN6;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN7;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN8;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN9;	/**< PRN of SV PRN tracking, null field until position solution available*/
	unsigned long PRN10;	/**< only 10 PRN implemented at the moment*/
} gps_novatel_psrdop_packet_t;


typedef struct {
	// todo implement it



	uint32_t solstat; /**< Solution status */
	uint32_t veltype; /**< Velocity type */
	float latency; /**< A measure of the latency in the velocity time tag in seconds. It should be subtracted from the time to give improved results */
	float dgps_age; /**< Differential age, s */
	double_t hor_spd; /**< Horizontal speed over ground, in meters per second */
	double_t trk_gnd; /**< Actual direction of motion over ground (track over ground) with respect to True North, in degrees */
	double_t vert_spd; /**< Vertical speed, in meters per second, where positive values indicate increasing altitude (up) and negative values indicate decreasing altitude (down) */
	float reserved; /**< Reserved */
} gps_novatel_psrpos_packet_t;



#include <uORB/topics/novatel_bestpos.h>
#include <uORB/topics/novatel_bestvel.h>
#include <uORB/topics/novatel_heading.h>


#pragma pack(pop)

#define NOVATEL_RECV_BUFFER_SIZE 1500

class NOVATEL : public GPS_Helper
{
public:
	NOVATEL(const int &fd, struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info,struct vehicle_gps_heading_s *gps_heading);
	~NOVATEL();
	int				receive(unsigned timeout);
	int				configure(unsigned &baudrate);

private:
	/**
	 * Parse the binary NOVATEL packet
	 */
	int				parse_char(uint8_t b);

	/**
	 * Handle the package once it has arrived
	 */
	int				handle_message(void);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void				decode_init(void);

	/**
	 * Calculate the CRC-32 of a block of data
	 */
	unsigned long		calculate_block_crc32(unsigned long message_lgth, unsigned char *data);
	unsigned long 		CRC32Value(int i);

	int					_fd;
	struct vehicle_gps_position_s *_gps_position;
	struct vehicle_gps_heading_s *_gps_heading;
	struct satellite_info_s *_satellite_info;

	gps_novatel_time_t		_gps_time;
	struct novatel_bestvel_s	_best_vel;
	struct novatel_bestpos_s	_best_pos;
	struct novatel_heading_s	_heading;

	orb_advert_t	_report_best_pos=0;
	orb_advert_t	_report_best_vel=0;
	orb_advert_t	_report_heading=0;


	novatel_decode_state_t	_decode_state;
	uint8_t				_novatel_revision;
	uint8_t				_rx_header_lgth;
	uint16_t			_rx_message_lgth;
	uint16_t			_rx_message_id;
	uint8_t				_rx_buffer[NOVATEL_RECV_BUFFER_SIZE];
	uint8_t				_crc_buffer[4];
	uint32_t			_rx_count;
	uint32_t 			_rx_crc;
	uint64_t		_calculated_crc;
	novatel_gps_time_status		_rx_message_time_status;
	uint16_t 			_rx_message_week;
	uint32_t			_rx_message_ms;
	bool			_got_posllh;
	bool			_got_velned;
};

#endif /* NOVATEL_H_ */
