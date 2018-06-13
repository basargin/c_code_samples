// U-BLOX M8 proto parser example

#include <memory.h>
#include <stdio.h>
#include <stdint.h>


// Frame handler for UBX: frame <msg> is stripped of SYNC chars and checksums 
void ubx_frame_received(size_t len, uint8_t* msg) {
	// TODO : implement UBX handler 
};

// Frame handler for NMEA: frame <msg> is stripped of Start char, checksums and End sequence
void nmea_frame_received(size_t len, uint8_t* msg) {
	// TODO : implement NMEA handler 

	// DEBUG
	static size_t cnt = 0;
	char buf[1024];
	size_t n = len > sizeof(buf) ? sizeof(buf) - 1 : len;
	memcpy(buf, msg, n);
	msg[n] = 0;
	printf("#%d : %s\r\n", cnt, msg);
	cnt++;
	// DEBUG
}

// NMEA definitions 
#define NMEA_MAX_LEN		82		// Just well-known 
#define NMEA_MIN_LEN		2
#define NMEA_SOP_TOK		'$'		// Start of packet token
#define NMEA_CS_TOK			'*'		// Checksum token
#define NMEA_EOP_TOK1		13		// End of packet token #1
#define NMEA_EOP_TOK2		10		// End of packet token #2
#define NMEA_MAX_CHR_CODE	127		// 

// UBX definitions 
#define UBX_MAX_LEN		256		// To fit for arbitrary tiny MCU 
#define UBX_SOP_TOK1	0xB5	// Start of packet token #1
#define UBX_SOP_TOK2	0x62	// Start of packet token #2
#define UBX_LEN_IDX		2		// Start index of length of payload field (SOP not counts)
#define UBX_LEN_N		2		// Size of length field
#define UBX_CS_N		2		// Size of checksum field


#define PROTO_BUFF_MAX		(NMEA_MAX_LEN > UBX_MAX_LEN ? NMEA_MAX_LEN : UBX_MAX_LEN)

#define PROTO_WD_MAX_TIME	(1000)	// [ms] Timeout of frame completion (just the same as for UART of u-blox N8)

typedef enum {
	epk_NMEA,
	epk_UBX
} ProtoKind_t;

typedef enum {
	ps_WaitForSOP,

	// States for NMEA framing
	ps_NMEA_CSTok,
	ps_NMEA_CS1,
	ps_NMEA_CS2,
	ps_NMEA_EOP1,
	ps_NMEA_EOP2,

	// States for UBX framing
	ps_UBX_SOP2,
	ps_UBX_Len,
	ps_UBX_Payload,
	ps_UBX_CS1,
	ps_UBX_CS2

} ProtoFSMState_t;


typedef struct UbloxProtoParser_TAG {
	ProtoFSMState_t state;
	ProtoKind_t proto;
	uint8_t nmea_cs_acc; // accumulators for checksum to distribute calculations over RX interrupts
	uint8_t ubx_cs_acc1;
	uint8_t ubx_cs_acc2;
	uint16_t ubx_len;
	uint16_t cnt;
	uint8_t* p;

	uint8_t buffer [ PROTO_BUFF_MAX ];

} UbloxProtoParser_t;


static UbloxProtoParser_t parser = { ps_WaitForSOP, epk_NMEA };

void kick_watch_dog(const size_t tm) {
	// TODO : implement restart of watch dog timer here.
}

int upper_hex2bin(const uint8_t c) {
	if (c >= '0' && c <= '9')
		return c - '0';
	else if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	else
		return -1;
}

uint16_t ubx_store(const uint8_t c) {
	*parser.p = c;
	parser.ubx_cs_acc1 += c;
	parser.ubx_cs_acc2 += parser.ubx_cs_acc1;
	parser.p++;
	parser.cnt++;
	return parser.cnt;
}

void recv_byte(uint8_t byte) {

	// Check all chars for SOPs
	if (ps_WaitForSOP == parser.state || epk_NMEA == parser.proto)
		if (NMEA_SOP_TOK == byte) {
			// Initialize NMEA framing
			parser.state = ps_NMEA_CSTok;
			parser.proto = epk_NMEA;
			parser.cnt = 0;
			parser.p = parser.buffer;
			parser.nmea_cs_acc = 0;
			kick_watch_dog(PROTO_WD_MAX_TIME);
			return;
		}
		else if (UBX_SOP_TOK1 == byte) {
			// Initialize UBX framing
			parser.state = ps_UBX_SOP2;
			parser.proto = epk_UBX;
			parser.cnt = 0;
			parser.p = parser.buffer;
			parser.ubx_cs_acc1 = 0;
			parser.ubx_cs_acc2 = 0;
			kick_watch_dog(PROTO_WD_MAX_TIME);
			return;
		}

	switch (parser.state)
	{
		// Begin of NMEA framing -----
	case ps_NMEA_CSTok:
		// Valid character ?
		if (byte > NMEA_MAX_CHR_CODE) {
			// Invalid NMEA char - restart
			parser.state = ps_WaitForSOP;
			break;
		}

		// Check sum token?
		if (NMEA_CS_TOK == byte)
		{
			// Start waiting for CS1
			parser.state = ps_NMEA_CS1;
			kick_watch_dog(PROTO_WD_MAX_TIME);
			break;
		}

		// Check overflow (strict check must check proto-specific length)
		if (parser.cnt >= sizeof(parser.buffer) / sizeof(parser.buffer[0])) {
			// Overflow - restart
			parser.state = ps_WaitForSOP;
			break;
		}

		// Save and iterate checksum calc
		*parser.p = byte;
		parser.nmea_cs_acc ^= byte;
		parser.p++;
		parser.cnt++;
		kick_watch_dog(PROTO_WD_MAX_TIME);
		break;

		// ---
	case ps_NMEA_CS1: {
		int cs_high_nibble = upper_hex2bin(byte);
		if (cs_high_nibble < 0) {
			// Invalid NMEA cecksum char - restart
			parser.state = ps_WaitForSOP;
			break;
		}
		parser.state = ps_NMEA_CS2;
		parser.nmea_cs_acc ^= (cs_high_nibble << 4) & 0xF0;
		kick_watch_dog(PROTO_WD_MAX_TIME);
	} break;

		//  ---
	case ps_NMEA_CS2: {
		int cs_low_nibble = upper_hex2bin(byte);
		if (cs_low_nibble < 0) {
			// Invalid NMEA cecksum char - restart
			parser.state = ps_WaitForSOP;
			break;
		}
		parser.state = ps_NMEA_EOP1;
		parser.nmea_cs_acc ^= cs_low_nibble;
	} break;

		//  ---
	case ps_NMEA_EOP1: {
		if (byte != NMEA_EOP_TOK1) {
			// Invalid NMEA EOP1 char - restart
			parser.state = ps_WaitForSOP;
			break;
		}
		parser.state = ps_NMEA_EOP2;
		kick_watch_dog(PROTO_WD_MAX_TIME);
	} break;

		//  ---
	case ps_NMEA_EOP2: {
		if (byte != NMEA_EOP_TOK2) {
			// Invalid NMEA EOP2 char - restart
			parser.state = ps_WaitForSOP;
			break;
		}

		// Check CS
		if (0 == parser.nmea_cs_acc) {
		
			// Call NMEA handler
			nmea_frame_received(parser.cnt, parser.buffer);
		}

		// Restart parser.
		parser.state = ps_WaitForSOP;
	} break;
	// End of NMEA framing -----

	// Begin of UBX framing  -----
	case ps_UBX_SOP2:
		if (UBX_SOP_TOK2 == byte) {
			// Initialize UBX framing
			parser.state = ps_UBX_Len;
			kick_watch_dog(PROTO_WD_MAX_TIME);
		}
		break;

	case ps_UBX_Len:
		// Save and iterate checksum calc		
		if ((UBX_LEN_IDX + UBX_LEN_N) == ubx_store(byte)) {
			
			
			// unpack LE 16 bit
			parser.ubx_len = ((uint16_t)parser.buffer[UBX_LEN_IDX + 1]) << 8 |
										parser.buffer[UBX_LEN_IDX];

			// Check lenght
			if (parser.ubx_len > sizeof(parser.buffer) / sizeof(parser.buffer[0])) {
				// Huge length - restart
				parser.state = ps_WaitForSOP;
				break;
			}

			// Length rx-ed OK - go to payload rxing
			parser.state = ps_UBX_Payload;
		}
		kick_watch_dog(PROTO_WD_MAX_TIME);
		break;

	case ps_UBX_Payload:
		// Save and iterate checksum calc		
		if (parser.ubx_len == ubx_store(byte)) {

			// Payload rx-ed - go to CS rx-ing 
			parser.state = ps_UBX_CS1;
		}
		kick_watch_dog(PROTO_WD_MAX_TIME);
		break;

	case ps_UBX_CS1:
		// Check CS1
		if (parser.ubx_cs_acc1 != byte) {

			// Bad checksum - restart 
			parser.state = ps_WaitForSOP;
			break;
		}

		// CS1 - OK - go to CS2 rx-ing 
		parser.state = ps_UBX_CS2;
		kick_watch_dog(PROTO_WD_MAX_TIME);
		break;

	case ps_UBX_CS2:
		// Check CS1
		if (parser.ubx_cs_acc2 == byte) {

			// Good checksum - call handler
			ubx_frame_received(parser.ubx_len, parser.buffer);
		}

		// Restart parser
		parser.state = ps_WaitForSOP;
		break;
	// End of UBX framing  -----

	}
}

// 15 good messages
uint8_t testStr[] = 
	"$GPGGA,213638.949,,,,,0,00,,,M,0.0,M,,0000*5F\r\n"\
	"$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n"\
	"$GPRMC,213638.949,V,,,,,,,010207,,,N*40\r\n"\
	/* invalid */"$#####################################################*AB\r\n"\
	"$GPGGA,213639.897,,,,,0,00,,,M,0.0,M,,0000*5C\r\n"\
	"$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n"\
	"$GPRMC,213639.897,V,,,,,,,010207,,,N*43\r\n"\
	/* invalid */"$$$$$$****$$$$$####*****"\
	"$GPGGA,213640.886,,,,,0,00,,,M,0.0,M,,0000*52\r\n"\
	/* invalid */"$\r\n"\
	"$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n"\
	"$GPRMC,213640.886,V,,,,,,,010207,,,N*4D\r\n"\
	"$GPGGA,213641.886,,,,,0,00,,,M,0.0,M,,0000*53\r\n"\
	/* invalid */"************************************************"\
	"$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n"\
	"$GPRMC,213641.886,V,,,,,,,010207,,,N*4C\r\n"\
	"$GPGGA,213642.897,,,,,0,00,,,M,0.0,M,,0000*50\r\n"\
	/* invalid */"$#####################################################*\r\n"\
	"$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n"\
	"$GPGSV,3,1,12,20,00,000,,10,00,000,,25,00,000,,27,00,000,*79\r\n";

int main()
{
	// test
	for (size_t i = 0; i < sizeof(testStr); i++)
	{
		recv_byte(testStr[i]);
	}
    return 0;
}

