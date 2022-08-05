/** CAN message encoder/decoder: automatically generated - do not edit
  * Generated by dbcc: See https://github.com/howerj/dbcc */
#ifndef UMRR_H
#define UMRR_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" { 
#endif

#ifndef PREPACK
#define PREPACK
#endif

#ifndef POSTPACK
#define POSTPACK
#endif

#ifndef DBCC_TIME_STAMP
#define DBCC_TIME_STAMP
typedef uint32_t dbcc_time_stamp_t; /* Time stamp for message; you decide on units */
#endif

#ifndef DBCC_STATUS_ENUM
#define DBCC_STATUS_ENUM
typedef enum {
	DBCC_SIG_STAT_UNINITIALIZED_E = 0, /* Message never sent/received */
	DBCC_SIG_STAT_OK_E            = 1, /* Message ok */
	DBCC_SIG_STAT_ERROR_E         = 2, /* Encode/Decode/Timestamp/Any error */
} dbcc_signal_status_e;
#endif

typedef PREPACK struct {
	uint32_t CYCLE_COUNTER; /* scaling 1.0, offset 0.0, units none  */
	uint16_t CYCLE_DURATION; /* scaling 0.1, offset 0.0, units none  */
	uint8_t NUMBER_OF_OBJECTS; /* scaling 1.0, offset 0.0, units none  */
} POSTPACK can_0x400_RAW_HEADER_t;

typedef PREPACK struct {
	uint16_t RANGE; /* scaling 0.0, offset 0.0, units m  */
	uint16_t SPEED_RADIAL; /* scaling 0.0, offset -119.7, units m/s  */
	uint16_t AZIMUTH; /* scaling 0.2, offset -81.8, units degree  */
	uint16_t ELEVATION; /* scaling 0.0, offset -20.4, units degree  */
	uint8_t RCS; /* scaling 0.2, offset -15.0, units dBm�  */
	uint8_t MODE_SIGNAL; /* scaling 1.0, offset 0.0, units none  */
} POSTPACK can_0x401_TARGET_DATA_1_t;

typedef PREPACK struct {
    dbcc_time_stamp_t can_0x400_RAW_HEADER_time_stamp_rx;

	unsigned can_0x400_RAW_HEADER_status : 2;
	unsigned can_0x400_RAW_HEADER_tx : 1;
	unsigned can_0x400_RAW_HEADER_rx : 1;
	unsigned can_0x401_TARGET_DATA_1_status : 2;
	unsigned can_0x401_TARGET_DATA_1_tx : 1;
	unsigned can_0x401_TARGET_DATA_1_rx : 1;

	can_0x400_RAW_HEADER_t can_0x400_RAW_HEADER;

} POSTPACK can_obj_umrr_header_h_t;

typedef PREPACK struct {
    dbcc_time_stamp_t can_0x401_TARGET_DATA_1_time_stamp_rx;

    unsigned can_0x400_RAW_HEADER_status : 2;
    unsigned can_0x400_RAW_HEADER_tx : 1;
    unsigned can_0x400_RAW_HEADER_rx : 1;
    unsigned can_0x401_TARGET_DATA_1_status : 2;
    unsigned can_0x401_TARGET_DATA_1_tx : 1;
    unsigned can_0x401_TARGET_DATA_1_rx : 1;

    can_0x401_TARGET_DATA_1_t can_0x401_TARGET_DATA_1;

} POSTPACK can_obj_umrr_target_h_t;

int unpack_message_header(can_obj_umrr_header_h_t *o, const unsigned long id, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp);
int pack_message_header(can_obj_umrr_header_h_t *o, const unsigned long id, uint64_t *data);
int print_message_header(const can_obj_umrr_header_h_t *o, const unsigned long id, FILE *output);

int unpack_message_target(can_obj_umrr_target_h_t *o, const unsigned long id, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp);
int pack_message_target(can_obj_umrr_target_h_t *o, const unsigned long id, uint64_t *data);
int print_message_target(const can_obj_umrr_target_h_t *o, const unsigned long id, FILE *output);

int decode_can_0x400_CYCLE_COUNTER(const can_obj_umrr_header_h_t *o, uint32_t *out);
int encode_can_0x400_CYCLE_COUNTER(can_obj_umrr_header_h_t *o, uint32_t in);
int decode_can_0x400_CYCLE_DURATION(const can_obj_umrr_header_h_t *o, double *out);
int encode_can_0x400_CYCLE_DURATION(can_obj_umrr_header_h_t *o, double in);
int decode_can_0x400_NUMBER_OF_OBJECTS(const can_obj_umrr_header_h_t *o, uint8_t *out);
int encode_can_0x400_NUMBER_OF_OBJECTS(can_obj_umrr_header_h_t *o, uint8_t in);


int decode_can_0x401_RANGE(const can_obj_umrr_target_h_t *o, double *out);
int encode_can_0x401_RANGE(can_obj_umrr_target_h_t *o, double in);
int decode_can_0x401_SPEED_RADIAL(const can_obj_umrr_target_h_t *o, double *out);
int encode_can_0x401_SPEED_RADIAL(can_obj_umrr_target_h_t *o, double in);
int decode_can_0x401_AZIMUTH(const can_obj_umrr_target_h_t *o, double *out);
int encode_can_0x401_AZIMUTH(can_obj_umrr_target_h_t *o, double in);
int decode_can_0x401_ELEVATION(const can_obj_umrr_target_h_t *o, double *out);
int encode_can_0x401_ELEVATION(can_obj_umrr_target_h_t *o, double in);
int decode_can_0x401_RCS(const can_obj_umrr_target_h_t *o, double *out);
int encode_can_0x401_RCS(can_obj_umrr_target_h_t *o, double in);
int decode_can_0x401_MODE_SIGNAL(const can_obj_umrr_target_h_t *o, uint8_t *out);
int encode_can_0x401_MODE_SIGNAL(can_obj_umrr_target_h_t *o, uint8_t in);

uint64_t u64_from_can_msg(const uint8_t m[8]);
void u64_to_can_msg(const uint64_t u, uint8_t m[8]);

#ifdef __cplusplus
} 
#endif

#endif