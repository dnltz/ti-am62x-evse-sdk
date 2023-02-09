/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.4 */

#ifndef PB_LOW_TO_HIGH_PB_H_INCLUDED
#define PB_LOW_TO_HIGH_PB_H_INCLUDED
#include <nanopb/pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _IEC61851Event {
    IEC61851Event_CAR_PLUGGED_IN = 0,
    IEC61851Event_CAR_REQUESTED_POWER = 1,
    IEC61851Event_POWER_ON = 2,
    IEC61851Event_POWER_OFF = 3,
    IEC61851Event_CAR_REQUESTED_STOP_POWER = 4,
    IEC61851Event_CAR_UNPLUGGED = 5,
    IEC61851Event_ERROR_E = 6,
    IEC61851Event_ERROR_DF = 7,
    IEC61851Event_ERROR_RELAIS = 8,
    IEC61851Event_ERROR_RCD = 9,
    IEC61851Event_ERROR_VENTILATION_NOT_AVAILABLE = 10,
    IEC61851Event_ERROR_OVER_CURRENT = 11,
    IEC61851Event_EF_TO_BCD = 12,
    IEC61851Event_BCD_TO_EF = 13,
    IEC61851Event_PERMANENT_FAULT = 14,
    IEC61851Event_EVSE_REPLUG_STARTED = 15,
    IEC61851Event_EVSE_REPLUG_FINISHED = 16
} IEC61851Event;

/* Struct definitions */
typedef struct _McuHeartbeat {
    char dummy_field;
} McuHeartbeat;

typedef struct _LowToHigh {
    pb_size_t which_message;
    union {
        IEC61851Event event;
        McuHeartbeat heartbeat;
    } message;
} LowToHigh;


/* Helper constants for enums */
#define _IEC61851Event_MIN IEC61851Event_CAR_PLUGGED_IN
#define _IEC61851Event_MAX IEC61851Event_EVSE_REPLUG_FINISHED
#define _IEC61851Event_ARRAYSIZE ((IEC61851Event)(IEC61851Event_EVSE_REPLUG_FINISHED+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define McuHeartbeat_init_default                {0}
#define LowToHigh_init_default                   {0, {_IEC61851Event_MIN}}
#define McuHeartbeat_init_zero                   {0}
#define LowToHigh_init_zero                      {0, {_IEC61851Event_MIN}}

/* Field tags (for use in manual encoding/decoding) */
#define LowToHigh_event_tag                      1
#define LowToHigh_heartbeat_tag                  2

/* Struct field encoding specification for nanopb */
#define McuHeartbeat_FIELDLIST(X, a) \

#define McuHeartbeat_CALLBACK NULL
#define McuHeartbeat_DEFAULT NULL

#define LowToHigh_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    UENUM,    (message,event,message.event),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message,heartbeat,message.heartbeat),   2)
#define LowToHigh_CALLBACK NULL
#define LowToHigh_DEFAULT NULL
#define LowToHigh_message_heartbeat_MSGTYPE McuHeartbeat

extern const pb_msgdesc_t McuHeartbeat_msg;
extern const pb_msgdesc_t LowToHigh_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define McuHeartbeat_fields &McuHeartbeat_msg
#define LowToHigh_fields &LowToHigh_msg

/* Maximum encoded size of messages (where known) */
#define McuHeartbeat_size                        0
#define LowToHigh_size                           2

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
