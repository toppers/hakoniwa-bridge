#ifndef _pdu_ctype_std_msgs_Header_H_
#define _pdu_ctype_std_msgs_Header_H_

#include "pdu_primitive_ctypes.h"
#include "builtin_interfaces/pdu_ctype_Time.h"

typedef struct {
        Hako_Time    stamp;
        char frame_id[HAKO_STRING_SIZE];
} Hako_Header;

#endif /* _pdu_ctype_std_msgs_Header_H_ */
