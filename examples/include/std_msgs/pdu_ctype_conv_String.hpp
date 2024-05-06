#ifndef _PDU_CTYPE_CONV_HAKO_std_msgs_String_HPP_
#define _PDU_CTYPE_CONV_HAKO_std_msgs_String_HPP_

#include "pdu_primitive_ctypes.h"
#include "ros_primitive_types.hpp"
#include "pdu_primitive_ctypes_conv.hpp"
/*
 * Dependent pdu data
 */
#include "std_msgs/pdu_ctype_String.h"
/*
 * Dependent ros data
 */
#include "std_msgs/msg/string.hpp"

/*
 * Dependent Convertors
 */

/***************************
 *
 * PDU ==> ROS2
 *
 ***************************/
static inline int hako_convert_pdu2ros_String(Hako_String &src,  std_msgs::msg::String &dst)
{
    //string convertor
    dst.data = (const char*)src.data;
    return 0;
}

template<int _src_len, int _dst_len>
int hako_convert_pdu2ros_array_String(Hako_String src[], std::array<std_msgs::msg::String, _dst_len> &dst)
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_pdu2ros_String(src[i], dst[i]);
    }
    return ret;
}
template<int _src_len, int _dst_len>
int hako_convert_pdu2ros_array_String(Hako_String src[], std::vector<std_msgs::msg::String> &dst)
{
    dst.resize(_src_len);
    for (int i = 0; i < _src_len; i++) {
        (void)hako_convert_pdu2ros_String(src[i], dst[i]);
    }
    return 0;
}

/***************************
 *
 * ROS2 ==> PDU
 *
 ***************************/
static inline int hako_convert_ros2pdu_String(std_msgs::msg::String &src, Hako_String &dst)
{
    //string convertor
    (void)hako_convert_ros2pdu_array(
        src.data, src.data.length(),
        dst.data, M_ARRAY_SIZE(Hako_String, char, data));
    return 0;
}

template<int _src_len, int _dst_len>
int hako_convert_ros2pdu_array_String(std::array<std_msgs::msg::String, _src_len> &src, Hako_String dst[])
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_ros2pdu_String(src[i], dst[i]);
    }
    return ret;
}
template<int _src_len, int _dst_len>
int hako_convert_ros2pdu_array_String(std::vector<std_msgs::msg::String> &src, Hako_String dst[])
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_ros2pdu_String(src[i], dst[i]);
    }
    return ret;
}

#endif /* _PDU_CTYPE_CONV_HAKO_std_msgs_String_HPP_ */
