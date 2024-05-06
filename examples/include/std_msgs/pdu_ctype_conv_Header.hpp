#ifndef _PDU_CTYPE_CONV_HAKO_std_msgs_Header_HPP_
#define _PDU_CTYPE_CONV_HAKO_std_msgs_Header_HPP_

#include "pdu_primitive_ctypes.h"
#include "ros_primitive_types.hpp"
#include "pdu_primitive_ctypes_conv.hpp"
/*
 * Dependent pdu data
 */
#include "std_msgs/pdu_ctype_Header.h"
/*
 * Dependent ros data
 */
#include "std_msgs/msg/header.hpp"

/*
 * Dependent Convertors
 */
#include "builtin_interfaces/pdu_ctype_conv_Time.hpp"

/***************************
 *
 * PDU ==> ROS2
 *
 ***************************/
static inline int hako_convert_pdu2ros_Header(Hako_Header &src,  std_msgs::msg::Header &dst)
{
    //struct convert
    hako_convert_pdu2ros_Time(src.stamp, dst.stamp);
    //string convertor
    dst.frame_id = (const char*)src.frame_id;
    return 0;
}

template<int _src_len, int _dst_len>
int hako_convert_pdu2ros_array_Header(Hako_Header src[], std::array<std_msgs::msg::Header, _dst_len> &dst)
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_pdu2ros_Header(src[i], dst[i]);
    }
    return ret;
}
template<int _src_len, int _dst_len>
int hako_convert_pdu2ros_array_Header(Hako_Header src[], std::vector<std_msgs::msg::Header> &dst)
{
    dst.resize(_src_len);
    for (int i = 0; i < _src_len; i++) {
        (void)hako_convert_pdu2ros_Header(src[i], dst[i]);
    }
    return 0;
}

/***************************
 *
 * ROS2 ==> PDU
 *
 ***************************/
static inline int hako_convert_ros2pdu_Header(std_msgs::msg::Header &src, Hako_Header &dst)
{
    //struct convert
    hako_convert_ros2pdu_Time(src.stamp, dst.stamp);
    //string convertor
    (void)hako_convert_ros2pdu_array(
        src.frame_id, src.frame_id.length(),
        dst.frame_id, M_ARRAY_SIZE(Hako_Header, char, frame_id));
    return 0;
}

template<int _src_len, int _dst_len>
int hako_convert_ros2pdu_array_Header(std::array<std_msgs::msg::Header, _src_len> &src, Hako_Header dst[])
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_ros2pdu_Header(src[i], dst[i]);
    }
    return ret;
}
template<int _src_len, int _dst_len>
int hako_convert_ros2pdu_array_Header(std::vector<std_msgs::msg::Header> &src, Hako_Header dst[])
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_ros2pdu_Header(src[i], dst[i]);
    }
    return ret;
}

#endif /* _PDU_CTYPE_CONV_HAKO_std_msgs_Header_HPP_ */
