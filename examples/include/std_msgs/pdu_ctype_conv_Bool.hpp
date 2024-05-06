#ifndef _PDU_CTYPE_CONV_HAKO_std_msgs_Bool_HPP_
#define _PDU_CTYPE_CONV_HAKO_std_msgs_Bool_HPP_

#include "pdu_primitive_ctypes.h"
#include "ros_primitive_types.hpp"
#include "pdu_primitive_ctypes_conv.hpp"
/*
 * Dependent pdu data
 */
#include "std_msgs/pdu_ctype_Bool.h"
/*
 * Dependent ros data
 */
#include "std_msgs/msg/bool.hpp"

/*
 * Dependent Convertors
 */

/***************************
 *
 * PDU ==> ROS2
 *
 ***************************/
static inline int hako_convert_pdu2ros_Bool(Hako_Bool &src,  std_msgs::msg::Bool &dst)
{
    //primitive convert
    hako_convert_pdu2ros(src.data, dst.data);
    return 0;
}

template<int _src_len, int _dst_len>
int hako_convert_pdu2ros_array_Bool(Hako_Bool src[], std::array<std_msgs::msg::Bool, _dst_len> &dst)
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_pdu2ros_Bool(src[i], dst[i]);
    }
    return ret;
}
template<int _src_len, int _dst_len>
int hako_convert_pdu2ros_array_Bool(Hako_Bool src[], std::vector<std_msgs::msg::Bool> &dst)
{
    dst.resize(_src_len);
    for (int i = 0; i < _src_len; i++) {
        (void)hako_convert_pdu2ros_Bool(src[i], dst[i]);
    }
    return 0;
}

/***************************
 *
 * ROS2 ==> PDU
 *
 ***************************/
static inline int hako_convert_ros2pdu_Bool(std_msgs::msg::Bool &src, Hako_Bool &dst)
{
    //primitive convert
    hako_convert_ros2pdu(src.data, dst.data);
    return 0;
}

template<int _src_len, int _dst_len>
int hako_convert_ros2pdu_array_Bool(std::array<std_msgs::msg::Bool, _src_len> &src, Hako_Bool dst[])
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_ros2pdu_Bool(src[i], dst[i]);
    }
    return ret;
}
template<int _src_len, int _dst_len>
int hako_convert_ros2pdu_array_Bool(std::vector<std_msgs::msg::Bool> &src, Hako_Bool dst[])
{
    int ret = 0;
    int len = _dst_len;
    if (_dst_len > _src_len) {
        len = _src_len;
        ret = -1;
    }
    for (int i = 0; i < len; i++) {
        (void)hako_convert_ros2pdu_Bool(src[i], dst[i]);
    }
    return ret;
}

#endif /* _PDU_CTYPE_CONV_HAKO_std_msgs_Bool_HPP_ */
