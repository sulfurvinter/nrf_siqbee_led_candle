/* 2026-04-26 - Candle Zigbee device descriptor with Power Config cluster (Claude)
 * Extends zb_dimmable_light.h to add battery reporting via Power Config cluster.
 */

#ifndef ZB_CANDLE_H
#define ZB_CANDLE_H

#include "zb_dimmable_light.h"

/* 7 server clusters: Basic, Identify, Scenes, Groups, On/Off, Level, PowerConfig */
#define ZB_CANDLE_IN_CLUSTER_NUM  7
#define ZB_CANDLE_OUT_CLUSTER_NUM 0

#define ZB_CANDLE_REPORT_ATTR_COUNT \
	(ZB_ZCL_ON_OFF_REPORT_ATTR_COUNT + ZB_ZCL_LEVEL_CONTROL_REPORT_ATTR_COUNT)

#define ZB_CANDLE_CVC_ATTR_COUNT 1

#define ZB_DECLARE_CANDLE_CLUSTER_LIST(					\
	cluster_list_name,						\
	basic_attr_list,						\
	identify_attr_list,						\
	groups_attr_list,						\
	scenes_attr_list,						\
	on_off_attr_list,						\
	level_control_attr_list,					\
	power_config_attr_list)						\
	zb_zcl_cluster_desc_t cluster_list_name[] =			\
	{								\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_IDENTIFY,			\
			ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t), \
			(identify_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_BASIC,			\
			ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t), \
			(basic_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_SCENES,			\
			ZB_ZCL_ARRAY_SIZE(scenes_attr_list, zb_zcl_attr_t), \
			(scenes_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_GROUPS,			\
			ZB_ZCL_ARRAY_SIZE(groups_attr_list, zb_zcl_attr_t), \
			(groups_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_ON_OFF,			\
			ZB_ZCL_ARRAY_SIZE(on_off_attr_list, zb_zcl_attr_t), \
			(on_off_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,		\
			ZB_ZCL_ARRAY_SIZE(level_control_attr_list, zb_zcl_attr_t), \
			(level_control_attr_list),			\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_POWER_CONFIG,			\
			ZB_ZCL_ARRAY_SIZE(power_config_attr_list, zb_zcl_attr_t), \
			(power_config_attr_list),			\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		)							\
	}

/* Pass cluster counts as parameters (like zb_dimmable_light.h) so token pasting
 * in ZB_DECLARE_SIMPLE_DESC/ZB_AF_SIMPLE_DESC_TYPE receives literal integers. */
#define ZB_ZCL_DECLARE_CANDLE_SIMPLE_DESC(ep_name, ep_id, in_clust_num, out_clust_num) \
	ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num);		\
	ZB_AF_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num)		\
	simple_desc_##ep_name = {					\
		ep_id,							\
		ZB_AF_HA_PROFILE_ID,					\
		ZB_DIMMABLE_LIGHT_DEVICE_ID,				\
		ZB_DEVICE_VER_DIMMABLE_LIGHT,				\
		0,							\
		in_clust_num,						\
		out_clust_num,						\
		{							\
			ZB_ZCL_CLUSTER_ID_BASIC,			\
			ZB_ZCL_CLUSTER_ID_IDENTIFY,			\
			ZB_ZCL_CLUSTER_ID_SCENES,			\
			ZB_ZCL_CLUSTER_ID_GROUPS,			\
			ZB_ZCL_CLUSTER_ID_ON_OFF,			\
			ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,		\
			ZB_ZCL_CLUSTER_ID_POWER_CONFIG,			\
		}							\
	}

#define ZB_DECLARE_CANDLE_EP(ep_name, ep_id, cluster_list)		\
	ZB_ZCL_DECLARE_CANDLE_SIMPLE_DESC(ep_name, ep_id,		\
		ZB_CANDLE_IN_CLUSTER_NUM,				\
		ZB_CANDLE_OUT_CLUSTER_NUM);				\
	ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##ep_name,	\
		ZB_CANDLE_REPORT_ATTR_COUNT);				\
	ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_info##ep_name,	\
		ZB_CANDLE_CVC_ATTR_COUNT);				\
	ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id,			\
		ZB_AF_HA_PROFILE_ID, 0, NULL,				\
		ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t),	\
		cluster_list,						\
		(zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,	\
		ZB_CANDLE_REPORT_ATTR_COUNT,				\
		reporting_info##ep_name,				\
		ZB_CANDLE_CVC_ATTR_COUNT,				\
		cvc_alarm_info##ep_name)

#endif /* ZB_CANDLE_H */
