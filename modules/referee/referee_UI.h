/**
* @file referee_UI.h
* @brief 声明裁判系统客户端的图形、文字绘制和发送接口。
* 坐标和线宽使用像素值；图层为0～9，操作和颜色使用 referee_protocol.h 中的枚举。
* 图形名最多读取3个字符；绘图接口会改写传入结构体，不检查空指针或数值范围。
*/
#ifndef REFEREE_UI_H
#define REFEREE_UI_H

#include "stdarg.h"
#include "stdint.h"
#include "referee_protocol.h"
#include "rm_referee.h"

#pragma pack(1) // UI 帧结构按协议要求逐字节排列。

/* 此处的定义只与UI绘制有关 */
typedef struct
{
   xFrameHeader FrameHeader;
   uint16_t CmdID;
   ext_student_interactive_header_data_t datahead;
   uint8_t Delete_Operate; // 删除操作
   uint8_t Layer;
   uint16_t frametail;
} UI_delete_t;

typedef struct
{
   xFrameHeader FrameHeader;
   uint16_t CmdID;
   ext_student_interactive_header_data_t datahead;
   uint16_t frametail;
} UI_GraphReFresh_t;

typedef struct
{
   xFrameHeader FrameHeader;
   uint16_t CmdID;
   ext_student_interactive_header_data_t datahead;
   String_Data_t String_Data;
   uint16_t frametail;
} UI_CharReFresh_t; // 打印字符串数据

#pragma pack()

/**
* @brief 删除指定图层或全部 UI 图形。
* @param[in] _id 已确定机器人编号的裁判 ID。
* @param[in] Del_Operate 使用 UI_Data_Del_Layer 或 UI_Data_Del_ALL。
* @param[in] Del_Layer 要删除的图层，范围0～9；删除全部时忽略此值。
* @note 不检查空指针或参数范围；CRC 计算失败直接返回，不发送，也不增加序号。
*/
void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer);

/** @brief 把直线的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param graph 接收图形数据的可写对象。
* @param graphname 图形名称；最多读取3个字符。
* @param Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param Graph_Layer 图层，范围0～9。
* @param Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param Graph_Width 线宽，像素。
* @param Start_x 直线起点 x 坐标，像素。
* @param Start_y 直线起点 y 坐标，像素。
* @param End_x 直线终点 x 坐标，像素。
* @param End_y 直线终点 y 坐标，像素。
* @note 函数不发送数据，也不检查指针和数值范围。
*/
void UILineDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

/** @brief 把矩形的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param graph 接收图形数据的可写对象。
* @param graphname 图形名称；最多读取3个字符。
* @param Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param Graph_Layer 图层，范围0～9。
* @param Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param Graph_Width 边线宽度，像素。
* @param Start_x 矩形一个顶点的 x 坐标，像素。
* @param Start_y 矩形一个顶点的 y 坐标，像素。
* @param End_x 与起点相对的顶点 x 坐标，像素。
* @param End_y 与起点相对的顶点 y 坐标，像素。
* @note 函数不发送数据，也不检查指针和数值范围。
*/
void UIRectangleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

/** @brief 把圆的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param graph 接收图形数据的可写对象。
* @param graphname 图形名称；最多读取3个字符。
* @param Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param Graph_Layer 图层，范围0～9。
* @param Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param Graph_Width 圆边线宽度，像素。
* @param Start_x 圆心 x 坐标，像素。
* @param Start_y 圆心 y 坐标，像素。
* @param Graph_Radius 半径，像素。
* @note 函数不发送数据，也不检查指针和数值范围。
*/
void UICircleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                  uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);

/** @brief 把椭圆的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param graph 接收图形数据的可写对象。
* @param graphname 图形名称；最多读取3个字符。
* @param Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param Graph_Layer 图层，范围0～9。
* @param Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param Graph_Width 边线宽度，像素。
* @param Start_x 椭圆中心 x 坐标，像素。
* @param Start_y 椭圆中心 y 坐标，像素。
* @param end_x x 轴半径，像素。
* @param end_y y 轴半径，像素。
* @note 函数不发送数据，也不检查指针和数值范围。
*/
void UIOvalDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y);

/** @brief 把圆弧的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param graph 接收图形数据的可写对象。
* @param graphname 图形名称；最多读取3个字符。
* @param Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param Graph_Layer 图层，范围0～9。
* @param Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param Graph_StartAngle 起始角度，按协议角度值传入。
* @param Graph_EndAngle 结束角度，按协议角度值传入。
* @param Graph_Width 线宽，像素。
* @param Start_x 圆弧中心 x 坐标，像素。
* @param Start_y 圆弧中心 y 坐标，像素。
* @param end_x x 轴半径，像素。
* @param end_y y 轴半径，像素。
* @note 函数不发送数据，也不检查指针和数值范围。
*/
void UIArcDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
               uint32_t end_x, uint32_t end_y);

/** @brief 把浮点显示值的参数和编码结果写入图形结构。
* @param graph 接收图形数据的可写对象。
* @param graphname 图形名称；最多读取3个字符。
* @param Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param Graph_Layer 图层，范围0～9。
* @param Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param Graph_Size 字号参数，按协议规定的数值传入。
* @param Graph_Digit 小数位数。
* @param Graph_Width 字符线宽，像素。
* @param Start_x 左上角 x 坐标，像素。
* @param Start_y 左上角 y 坐标，像素。
* @param Graph_Float 调用方按协议把显示数值放大1000倍后传入；函数拆分其位段。
* @note 函数只准备图形数据，不发送，也不检查指针和数值范围。
*/
void UIFloatDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                 uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float);

/** @brief 把整数显示值的参数和编码结果写入图形结构。
* @param graph 接收图形数据的可写对象。
* @param graphname 图形名称；最多读取3个字符。
* @param Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param Graph_Layer 图层，范围0～9。
* @param Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param Graph_Size 字号参数，按协议规定的数值传入。
* @param Graph_Width 字符线宽，像素。
* @param Start_x 左上角 x 坐标，像素。
* @param Start_y 左上角 y 坐标，像素。
* @param Graph_Integer 要显示并按协议拆分到图形字段的整数。
* @note 函数只准备图形数据，不发送，也不检查指针和数值范围。
*/
void UIIntDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer);

/** @brief 格式化并准备一条文字图形。
* @param graph 接收文字和图形参数的可写对象；内部文字缓冲区为30字节。
* @param graphname 图形名称；最多读取3个字符。
* @param Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param Graph_Layer 图层，范围0～9。
* @param Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param Graph_Size 字号参数，按协议规定的数值传入。
* @param Graph_Width 字符线宽，像素。
* @param Start_x 左上角 x 坐标，像素。
* @param Start_y 左上角 y 坐标，像素。
* @param fmt printf 格式字符串，后续参数提供对应的格式化内容。
* @note 格式化后的内容连同结束标记必须能放入30字节；本函数不检查指针或长度。
*/
void UICharDraw(String_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...);

/**
* @brief 一次发送1、2、5或7个图形。
* @param _id 已确定发送对象编号的裁判 ID。
* @param cnt 图形个数，只支持1、2、5、7。
* @param ... 按值传入 cnt 个 Graph_Data_t 图形对象，不是指针。
* @note 调用方须保证参数个数与 cnt 一致；本函数不检查指针或不支持的数量。
* CRC 计算失败直接返回，不提交发送缓冲区。
*/
void UIGraphRefresh(referee_id_t *_id, int cnt, ...);

/** @brief 发送一条已准备好的文字图形。
* @param _id 已确定发送对象编号的裁判 ID。
* @param string_Data 已由 UICharDraw 填写的文字和图形数据。
* @note 不检查空指针；CRC 计算失败直接返回，不发送，也不增加序号。
*/
void UICharRefresh(referee_id_t *_id, String_Data_t string_Data);

#endif
