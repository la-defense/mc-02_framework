/**
* @file referee_UI.h
* @brief 声明裁判系统客户端的图形、文字绘制和发送接口。
* 坐标和线宽使用像素值；图层为0～9，操作和颜色使用 referee_protocol.h 中的枚举。
* 绘图先生成候选对象，失败保留原对象；发送须在RefereeInit成功后由裁判任务串行调用。
* 坐标x为0～1920、y为0～1080；线宽0～1023、圆半径0～1023、椭圆半轴0～2047像素。
* 操作为0～3、图层0～9、颜色0～8、角度0～360度、字号0～511，两个边界均允许。
* 不检查整幅图形是否落在屏幕内。名称为最多3字节的可读数组，遇到NUL停止；保留旧逆序名称。
* 所有非空指针仍须指向实际有效对象；构图无需初始化。输出对象在调用结束后归调用者使用。
* 选手端ID仅0x0101～0x0106、0x0165～0x016A；普通发送者须为客户端ID减0x100。
* 雷达发送者9／109可向任意同队选手端发送；身份不符返回范围错误，不提交帧。
*/
#ifndef REFEREE_UI_H
#define REFEREE_UI_H

#include <stddef.h>
#include "stdint.h"
#include "referee_protocol.h"
#include "rm_referee.h"

/** @brief UI发送身份；由新鲜的0x0201裁判数据重新计算，不能来自可变接收对象。 */
typedef struct {
    uint8_t Robot_Color;
    uint16_t Robot_ID;
    uint16_t Cilent_ID;
    uint16_t Receiver_Robot_ID;
} referee_id_t;

/** @brief UI发送帧序号；仅由串行的裁判UI发送路径在本机接受后递增。 */
extern uint8_t UI_Seq;

/** @brief OK表示本地构图成功，SUBMITTED表示本机接受发送；两者都不证明客户端已显示。 */
typedef enum {
    UI_OK = 0, UI_SUBMITTED = 1,
    UI_ERROR_ARGUMENT = -1, UI_ERROR_RANGE = -2, UI_ERROR_LENGTH = -3,
    UI_ERROR_FORMAT = -4, UI_ERROR_CRC = -5, UI_NOT_INITIALIZED = -6,
    UI_SEND_BUSY = -7, UI_SEND_ERROR = -8, UI_SEND_TIMEOUT = -9
} UIStatus_t;

/**
* @brief 删除指定图层或全部 UI 图形。
* @param[in] _id 已确定机器人编号的裁判 ID。
* @param[in] Del_Operate 0为空操作、1删除图层、2删除全部。
* @param[in] Del_Layer 图层0～9；即使删除全部也须在范围内。
* @return 本机接受返回UI_SUBMITTED；否则返回参数、范围、CRC或发送错误，失败不消耗序号。
* @note 任务、身份和缓冲生命周期条件同UIGraphRefresh；不会确认客户端显示。
*/
UIStatus_t UIDelete(const referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer);

/** @brief 把直线的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param[out] graph 接收图形数据的可写对象。
* @param[in] graphname 图形名称；最多读取3个字符。
* @param[in] Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param[in] Graph_Layer 图层，范围0～9。
* @param[in] Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param[in] Graph_Width 线宽，像素。
* @param[in] Start_x 直线起点 x 坐标，像素。
* @param[in] Start_y 直线起点 y 坐标，像素。
* @param[in] End_x 直线终点 x 坐标，像素。
* @param[in] End_y 直线终点 y 坐标，像素。
* @return UI_OK表示构图成功；参数或范围错误返回具体状态。
* @note 构图失败保持原对象；不进行发送。
*/
UIStatus_t UILineDraw(Graph_Data_t *graph, const char *graphname, uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

/** @brief 把矩形的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param[out] graph 接收图形数据的可写对象。
* @param[in] graphname 图形名称；最多读取3个字符。
* @param[in] Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param[in] Graph_Layer 图层，范围0～9。
* @param[in] Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param[in] Graph_Width 边线宽度，像素。
* @param[in] Start_x 矩形一个顶点的 x 坐标，像素。
* @param[in] Start_y 矩形一个顶点的 y 坐标，像素。
* @param[in] End_x 与起点相对的顶点 x 坐标，像素。
* @param[in] End_y 与起点相对的顶点 y 坐标，像素。
* @return UI_OK表示构图成功；参数或范围错误返回具体状态。
* @note 构图失败保持原对象；不进行发送。
*/
UIStatus_t UIRectangleDraw(Graph_Data_t *graph, const char *graphname, uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

/** @brief 把圆的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param[out] graph 接收图形数据的可写对象。
* @param[in] graphname 图形名称；最多读取3个字符。
* @param[in] Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param[in] Graph_Layer 图层，范围0～9。
* @param[in] Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param[in] Graph_Width 圆边线宽度，像素。
* @param[in] Start_x 圆心 x 坐标，像素。
* @param[in] Start_y 圆心 y 坐标，像素。
* @param[in] Graph_Radius 半径，像素。
* @return UI_OK表示构图成功；参数或范围错误返回具体状态。
* @note 构图失败保持原对象；不进行发送。
*/
UIStatus_t UICircleDraw(Graph_Data_t *graph, const char *graphname, uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                  uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);

/** @brief 把椭圆的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param[out] graph 接收图形数据的可写对象。
* @param[in] graphname 图形名称；最多读取3个字符。
* @param[in] Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param[in] Graph_Layer 图层，范围0～9。
* @param[in] Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param[in] Graph_Width 边线宽度，像素。
* @param[in] Start_x 椭圆中心 x 坐标，像素。
* @param[in] Start_y 椭圆中心 y 坐标，像素。
* @param[in] end_x x 轴半径，像素。
* @param[in] end_y y 轴半径，像素。
* @return UI_OK表示构图成功；参数或范围错误返回具体状态。
* @note 构图失败保持原对象；不进行发送。
*/
UIStatus_t UIOvalDraw(Graph_Data_t *graph, const char *graphname, uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y);

/** @brief 把圆弧的参数写入图形结构，随后用 UIGraphRefresh 发送。
* @param[out] graph 接收图形数据的可写对象。
* @param[in] graphname 图形名称；最多读取3个字符。
* @param[in] Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param[in] Graph_Layer 图层，范围0～9。
* @param[in] Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param[in] Graph_StartAngle 起始角度，按协议角度值传入。
* @param[in] Graph_EndAngle 结束角度，按协议角度值传入。
* @param[in] Graph_Width 线宽，像素。
* @param[in] Start_x 圆弧中心 x 坐标，像素。
* @param[in] Start_y 圆弧中心 y 坐标，像素。
* @param[in] end_x x 轴半径，像素。
* @param[in] end_y y 轴半径，像素。
* @return UI_OK表示构图成功；参数或范围错误返回具体状态。
* @note 构图失败保持原对象；不进行发送。
*/
UIStatus_t UIArcDraw(Graph_Data_t *graph, const char *graphname, uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
               uint32_t end_x, uint32_t end_y);

/** @brief 把浮点显示值的参数和编码结果写入图形结构。
* @param[out] graph 接收图形数据的可写对象。
* @param[in] graphname 图形名称；最多读取3个字符。
* @param[in] Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param[in] Graph_Layer 图层，范围0～9。
* @param[in] Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param[in] Graph_Size 字号参数，按协议规定的数值传入。
* @param[in] Graph_Digit 兼容旧调用的0～511参数；2026协议中该字段无作用，固定发送0。
* @param[in] Graph_Width 字符线宽，像素。
* @param[in] Start_x 左上角 x 坐标，像素。
* @param[in] Start_y 左上角 y 坐标，像素。
* @param[in] Graph_Float 调用方按协议把显示数值放大1000倍后传入；函数拆分其位段。
* @return UI_OK表示构图成功；参数或范围错误返回具体状态。
* @note 构图失败保持原对象；不进行发送。
*/
UIStatus_t UIFloatDraw(Graph_Data_t *graph, const char *graphname, uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                 uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float);

/** @brief 把整数显示值的参数和编码结果写入图形结构。
* @param[out] graph 接收图形数据的可写对象。
* @param[in] graphname 图形名称；最多读取3个字符。
* @param[in] Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param[in] Graph_Layer 图层，范围0～9。
* @param[in] Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param[in] Graph_Size 字号参数，按协议规定的数值传入。
* @param[in] Graph_Width 字符线宽，像素。
* @param[in] Start_x 左上角 x 坐标，像素。
* @param[in] Start_y 左上角 y 坐标，像素。
* @param[in] Graph_Integer 要显示并按协议拆分到图形字段的整数。
* @return UI_OK表示构图成功；参数或范围错误返回具体状态。
* @note 构图失败保持原对象；不进行发送。
*/
UIStatus_t UIIntDraw(Graph_Data_t *graph, const char *graphname, uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer);

/** @brief 格式化并准备一条文字图形。
* @param[out] graph 接收文字和图形参数的可写对象；内部文字缓冲区为30字节。
* @param[in] graphname 图形名称；最多读取3个字符。
* @param[in] Graph_Operate 操作类型，使用 UI_Graph_ADD、UI_Graph_Change 或 UI_Graph_Del。
* @param[in] Graph_Layer 图层，范围0～9。
* @param[in] Graph_Color 颜色枚举 UI_Graph_Color_e 中的值。
* @param[in] Graph_Size 字号参数，按协议规定的数值传入。
* @param[in] Graph_Width 字符线宽，像素。
* @param[in] Start_x 左上角 x 坐标，像素。
* @param[in] Start_y 左上角 y 坐标，像素。
* @param[in] fmt 非空的合法printf格式字符串，参数类型须匹配且所引用对象可读至调用结束。
* @note 使用31字节临时区，协议文字长度为0～30字节；超长整条拒绝，空文字允许。
* 文字按字节计数，不提供编码转换；恰好30字节时不发送NUL，剩余未用文字字节清零。
* @return UI_OK表示构图成功；否则返回参数、范围、长度或格式化错误，保持原对象。
* @note 构图失败保持原对象；不进行发送。
*/
UIStatus_t UICharDraw(String_Data_t *graph, const char *graphname, uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, const char *fmt, ...);

/** @brief 把1、2、5或7个检查通过的图形作为一帧提交。
 * @param[in] _id 非空，机器人及选手端ID合法且相互对应，可读至返回。
 * @param[in] graphs 非空图形数组，可读至返回；不接受字符图形，字符改用UICharRefresh。
 * @param[in] capacity 实际元素容量，不小于count；调用者保证所声明容量真实可读。
 * @param[in] count 仅1、2、5或7，其他值在读取图形之前拒绝。
 * @return 本机接受返回UI_SUBMITTED；其他返回参数、长度、范围、CRC或发送错误，失败不消耗序号。
 * @note 先成功RefereeInit；由裁判任务串行调用，禁止ISR。传输层复制整帧，输入返回后可复用。
 * 本机接受不表示发送完成或客户端显示；不同提交共享UI_Seq，只在接受后递增并自然回绕。
 */
UIStatus_t UIGraphRefresh(const referee_id_t *_id, const Graph_Data_t *graphs, size_t capacity, size_t count);

/** @brief 提交一份已准备好的文字对象。
 * @param[in] _id 非空且合法的发送者/选手端组合，可读至返回。
 * @param[in] string_Data 非空可读对象，类型必须为字符，声明长度0～30字节；无需NUL。
 * @return 本机接受返回UI_SUBMITTED；否则返回具体错误，失败保持序号，不改输入。
 * @note 任务与存储条件同UIGraphRefresh；只复制声明长度，其余固定文字区发0。
 */
UIStatus_t UICharRefresh(const referee_id_t *_id, const String_Data_t *string_Data);
#endif
