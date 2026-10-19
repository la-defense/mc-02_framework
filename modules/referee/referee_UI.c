/**
 * @file referee_UI.c
 * @author kidneygood (you@domain.com)
 * @brief 检查绘图候选值并显式编码裁判UI帧；发送由裁判任务串行调用。
 * @version 0.1
 * @date 2023-1-18
 * @copyright Copyright (c) 2022
 */
#include "referee_UI.h"
#include "crc.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static RefereeSnapshot_t ui_identity_snapshot; // UI任务串行发送时复用，避免在任务栈上放置整份快照。

// 官方2026 V2.0.0表1-27～1-32；尺寸是编码范围，不推算图形是否完全落在屏幕中。
static int common_valid(uint32_t op, uint32_t layer, uint32_t color, uint32_t width, uint32_t x, uint32_t y)
{
    return op <= 3u && layer <= 9u && color <= 8u && width <= 1023u && x <= 1920u && y <= 1080u;
}

// 名称最多读3字节；遇到结束标记即停止，候选对象清零避免留下上次的名称。
static UIStatus_t prepare(Graph_Data_t *candidate, const void *output, const char *name,
                         uint32_t op, uint32_t layer, uint32_t color,
                         uint32_t width, uint32_t x, uint32_t y, uint32_t type)
{
    if (output == NULL || name == NULL) return UI_ERROR_ARGUMENT;
    if (!common_valid(op, layer, color, width, x, y)) return UI_ERROR_RANGE;
    memset(candidate, 0, sizeof(*candidate));
    for (size_t i = 0; i < 3u && name[i] != '\0'; ++i) candidate->graphic_name[2u-i] = (uint8_t)name[i];
    candidate->operate_tpye = op; candidate->graphic_tpye = type;
    candidate->layer = layer; candidate->color = color;
    candidate->width = width; candidate->start_x = x; candidate->start_y = y;
    return UI_OK;
}

UIStatus_t UILineDraw(Graph_Data_t *graph, const char *name, uint32_t op, uint32_t layer,
                     uint32_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t ex, uint32_t ey)
{
    Graph_Data_t candidate;
    UIStatus_t status = prepare(&candidate, graph, name, op, layer, color, width, x, y, UI_Graph_Line);
    if (status != UI_OK) return status;
    if (ex > 1920u || ey > 1080u) return UI_ERROR_RANGE;
    candidate.end_x = ex; candidate.end_y = ey;
    *graph = candidate;
    return UI_OK;
}

UIStatus_t UIRectangleDraw(Graph_Data_t *graph, const char *name, uint32_t op, uint32_t layer,
                          uint32_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t ex, uint32_t ey)
{
    if (graph == NULL) return UI_ERROR_ARGUMENT;
    Graph_Data_t candidate;
    UIStatus_t status = UILineDraw(&candidate, name, op, layer, color, width, x, y, ex, ey);
    if (status != UI_OK) return status;
    candidate.graphic_tpye = UI_Graph_Rectangle;
    *graph = candidate;
    return UI_OK;
}

UIStatus_t UICircleDraw(Graph_Data_t *graph, const char *name, uint32_t op, uint32_t layer,
                       uint32_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t radius)
{
    Graph_Data_t candidate;
    UIStatus_t status = prepare(&candidate, graph, name, op, layer, color, width, x, y, UI_Graph_Circle);
    if (status != UI_OK) return status;
    if (radius > 1023u) return UI_ERROR_RANGE;
    candidate.radius = radius;
    *graph = candidate;
    return UI_OK;
}

UIStatus_t UIOvalDraw(Graph_Data_t *graph, const char *name, uint32_t op, uint32_t layer,
                     uint32_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t rx, uint32_t ry)
{
    Graph_Data_t candidate;
    UIStatus_t status = prepare(&candidate, graph, name, op, layer, color, width, x, y, UI_Graph_Ellipse);
    if (status != UI_OK) return status;
    if (rx > 2047u || ry > 2047u) return UI_ERROR_RANGE;
    candidate.end_x = rx; candidate.end_y = ry;
    *graph = candidate;
    return UI_OK;
}

UIStatus_t UIArcDraw(Graph_Data_t *graph, const char *name, uint32_t op, uint32_t layer,
                    uint32_t color, uint32_t start, uint32_t end, uint32_t width,
                    uint32_t x, uint32_t y, uint32_t rx, uint32_t ry)
{
    if (graph == NULL) return UI_ERROR_ARGUMENT;
    Graph_Data_t candidate;
    UIStatus_t status = UIOvalDraw(&candidate, name, op, layer, color, width, x, y, rx, ry);
    if (status != UI_OK) return status;
    if (start > 360u || end > 360u) return UI_ERROR_RANGE;
    candidate.graphic_tpye = UI_Graph_Arc;
    candidate.start_angle = start; candidate.end_angle = end;
    *graph = candidate;
    return UI_OK;
}

UIStatus_t UIIntDraw(Graph_Data_t *graph, const char *name, uint32_t op, uint32_t layer,
                    uint32_t color, uint32_t size, uint32_t width, uint32_t x, uint32_t y, int32_t value)
{
    Graph_Data_t candidate;
    UIStatus_t status = prepare(&candidate, graph, name, op, layer, color, width, x, y, UI_Graph_Int);
    if (status != UI_OK) return status;
    if (size > 511u) return UI_ERROR_RANGE;
    candidate.start_angle = size;
    // 先转无符号再拆位，负数也有明确的32位补码结果。
    uint32_t bits = (uint32_t)value;
    candidate.radius = bits & 0x3ffu;
    candidate.end_x = (bits >> 10u) & 0x7ffu;
    candidate.end_y = (bits >> 21u) & 0x7ffu;
    *graph = candidate;
    return UI_OK;
}

UIStatus_t UIFloatDraw(Graph_Data_t *graph, const char *name, uint32_t op, uint32_t layer,
                      uint32_t color, uint32_t size, uint32_t digit, uint32_t width,
                      uint32_t x, uint32_t y, int32_t value)
{
    if (graph == NULL) return UI_ERROR_ARGUMENT;
    Graph_Data_t candidate;
    UIStatus_t status = UIIntDraw(&candidate, name, op, layer, color, size, width, x, y, value);
    if (status != UI_OK) return status;
    if (digit > 511u) return UI_ERROR_RANGE;
    // 2026协议details_b无作用；保留旧形参但固定发送0，不虚构可配置小数位。
    candidate.graphic_tpye = UI_Graph_Float;
    *graph = candidate;
    return UI_OK;
}

UIStatus_t UICharDraw(String_Data_t *graph, const char *name, uint32_t op, uint32_t layer,
                     uint32_t color, uint32_t size, uint32_t width, uint32_t x, uint32_t y, const char *fmt, ...)
{
    String_Data_t candidate = {0};
    UIStatus_t status = prepare(&candidate.Graph_Control, graph, name, op, layer, color, width, x, y, UI_Graph_Char);
    if (status != UI_OK) return status;
    if (fmt == NULL) return UI_ERROR_ARGUMENT;
    if (size > 511u) return UI_ERROR_RANGE;
    char text[31];
    va_list ap;
    va_start(ap, fmt);
    int length = vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    // 临时区另留结束标记；超长整条拒绝，不改旧对象。
    if (length < 0) return UI_ERROR_FORMAT;
    if (length > 30) return UI_ERROR_LENGTH;
    memcpy(candidate.show_Data, text, (size_t)length);
    candidate.Graph_Control.start_angle = size;
    candidate.Graph_Control.end_angle = (uint32_t)length;
    *graph = candidate;
    return UI_OK;
}

// 数字图形的后三个位段合起来是int32，不按坐标检查。
static int valid_graph(const Graph_Data_t *g, int text)
{
    if (!common_valid(g->operate_tpye, g->layer, g->color, g->width, g->start_x, g->start_y)) return 0;
    switch (g->graphic_tpye)
    {
    case UI_Graph_Line:
    case UI_Graph_Rectangle:
        return !text && g->start_angle == 0u && g->end_angle == 0u && g->radius == 0u &&
               g->end_x <= 1920u && g->end_y <= 1080u;
    case UI_Graph_Circle:
        return !text && g->start_angle == 0u && g->end_angle == 0u && g->end_x == 0u && g->end_y == 0u;
    case UI_Graph_Ellipse:
        return !text && g->start_angle == 0u && g->end_angle == 0u && g->radius == 0u;
    case UI_Graph_Arc:
        return !text && g->start_angle <= 360u && g->end_angle <= 360u && g->radius == 0u;
    case UI_Graph_Float:
    case UI_Graph_Int:
        return !text && g->end_angle == 0u;
    case UI_Graph_Char:
        return text && g->end_angle <= 30u && g->radius == 0u && g->end_x == 0u && g->end_y == 0u;
    default: return 0;
    }
}

static void le16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8u);
}

static void le32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0; i < 4u; ++i) out[i] = (uint8_t)(value >> (8u*i));
}

// 位域仅作为本地对象，线上字节显式组合，不依赖编译器布局。
static void encode_graph(uint8_t *out, const Graph_Data_t *g)
{
    memcpy(out, g->graphic_name, 3u);
    le32(out+3, g->operate_tpye | (g->graphic_tpye << 3u) | (g->layer << 6u) |
         (g->color << 10u) | (g->start_angle << 14u) | ((uint32_t)g->end_angle << 23u));
    le32(out+7, g->width | (g->start_x << 10u) | ((uint32_t)g->start_y << 21u));
    le32(out+11, g->radius | (g->end_x << 10u) | ((uint32_t)g->end_y << 21u));
}

// 附录二只列出每方1～6号选手端；雷达另允许向任意己方选手端发送。
static int valid_id(const referee_id_t *id)
{
    unsigned client = id->Cilent_ID;
    if (client < 0x100u) return 0;
    client -= 0x100u;
    int red = client >= 1u && client <= 6u;
    int blue = client >= 101u && client <= 106u;
    return (red || blue) && (id->Robot_ID == client ||
           (red && id->Robot_ID == 9u) || (blue && id->Robot_ID == 109u));
}

static UIStatus_t validate_current_id(const referee_id_t *id)
{
    if (RefereeGetSnapshot(&ui_identity_snapshot) != REFEREE_OK ||
        ui_identity_snapshot.command[REF_CMD_ROBOT_STATE].state != REFEREE_DATA_FRESH)
        return UI_NOT_INITIALIZED;
    const RefereeRobotState_t *robot = &ui_identity_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state;
    if (id->Robot_ID != robot->robot_id || id->Cilent_ID != (uint16_t)(0x100u+robot->robot_id) ||
        id->Robot_Color != (robot->robot_id >= 100u ? Robot_Blue : Robot_Red))
        return UI_NOT_INITIALIZED;
    return UI_OK;
}

static UIStatus_t send_frame(const referee_id_t *id, uint16_t command, const uint8_t *content, size_t length)
{
    if (id == NULL) return UI_ERROR_ARGUMENT;
    if (!valid_id(id)) return UI_ERROR_RANGE;
    UIStatus_t identity_status = validate_current_id(id);
    if (identity_status != UI_OK) return identity_status;
    uint8_t frame[120] = {0};
    size_t total = 15u + length;
    if (total > sizeof(frame)) return UI_ERROR_LENGTH;
    frame[0] = REFEREE_SOF;
    le16(frame+1, (uint16_t)(6u+length));
    frame[3] = UI_Seq;
    uint32_t crc;
    if (CRCCompute(CRC8_REFEREE, frame, 4u, &crc) != CRC_OK) return UI_ERROR_CRC;
    frame[4] = (uint8_t)crc;
    le16(frame+5, ID_student_interactive);
    le16(frame+7, command); le16(frame+9, id->Robot_ID); le16(frame+11, id->Cilent_ID);
    memcpy(frame+13, content, length);
    if (CRCCompute(CRC16_SP_REFEREE, frame, total-2u, &crc) != CRC_OK) return UI_ERROR_CRC;
    le16(frame+total-2u, (uint16_t)crc);
    RefereeSendStatus_t status = RefereeSend(frame, (uint16_t)total);
    // 仅本机接受后推进序号，失败不消费序号，也不冒称客户端已经显示。
    switch (status)
    {
    case REFEREE_SEND_ACCEPTED: ++UI_Seq; return UI_SUBMITTED;
    case REFEREE_SEND_NOT_INITIALIZED: return UI_NOT_INITIALIZED;
    case REFEREE_SEND_BUSY: return UI_SEND_BUSY;
    case REFEREE_SEND_TIMEOUT: return UI_SEND_TIMEOUT;
    default: return UI_SEND_ERROR;
    }
}

UIStatus_t UIDelete(const referee_id_t *id, uint8_t operation, uint8_t layer)
{
    if (operation > 2u || layer > 9u) return UI_ERROR_RANGE;
    uint8_t content[2] = {operation, layer};
    return send_frame(id, UI_Data_ID_Del, content, sizeof(content));
}

UIStatus_t UIGraphRefresh(const referee_id_t *id, const Graph_Data_t *graphs, size_t capacity, size_t count)
{
    if (id == NULL || graphs == NULL) return UI_ERROR_ARGUMENT;
    uint16_t command;
    switch (count)
    {
    case 1: command = UI_Data_ID_Draw1; break;
    case 2: command = UI_Data_ID_Draw2; break;
    case 5: command = UI_Data_ID_Draw5; break;
    case 7: command = UI_Data_ID_Draw7; break;
    default: return UI_ERROR_LENGTH;
    }
    if (count > capacity) return UI_ERROR_LENGTH;
    uint8_t content[105];
    // 整批检查并编码成功才提交，某个对象非法时不发送前半批。
    for (size_t i = 0; i < count; ++i)
    {
        if (!valid_graph(&graphs[i], 0)) return UI_ERROR_RANGE;
        encode_graph(content+15u*i, &graphs[i]);
    }
    return send_frame(id, command, content, count*15u);
}

UIStatus_t UICharRefresh(const referee_id_t *id, const String_Data_t *text)
{
    if (id == NULL || text == NULL) return UI_ERROR_ARGUMENT;
    if (!valid_graph(&text->Graph_Control, 1)) return UI_ERROR_RANGE;
    uint8_t content[45] = {0};
    encode_graph(content, &text->Graph_Control);
    // 仅复制声明长度，余下固定文字区保持0；30字节文本无需结束标记。
    memcpy(content+15, text->show_Data, text->Graph_Control.end_angle);
    return send_frame(id, UI_Data_ID_DrawChar, content, sizeof(content));
}
