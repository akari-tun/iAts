#include <Arduino.h>
#include "config.h"
#include "tprotocol.h"
#include "eeprom_functions.h"

//协议解析到什么状态了
enum DataState
{
    IDLE,
    STATE_LEAD,
    STATE_START,
    STATE_CMD,
    STATE_INDEX,
    STATE_LEN,
    STATE_DATA
};

static uint8_t Tag_Max_Length = MAX_TAG_COUNT * 6;
static uint8_t Receive_Tag_Buffer[MAX_TAG_COUNT * 6]; //接收TAG域的缓存，最多缓存10个TAG
static uint8_t Receive_Tag_Length = 0;
static uint8_t Receive_Tag_Index = 0;
static uint8_t Receive_TProtocol_Cmd = 0;
static uint8_t Receive_TProtocol_Chk = 0;
static uint8_t Receive_Cmd_Index = 0;
static uint8_t Send_Head_Buffer[5];                //发送的缓存头
static uint8_t Send_Tag_Buffer[MAX_TAG_COUNT * 6]; //发送TAG域的缓存，最多缓存10个TAG
static uint8_t Send_Tag_Length = 0;
static uint8_t Send_Tag_Index = 0;
static uint8_t Send_TProtocol_Cmd = 0;
static uint8_t Send_TProtocol_Chk = 0;
static uint8_t Send_Cmd_Index = 0;
static uint8_t Send_Cmd_Buffer[MAX_CMD_COUNT];
static uint8_t Send_Cmd_Count = 0;
static uint8_t Receive_Cmd_Buffer[MAX_CMD_COUNT];
static uint8_t Receive_Cmd_Count = 0;
static uint8_t Receive_Cmd = 0;
static uint8_t Receive_Cmd_Ctr_Tag[MAX_TAG_COUNT]; //控制指令接收到的控制Tag
static uint8_t Receive_Cmd_Ctr_Tag_Count = 0;      //控制指令接收到的控制Tag数量

static uint8_t Data_State = IDLE;

static unsigned long Last_HeartBeet_Ack = 0;

static Airplane *PTarget;
static Tracker *PTracker;
static Parameter *PTParam;

static uint8_t Tag_Len_Map[TAG_COUNT][2] = {
    {TAG_BASE_ACK, 1},
    {TAG_BASE_HEARTBEAT, 1},
    {TAG_BASE_QUERY, 1},
    {TAG_PLANE_LONGITUDE, 4},
    {TAG_PLANE_LATITUDE, 4},
    {TAG_PLANE_ALTITUDE, 4},
    {TAG_PLANE_PITCH, 2},
    {TAG_PLANE_ROLL, 2},
    {TAG_PLANE_HEADING, 2},
    {TAG_PLANE_SPEED, 2},
    {TAG_PLANE_DISTANCE, 4},
    {TAG_PLANE_STAR, 1},
    {TAG_PLANE_FIX, 1},
    {TAG_HOME_LONGITUDE, 4},
    {TAG_HOME_LATITUDE, 4},
    {TAG_HOME_ALTITUDE, 2},
    {TAG_HOME_HEADING, 2},
    {TAG_HOME_PITCHING, 1},
    {TAG_HOME_VOLTAGE, 2},
    {TAG_HOME_MODE, 1},
    {TAG_HOME_DECLINATION, 1},
    {TAG_PARAM_PID_P, 2},
    {TAG_PARAM_PID_I, 2},
    {TAG_PARAM_PID_D, 2},
    {TAG_PARAM_TITL_0, 2},
    {TAG_PARAM_TITL_90, 2},
    {TAG_PARAM_PAN_0, 2},
    {TAG_PARAM_OFFSET, 2},
    {TAG_PARAM_START_TRACKING_DISTANCE, 1},
    {TAG_PARAM_MAX_PID_ERROR, 1},
    {TAG_PARAM_MIN_PAN_SPEED, 1},
    {TAG_PARAM_DECLINATION, 1},
    {TAG_CTR_MODE, 1},
    {TAG_CTR_AUTO_POINT_TO_NORTH, 1},
    {TAG_CTR_CALIBRATE, 1},
    {TAG_CTR_HEADING, 2},
    {TAG_CTR_TILT, 1}
};

uint8_t get_tag_len(uint8_t tag);
void set_send_tag_buffer_u8(uint8_t tag, uint8_t value);
void set_send_tag_buffer_u16(uint8_t tag, uint16_t value);
void set_send_tag_buffer_u32(uint8_t tag, uint32_t value);

/*
    初始化
*/
void tprotocol(Airplane *target, Tracker *tracker, Parameter *param)
{
    PTarget = target;
    PTracker = tracker;
    PTParam = param;
    Send_Head_Buffer[0] = TP_PACKET_LEAD;
    Send_Head_Buffer[1] = TP_PACKET_START;

    /* 读取配置参数 */
    PTParam->pid_p = LoadFromEEPROM_u16(PARAM_EPPROM_POS_PID_P);
    PTParam->pid_i = LoadFromEEPROM_u16(PARAM_EPPROM_POS_PID_I);
    PTParam->pid_d = LoadFromEEPROM_u16(PARAM_EPPROM_POS_PID_D);
    PTParam->pid_divider = LoadFromEEPROM_u16(PARAM_EPPROM_POS_PID_DIVIDER);
    PTParam->pid_max_error = LoadFromEEPROM_u16(PARAM_EPPROM_POS_PID_MAX_ERROR);
    PTParam->tilt_0 = LoadFromEEPROM_u16(PARAM_EPPROM_POS_TILT_0);
    PTParam->tilt_90 = LoadFromEEPROM_u16(PARAM_EPPROM_POS_TILT_90);
    PTParam->pan_center = LoadFromEEPROM_u16(PARAM_EPPROM_POS_PAN_CENTER);
    PTParam->pan_min_speed = LoadFromEEPROM_u8(PARAM_EPPROM_POS_PAN_MIN_SPEED);
    PTParam->compass_offset = LoadFromEEPROM_u16(PARAM_EPPROM_POS_COMPASS_OFFSET);
    PTParam->compass_declination = (int8_t)LoadFromEEPROM_u8(PARAM_EPPROM_POS_COMPASS_DECLINATION);
    PTParam->start_tracking_distance = LoadFromEEPROM_u8(PARAM_EPPROM_POS_START_TRACKING_DISTANCE);

    if (PTParam->pid_p == 0 || PTParam->pid_p == 0xffff)
        PTParam->pid_p = 4500;
    if (PTParam->pid_i == 0xffff)
        PTParam->pid_i = 0;
    if (PTParam->pid_d == 0xffff)
        PTParam->pid_d = 0;
    if (PTParam->pid_divider == 0 || PTParam->pid_divider == 0xff)
        PTParam->pid_divider = 15;
    if (PTParam->pid_max_error == 0 || PTParam->pid_max_error == 0xff)
        PTParam->pid_max_error = 10;
    if (PTParam->tilt_0 == 0 || PTParam->tilt_0 == 0xffff)
        PTParam->tilt_0 = 1500;
    if (PTParam->tilt_90 == 0 || PTParam->tilt_90 == 0xffff)
        PTParam->tilt_90 = 1500;
    if (PTParam->pan_center == 0 || PTParam->pan_center == 0xffff)
        PTParam->pan_center = 1500;
    if (PTParam->pan_min_speed == 0 || PTParam->pan_min_speed == 0xff)
        PTParam->pan_min_speed = 50;
    if (PTParam->compass_offset == 0xffff)
        PTParam->compass_offset = 0;
    if (PTParam->compass_declination == 0xff)
        PTParam->compass_declination = 0;
    if (PTParam->start_tracking_distance == 0 || PTParam->start_tracking_distance == 0xff)
        PTParam->start_tracking_distance = 10;
}

/*
    解析蓝牙传输过来的数据
    借鉴了之前LTM.cpp中的代码
*/
boolean encodeTrackerData(uint8_t c)
{
    boolean ret = true;

    if (Data_State == IDLE && c == TP_PACKET_LEAD)
    {
        Data_State = STATE_LEAD;
        #ifdef DEBUG
        Serial.print("STATE_LEAD:");
        Serial.println(c, HEX);
        #endif
    }
    else if (Data_State == STATE_LEAD && c == TP_PACKET_START)
    {
        Data_State = STATE_START;
        #ifdef DEBUG
        Serial.print("STATE_START:");
        Serial.println(c, HEX);
        #endif
    }
    else if (Data_State == STATE_START)
    {
        Data_State = STATE_CMD;
        Receive_TProtocol_Cmd = c;
        #ifdef DEBUG
        Serial.print("STATE_CMD:");
        Serial.println(c, HEX);
        #endif
    }
    else if (Data_State == STATE_CMD)
    {
        Data_State = STATE_INDEX;
        Receive_Cmd_Index = c;
        #ifdef DEBUG
        Serial.print("STATE_INDEX:");
        Serial.println(c, HEX);
        #endif
    }
    else if (Data_State == STATE_INDEX)
    {
        Data_State = STATE_LEN;
        Receive_Tag_Length = c;
        #ifdef DEBUG
        Serial.print("STATE_LEN:");
        Serial.println(c, HEX);
        #endif
    }
    else if ((Data_State == STATE_LEN || Data_State == STATE_DATA) && Receive_Tag_Length <= Tag_Max_Length)
    {
        if (Data_State == STATE_LEN)
        {
            Data_State = STATE_DATA;
            Receive_Tag_Index = 0;
        }

        if (Receive_Tag_Index == 0)
        {
            Receive_TProtocol_Chk = c;
            #ifdef DEBUG
            Serial.print("DATA:");
            #endif
        }
        else
        {
            Receive_TProtocol_Chk ^= c;
        }

        if (Receive_Tag_Index == Receive_Tag_Length)
        {
            #ifdef DEBUG
            Serial.println("");
            #endif
            //计算接收的数据域的TAG值
            if (Receive_TProtocol_Chk == 0)
            {
                //接收数据成功
                analysis();
                BT_CONNECTED = true;
                Data_State = IDLE;
                #ifdef DEBUG
                Serial.println("SUCCESS");
                #endif
            }
            else
            {
                //错误的数据，将数据丢弃
                Data_State = IDLE;
                ret = false;
                #ifdef DEBUG
                Serial.println("FAIL");
                #endif
            }
        }
        else
        {
            Receive_Tag_Buffer[Receive_Tag_Index++] = c;
            #ifdef DEBUG
            Serial.print(c, HEX);
            Serial.print(" ");
            #endif
        }
    }
    else
    {
        Data_State = IDLE;
        ret = false;
    }

    return ret;
}

/*
    解析TAG域中的数据
*/
void analysis()
{
    Receive_Tag_Index = 0;

    #ifdef DEBUG
    Serial.println("-----------TAG------------");
    Serial.println(Receive_TProtocol_Cmd, HEX);
    #endif

    switch (Receive_TProtocol_Cmd)
    {
    case CMD_D_HEARTBEAT:
        while (Receive_Tag_Index < Receive_Tag_Length)
        {
            #ifdef DEBUG
            Serial.print("Tag:");
            Serial.print(Receive_Tag_Buffer[Receive_Tag_Index], HEX);
            Serial.print(" | Value:");
            #endif

            switch (Receive_Tag_Buffer[Receive_Tag_Index++])
            {
            case TAG_BASE_ACK: //应答结果 L:1
                Receive_Tag_Index++;
                BT_CONNECTED = tagread_u8();
                if (BT_CONNECTED)
                    Last_HeartBeet_Ack = millis();
                #ifdef DEBUG
                Serial.print(BT_CONNECTED);
                #endif
                break;
            default:
                Receive_Tag_Index = Receive_Tag_Index + Receive_Tag_Buffer[Receive_Tag_Index++];
                break;
            }
            #ifdef DEBUG
            Serial.println("");
            #endif
        }
        break;
    case CMD_D_SET_HOME:
        while (Receive_Tag_Index < Receive_Tag_Length)
        {
            #ifdef DEBUG
            Serial.print("Tag:");
            Serial.print(Receive_Tag_Buffer[Receive_Tag_Index], HEX);
            Serial.print(" | Value:");
            #endif

            switch (Receive_Tag_Buffer[Receive_Tag_Index++])
            {
                case TAG_HOME_LONGITUDE: //家的经度 L:4
                    Receive_Tag_Index++;
                    PTracker->lon = tagread_u32();
                    #ifdef DEBUG
                    Serial.print(PTracker->lat);
                    #endif
                    break;
                case TAG_HOME_LATITUDE: //家的纬度 L:4
                    Receive_Tag_Index++;
                    PTracker->lat = tagread_u32();
                    #ifdef DEBUG
                    Serial.print(PTracker->lon);
                    #endif
                    break;
                case TAG_HOME_ALTITUDE: //家的高度 L:4
                    Receive_Tag_Index++;
                    PTracker->alt = tagread_u32();
                    #ifdef DEBUG
                    Serial.print(PTracker->alt);
                    #endif
                    break;
                case TAG_HOME_MODE: //家的模式 L:1
                    Receive_Tag_Index++;
                    PTracker->mode = tagread_u8();
                    #ifdef DEBUG
                    Serial.print(PTracker->mode);
                    #endif
                    break;
                case TAG_HOME_DECLINATION: //磁偏角 L:1
                    Receive_Tag_Index++;
                    PTracker->declination = (int8_t)tagread_u8();
                    #ifdef DEBUG
                    Serial.print(PTracker->mode);
                    #endif
                    break;
                default:
                    #ifdef DEBUG
                    Serial.print("Invald Tag");
                    #endif
                    Receive_Tag_Index++;
                    break;
            }
            #ifdef DEBUG
            Serial.println("");
            #endif
        }

        if (PTracker->lon > 0 && PTracker->lat > 0)
        {
            HOME_SETED = true;
            Receive_Cmd = Receive_TProtocol_Cmd;
            setSendCmdFirst(CMD_U_ACK);
        }
        break;
    case CMD_D_SET_PARAM:
        while (Receive_Tag_Index < Receive_Tag_Length)
        {
            #ifdef DEBUG
            Serial.print("Tag:");
            Serial.print(Receive_Tag_Buffer[Receive_Tag_Index], HEX);
            Serial.print(" | Value:");
            #endif

            switch (Receive_Tag_Buffer[Receive_Tag_Index++])
            {
                case TAG_PARAM_PID_P: //PID_P L:2
                    Receive_Tag_Index++;
                    PTParam->pid_p = tagread_u16();
                    StoreToEEPROM_u16(PTParam->pid_p, PARAM_EPPROM_POS_PID_P);
                    #ifdef DEBUG
                    Serial.print(PTParam->pid_p);
                    #endif
                    break;
                case TAG_PARAM_PID_I: //PID_I L:2
                    Receive_Tag_Index++;
                    PTParam->pid_i = tagread_u16();
                    StoreToEEPROM_u16(PTParam->pid_i, PARAM_EPPROM_POS_PID_I);
                    #ifdef DEBUG
                    Serial.print(PTParam->pid_i);
                    #endif
                    break;
                case TAG_PARAM_PID_D: //PID_D L:2
                    Receive_Tag_Index++;
                    PTParam->pid_d = tagread_u16();
                    StoreToEEPROM_u16(PTParam->pid_d, PARAM_EPPROM_POS_PID_D);
                    #ifdef DEBUG
                    Serial.print(PTParam->pid_d);
                    #endif
                    break;
                case TAG_PARAM_TITL_0: //俯仰零度 L:2
                    Receive_Tag_Index++;
                    PTParam->tilt_0 = tagread_u16();
                    StoreToEEPROM_u16(PTParam->tilt_0, PARAM_EPPROM_POS_TILT_0);
                    #ifdef DEBUG
                    Serial.print(PTParam->tilt_0);
                    #endif
                    break;
                case TAG_PARAM_TITL_90: //俯仰90度 L:2
                    Receive_Tag_Index++;
                    PTParam->tilt_90 = tagread_u16();
                    StoreToEEPROM_u16(PTParam->tilt_90, PARAM_EPPROM_POS_TILT_90);
                    #ifdef DEBUG
                    Serial.print(PTParam->tilt_90);
                    #endif
                    break;
                case TAG_PARAM_PAN_0: //水平中立点 L:2
                    Receive_Tag_Index++;
                    PTParam->pan_center = tagread_u16();
                    StoreToEEPROM_u16(PTParam->pan_center, PARAM_EPPROM_POS_PAN_CENTER);
                    #ifdef DEBUG
                    Serial.print(PTParam->pan_center);
                    #endif
                    break;
                case TAG_PARAM_OFFSET: //罗盘偏移量 L:2
                    Receive_Tag_Index++;
                    PTParam->compass_offset = (int8_t)tagread_u16();
                    StoreToEEPROM_u16(PTParam->compass_offset, PARAM_EPPROM_POS_COMPASS_OFFSET);
                    #ifdef DEBUG
                    Serial.print(PTParam->compass_offset);
                    #endif
                    break;
                case TAG_PARAM_START_TRACKING_DISTANCE: //开始跟踪距离 L:1
                    Receive_Tag_Index++;
                    PTParam->start_tracking_distance = tagread_u8();
                    StoreToEEPROM_u8(PTParam->start_tracking_distance, PARAM_EPPROM_POS_START_TRACKING_DISTANCE);
                    #ifdef DEBUG
                    Serial.print(PTParam->start_tracking_distance);
                    #endif
                    break;
                case TAG_PARAM_MAX_PID_ERROR: //最大角度偏移 L:1
                    Receive_Tag_Index++;
                    PTParam->pid_max_error = tagread_u8();
                    StoreToEEPROM_u8(PTParam->pid_max_error, PARAM_EPPROM_POS_PID_MAX_ERROR);
                    #ifdef DEBUG
                    Serial.print(PTParam->pid_max_error);
                    #endif
                    break;
                case TAG_PARAM_MIN_PAN_SPEED: //最小水平舵机速度 L:1
                    Receive_Tag_Index++;
                    PTParam->pan_min_speed = tagread_u8();
                    StoreToEEPROM_u8(PTParam->pan_min_speed, PARAM_EPPROM_POS_PAN_MIN_SPEED);
                    #ifdef DEBUG
                    Serial.print(PTParam->pan_min_speed);
                    #endif
                    break;
                case TAG_PARAM_DECLINATION: //磁偏角 L:1
                    Receive_Tag_Index++;
                    PTParam->compass_declination = (int8_t)tagread_u8();
                    StoreToEEPROM_u8((uint8_t)PTParam->compass_declination, PARAM_EPPROM_POS_COMPASS_DECLINATION);
                    #ifdef DEBUG
                    Serial.print(PTParam->compass_declination);
                    #endif
                    break;
                default:
                    #ifdef DEBUG
                    Serial.print("Invald Tag");
                    #endif
                    Receive_Tag_Index++;
                    break;
            }
            #ifdef DEBUG
            Serial.println("");
            #endif
        }

        Receive_Cmd = Receive_TProtocol_Cmd;
        setSendCmdFirst(CMD_U_ACK);
        break;
    case CMD_D_QUERY_PARAM:
        setSendCmd(CMD_U_PARAM);
        break;
    case CMD_D_CONTROL:
        Receive_Cmd = Receive_TProtocol_Cmd;
        setSendCmdFirst(CMD_U_ACK);
        Receive_Cmd_Ctr_Tag_Count = 0; //将接收到的控制Tag指令数量初始化
        while (Receive_Tag_Index < Receive_Tag_Length)
        {
            #ifdef DEBUG
            Serial.print("Tag:");
            Serial.print(Receive_Tag_Buffer[Receive_Tag_Index], HEX);
            Serial.print(" | Value:");
            #endif

            switch (Receive_Tag_Buffer[Receive_Tag_Index++])
            {
                case TAG_CTR_MODE: //设置模式 L:1
                    Receive_Tag_Index++;
                    PTracker->mode = tagread_u8();
                    setReceiveCmd(TAG_CTR_MODE);
                    Receive_Cmd_Ctr_Tag[Receive_Cmd_Ctr_Tag_Count++] = TAG_CTR_MODE;
                    #ifdef DEBUG
                    Serial.print("Set Mode:");
                    Serial.println(PTracker->mode);
                    #endif
                    break;
                case TAG_CTR_AUTO_POINT_TO_NORTH: //设置自动指北 L:1
                    Receive_Tag_Index++;
                    AUTO_POINT_TO_NORTH = tagread_u8();
                    Receive_Cmd_Ctr_Tag[Receive_Cmd_Ctr_Tag_Count++] = TAG_CTR_AUTO_POINT_TO_NORTH;
                    #ifdef DEBUG
                    Serial.print("Point To North:");
                    Serial.print(AUTO_POINT_TO_NORTH);
                    #endif
                    break;
                case TAG_CTR_CALIBRATE: //校准 L:1
                    Receive_Tag_Index++;
                    uint8_t cal;
                    cal = tagread_u8();
                    setReceiveCmd(TAG_CTR_CALIBRATE);
                    Receive_Cmd_Ctr_Tag[Receive_Cmd_Ctr_Tag_Count++] = TAG_CTR_CALIBRATE;
                    #ifdef DEBUG
                    Serial.print("Calibrate");
                    #endif
                    break;
                case TAG_CTR_HEADING: //指向 L:2
                    Receive_Tag_Index++;
                    PTracker->course = tagread_u16();
                    setReceiveCmd(TAG_CTR_HEADING);
                    Receive_Cmd_Ctr_Tag[Receive_Cmd_Ctr_Tag_Count++] = TAG_CTR_HEADING;
                    #ifdef DEBUG
                    Serial.print("Heading:");
                    Serial.println(PTracker->course);
                    #endif
                    break;
                case TAG_CTR_TILT: //俯仰 L:1
                    Receive_Tag_Index++;
                    PTracker->pitching = tagread_u8();
                    setReceiveCmd(TAG_CTR_TILT);
                    Receive_Cmd_Ctr_Tag[Receive_Cmd_Ctr_Tag_Count++] = TAG_CTR_TILT;
                    #ifdef DEBUG
                    Serial.print("Tilt:");
                    Serial.println(PTracker->pitching);
                    #endif
                    break;
                default:
                    #ifdef DEBUG
                    Serial.print("Invald Tag");
                    #endif
                    Receive_Tag_Index++;
                    break;
            }
            #ifdef DEBUG
            Serial.println("");
            #endif
        }
        break;
    }

    #ifdef DEBUG
    Serial.println("--------------------------");
    #endif
}

/*
    将下一指令提交到发送缓存
*/
void setSendCmd(uint8_t cmd)
{
    if (Send_Cmd_Count < MAX_CMD_COUNT)
    {
        for (uint8_t i = 0; i < MAX_CMD_COUNT; i++)
        {
            if (Send_Cmd_Buffer[i] == 0)
            {
                Send_Cmd_Buffer[i] = cmd;
                Send_Cmd_Count++;
                break;
            }
            else if (Send_Cmd_Buffer[i] == cmd)
            {
                break;
            }
        }
    }
}

/*
    提交到接收到的命令的缓存
*/
void setReceiveCmd(uint8_t cmd)
{
    if (Receive_Cmd_Count < MAX_CMD_COUNT)
    {
        for (uint8_t i = 0; i < MAX_CMD_COUNT; i++)
        {
            if (Receive_Cmd_Buffer[i] == 0)
            {
                Receive_Cmd_Buffer[i] = cmd;
                Receive_Cmd_Count++;
                break;
            }
            else if (Receive_Cmd_Buffer[i] == cmd)
            {
                break;
            }
        }
    }
}

/*
    将下一指令提交到发送缓存
*/
void setSendCmdFirst(uint8_t cmd)
{
    for (uint8_t i = MAX_CMD_COUNT - 1; i > 0; i--)
    {
        Send_Cmd_Buffer[i] = Send_Cmd_Buffer[i - 1];
        if (i == 1)
        {
            // Serial.print("Set Cmd First = ");
            // Serial.println(cmd, HEX);
            Send_Cmd_Buffer[0] = cmd;
            Send_Cmd_Count++;
        }
    }
}

/*
    缓存着的命令数量
*/
uint8_t getCmdCount()
{
    return Send_Cmd_Count;
}

/*
	最后心跳应答时间
*/
unsigned long getLastHeartBeetAck()
{
    return Last_HeartBeet_Ack;
}

/*
    取得将要发送的数据
*/
void getSendData(uint8_t *&p_data, uint8_t &len)
{
    uint8_t i;

    if (!BT_DATA_SENDING && Send_Cmd_Count != 0)
    {
        Send_Tag_Length = 0;
        Send_Tag_Index = 0;

        switch (Send_Cmd_Buffer[0])
        {
        case CMD_U_HEARTBEAT:
            set_send_tag_buffer_u8(TAG_BASE_HEARTBEAT, 0x01);
            if (!HOME_SETED)
                set_send_tag_buffer_u8(TAG_BASE_QUERY, CMD_D_SET_HOME);
            break;
        case CMD_U_AIRPLANE:
            set_send_tag_buffer_u32(TAG_PLANE_LONGITUDE, PTarget->lon);
            set_send_tag_buffer_u32(TAG_PLANE_LATITUDE, PTarget->lat);
            set_send_tag_buffer_u32(TAG_PLANE_ALTITUDE, PTarget->alt);
            set_send_tag_buffer_u16(TAG_PLANE_DISTANCE, PTarget->distance);
            set_send_tag_buffer_u16(TAG_PLANE_HEADING, PTarget->heading);
            set_send_tag_buffer_u16(TAG_PLANE_SPEED, PTarget->speed);
            set_send_tag_buffer_u8(TAG_PLANE_STAR, PTarget->sats);
            set_send_tag_buffer_u8(TAG_PLANE_FIX, PTarget->fix_type);
            break;
        case CMD_U_TRACKER:
            set_send_tag_buffer_u16(TAG_HOME_HEADING, PTracker->heading);
            set_send_tag_buffer_u8(TAG_HOME_PITCHING, PTracker->pitching);
            set_send_tag_buffer_u16(TAG_HOME_VOLTAGE, PTracker->voltage);
            set_send_tag_buffer_u8(TAG_HOME_MODE, PTracker->mode);
            break;
        case CMD_U_PARAM:
            set_send_tag_buffer_u16(TAG_PARAM_PID_P, PTParam->pid_p);
            set_send_tag_buffer_u16(TAG_PARAM_PID_I, PTParam->pid_i);
            set_send_tag_buffer_u16(TAG_PARAM_PID_D, PTParam->pid_d);
            set_send_tag_buffer_u16(TAG_PARAM_TITL_0, PTParam->tilt_0);
            set_send_tag_buffer_u16(TAG_PARAM_TITL_90, PTParam->tilt_90);
            set_send_tag_buffer_u16(TAG_PARAM_OFFSET, PTParam->compass_offset);
            set_send_tag_buffer_u8(TAG_PARAM_START_TRACKING_DISTANCE, PTParam->start_tracking_distance);
            set_send_tag_buffer_u8(TAG_PARAM_MAX_PID_ERROR, PTParam->pid_max_error);
            set_send_tag_buffer_u8(TAG_PARAM_MIN_PAN_SPEED, PTParam->pan_min_speed);
            break;
        case CMD_U_ACK:
            set_send_tag_buffer_u8(TAG_BASE_ACK, Receive_Cmd);
            // Serial.print("Send ACK = ");
            // Serial.println(Receive_Cmd, HEX);

            if (Receive_Cmd_Ctr_Tag_Count > 0)
            {
                // Serial.print("Receive_Cmd_Ctr_Tag_Count = ");
                // Serial.println(Receive_Cmd_Ctr_Tag_Count);

                for (i = 0; i< Receive_Cmd_Ctr_Tag_Count; i++)
                {
                    switch (Receive_Cmd_Ctr_Tag[i])
                    {
                        case TAG_CTR_MODE: //设置模式 L:1
                            set_send_tag_buffer_u8(TAG_CTR_MODE, PTracker->mode);
                            // Serial.print("TAG_CTR_MODE = ");
                            // Serial.println(PTracker->mode);
                            break;
                        case TAG_CTR_AUTO_POINT_TO_NORTH: //设置自动指北 L:1
                            set_send_tag_buffer_u8(TAG_CTR_AUTO_POINT_TO_NORTH, AUTO_POINT_TO_NORTH);
                            break;
                        case TAG_CTR_CALIBRATE: //校准 L:1
                            set_send_tag_buffer_u8(TAG_CTR_CALIBRATE, 0x01);
                            break;
                        case TAG_CTR_HEADING: //指向 L:1
                            set_send_tag_buffer_u16(TAG_CTR_HEADING, PTracker->course);
                            break;
                        case TAG_CTR_TILT: //俯仰 L:1
                            set_send_tag_buffer_u8(TAG_CTR_TILT, PTracker->pitching);
                            break;
                    }
                }

                Receive_Cmd_Ctr_Tag_Count = 0;
            }
            break;
        default:
            //将发送队列中已经处理的命令清掉
            for (i = 1; i < MAX_CMD_COUNT; i++)
            {
                Send_Cmd_Buffer[i - 1] = Send_Cmd_Buffer[i];
                Send_Cmd_Buffer[i] = 0;
                Send_Cmd_Count--;
            }
            return;
        }

        if (Send_Cmd_Index >= 255)
            Send_Cmd_Index = 0;

        Send_Cmd_Index++;

        Send_Head_Buffer[2] = Send_Cmd_Buffer[0];
        Send_Head_Buffer[3] = Send_Cmd_Index;
        Send_Head_Buffer[4] = Send_Tag_Length;

        p_data = &Send_Head_Buffer[0];
        len = 5;

        //将发送队列中已经处理的命令清掉
        for (i = 1; i < MAX_CMD_COUNT; i++)
        {
            Send_Cmd_Buffer[i - 1] = Send_Cmd_Buffer[i];
            Send_Cmd_Buffer[i] = 0;
        }

        Send_TProtocol_Chk = 0;
        Send_Cmd_Count--;
        BT_DATA_SENDING = true;

        #ifdef DEBUG
        Serial.println("-----------SendStart------------");
        Serial.print("Cmd = ");
        Serial.println(Send_Head_Buffer[2], HEX);
        Serial.println("------------------");
        Serial.print("Len = ");
        Serial.println(len);
        Serial.print("Point = ");
        Serial.println(*p_data);
        #endif
    }
    else if (BT_DATA_SENDING)
    {
        p_data = &Send_Tag_Buffer[Send_Tag_Index];
        len = (get_tag_len(Send_Tag_Buffer[Send_Tag_Index]) + 2);

        #ifdef DEBUG
        Serial.print("Send_Tag_Index = ");
        Serial.println(Send_Tag_Index);
        Serial.print("Len = ");
        Serial.println(len);
        Serial.print("Point = ");
        Serial.println(*p_data);
        Serial.print("S = ");
        #endif

        for (i = 0; i < len; i++)
        {
            #ifdef DEBUG
            Serial.print(Send_Tag_Buffer[Send_Tag_Index], HEX);
            Serial.print(" ");
            #endif
            Send_TProtocol_Chk ^= Send_Tag_Buffer[Send_Tag_Index++];
        }

        #ifdef DEBUG
        Serial.println("");
        #endif

        //判断数据是否发送完了
        if (Send_Tag_Index >= Send_Tag_Length)
        {
            len++;
            Send_Tag_Buffer[Send_Tag_Index++] = Send_TProtocol_Chk;
            BT_DATA_SENDING = false;
            #ifdef DEBUG
            Serial.println("-----------SendEnd------------");
            #endif
        }
    }
    else
    {
        len = 0;
        // Serial.println("len = 0");
    }
}

/*
    取得下发下来需要执行的命令
*/
uint8_t getReceiveCmd()
{
    uint8_t ret = 0;
    uint8_t i;

    if (Receive_Cmd_Buffer[0] != 0)
    {
        ret = Receive_Cmd_Buffer[0];
        Receive_Cmd_Buffer[0] = 0;
        //将队列中已经处理的命令清掉
        for (i = 1; i < MAX_CMD_COUNT; i++)
        {
            if (Receive_Cmd_Buffer[i] == 0)
                break;
            Receive_Cmd_Buffer[i - 1] = Receive_Cmd_Buffer[i];
            Receive_Cmd_Buffer[i] = 0;
        }
        if (Receive_Cmd_Count > 0) Receive_Cmd_Count--;
    }

    return ret;
}

/*
    取得对应Tag的长度
*/
uint8_t get_tag_len(uint8_t tag)
{
    for (uint8_t i = 0; i < TAG_COUNT; i++)
    {
        if (Tag_Len_Map[i][0] == tag)
            return Tag_Len_Map[i][1];
    }

    return 0;
}

/*
    填充sendbuffer
*/
void set_send_tag_buffer_u8(uint8_t tag, uint8_t value)
{
    Send_Tag_Buffer[Send_Tag_Length++] = tag;
    Send_Tag_Buffer[Send_Tag_Length++] = 1;
    Send_Tag_Buffer[Send_Tag_Length++] = value;
}

/*
    填充sendbuffer
*/
void set_send_tag_buffer_u16(uint8_t tag, uint16_t value)
{
    Send_Tag_Buffer[Send_Tag_Length++] = tag;
    Send_Tag_Buffer[Send_Tag_Length++] = 2;
    // Send_Tag_Buffer[Send_Tag_Length++] = *value;
    // Send_Tag_Buffer[Send_Tag_Length++] = *(value + 1);
    Send_Tag_Buffer[Send_Tag_Length++] = (value >> 8 * 0) & 0xFF;
    Send_Tag_Buffer[Send_Tag_Length++] = (value >> 8 * 1) & 0xFF;
}

/*
    填充sendbuffer
*/
void set_send_tag_buffer_u32(uint8_t tag, uint32_t value)
{
    Send_Tag_Buffer[Send_Tag_Length++] = tag;
    Send_Tag_Buffer[Send_Tag_Length++] = 4;
    // Send_Tag_Buffer[Send_Tag_Length++] = *value;
    // Send_Tag_Buffer[Send_Tag_Length++] = *(value + 1);
    // Send_Tag_Buffer[Send_Tag_Length++] = *(value + 2);
    // Send_Tag_Buffer[Send_Tag_Length++] = *(value + 3);
    Send_Tag_Buffer[Send_Tag_Length++] = (value >> 8 * 0) & 0xFF;
    Send_Tag_Buffer[Send_Tag_Length++] = (value >> 8 * 1) & 0xFF;
    Send_Tag_Buffer[Send_Tag_Length++] = (value >> 8 * 2) & 0xFF;
    Send_Tag_Buffer[Send_Tag_Length++] = (value >> 8 * 3) & 0xFF;
}

uint8_t tagread_u8()
{
    return Receive_Tag_Buffer[Receive_Tag_Index++];
}

uint16_t tagread_u16()
{
    uint16_t t = tagread_u8();
    t |= (uint16_t)tagread_u8() << 8;
    return t;
}

uint32_t tagread_u32()
{
    uint32_t t = tagread_u16();
    t |= (uint32_t)tagread_u16() << 16;
    return t;
}