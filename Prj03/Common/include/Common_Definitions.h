// TI File $Revision: /main/8 $
// Checkin $Date: November 16, 2011   10:42:30 $
//###########################################################################
//
// FILE:   Common_Definitions.h
//
// TITLE:  DSP280x Common Definitions.
//
//###########################################################################
// $TI Release: DSP280x Header Files V1.60 $
// $Release Date:  November 16, 2011   10:42:30 $
//###########################################################################
// ECCTL1 ( ECAP Control Reg 1)
//==========================
// CAPxPOL bits
#define EC_RISING 0x0

#define EC_FALLING 0x1
// CTRRSTx bits
#define EC_ABS_MODE 0x0
#define EC_DELTA_MODE 0x1
// PRESCALE bits
#define EC_BYPASS 0x0
#define EC_DIV1 0x0
#define EC_DIV2 0x1
#define EC_DIV4 0x2
#define EC_DIV6 0x3
#define EC_DIV8 0x4
#define EC_DIV10 0x5
// ECCTL2 ( ECAP Control Reg 2)
//==========================
// CONT/ONESHOT bit
#define EC_CONTINUOUS 0x0
#define EC_ONESHOT 0x1
// STOPVALUE bit
#define EC_EVENT1 0x0
#define EC_EVENT2 0x1
#define EC_EVENT3 0x2
#define EC_EVENT4 0x3
// RE-ARM bit
#define EC_ARM 0x1
// TSCTRSTOP bit
#define EC_FREEZE 0x0
#define EC_RUN 0x1

// SYNCO_SEL bit
#define EC_SYNCIN 0x0
#define EC_CTR_PRD 0x1
#define EC_SYNCO_DIS 0x2
// CAP/APWM mode bit
#define EC_CAP_MODE 0x0
#define EC_APWM_MODE 0x1
// APWMPOL bit
#define EC_ACTV_HI 0x0
#define EC_ACTV_LO 0x1
// Generic
#define EC_DISABLE 0x0
#define EC_ENABLE 0x1
#define EC_FORCE 0x1


// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  2000  // Period register, 20KHz
#define EPWM1_MAX_CMPA     1000
#define EPWM1_MIN_CMPA     1000
#define EPWM1_MAX_CMPB     1000
#define EPWM1_MIN_CMPB     1000

#define EPWM2_TIMER_TBPRD  2000  // Period register
#define EPWM2_MAX_CMPA     1000
#define EPWM2_MIN_CMPA     1000
#define EPWM2_MAX_CMPB     1000
#define EPWM2_MIN_CMPB     1000

#define EPWM3_TIMER_TBPRD  2000  // Period register
#define EPWM3_MAX_CMPA     1000
#define EPWM3_MIN_CMPA     1000
#define EPWM3_MAX_CMPB     1000
#define EPWM3_MIN_CMPB     1000

#define EPWM4_TIMER_TBPRD  2500  // Period register
#define EPWM4_MAX_CMPA     1250
#define EPWM4_MIN_CMPA     1250
#define EPWM4_MAX_CMPB     1250
#define EPWM4_MIN_CMPB     1250

// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

#define TIME_STARTUP 0x01
#define TIME_SYN 0x02


#define Time_TxA_Offset 5

typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;   
}EPWM_INFO;

#define CPU_FREQ    200E6
#define LSPCLK_FREQ CPU_FREQ / 2
#define SCIA_FREQ   38400
#define SCIA_PRD    ( LSPCLK_FREQ / ( SCIA_FREQ * 8 ) ) - 1

#define SCIB_FREQ   10000000
#define SCIB_PRD    ( LSPCLK_FREQ / ( SCIA_FREQ * 8 ) ) - 1


#define pi 3.14159265358979323846


//SCI Communication frame structure
struct TX_FRAME_Integrated_Control  // #1�ſ�����SCIA���ۿػ������ۿ�֡
{
    Uint16 SYNC[2];
    Uint16 LONGTH;
    Uint16 TXCNT[4];
    Uint16 RXCNT[4];
    Uint16 STATE[2];
    Uint16 FUNC[2];
    Uint16 DATA[20];
    Uint16 CSUM;

    // #1�ſ�����SCIA�����ۿ�֡36�ֽ�
};
extern volatile struct TX_FRAME_Integrated_Control    TX_FRAME_1SCIA;   // ��#1ʹ��

extern volatile struct RX_FRAME_Integrated_Control_12SCIA           RX_FRAME_12SCIA;  // #1��2ʹ��

struct TX_BUF
{
    unsigned char buf[64];
    unsigned int len;

};

#define RXBUF_MASK 0xFF

struct RX_BUF
{
    unsigned char rx_buf[RXBUF_MASK + 1];
    unsigned int rx_head;
    unsigned int rx_tail;     // Data of frame

};

extern volatile struct TX_BUF TX_A;
extern volatile struct TX_BUF TX_C;
extern volatile struct TX_BUF TX_B;
extern volatile struct TX_BUF TX_D;
extern volatile struct RX_BUF Rx_A, Rx_C;
extern volatile struct RX_BUF Rx_B, Rx_D;



struct  TxA_D9
{
    unsigned char ID;
    unsigned char gro_code;  //�����
    unsigned char pos_hub_mot;  //��챵����λ��
    unsigned char speed_hub_mot;
    unsigned char V_hub_mot;
    unsigned char I_hub_mot;
    unsigned char Tem_hub_mot;
    unsigned char stat_hub_mot;
    unsigned char pos_brake;
    unsigned char stat_brake;
    unsigned char pos_sus_mot; //���ҵ����λ��
    unsigned char speed_sus_mot;
    unsigned char pressure_sus_mot;
    unsigned char V_sus_mot;
    unsigned char I_sus_mot;
    unsigned char Tem_sus_mot;
    unsigned char stat_sus_mot;
};

struct DATA_TxA
{
/*    unsigned char FRAME_ID;
    unsigned char Eng_Mode;
    unsigned char Car_Mode;
    unsigned char Drive_Mode;
    unsigned char Yaw_Mode;
    unsigned int frame_cnt;
    struct  TxA_C3 C3;
    struct  TxA_C5 C5;
    struct  TxA_C6 C6;
    struct  TxA_CA CA;
    struct  TxA_D5 D5;
    struct  TxA_D6 D6;
    struct  TxA_D9 D9[8];
*/
    unsigned char FRAME_ID;
    unsigned int frame_cnt;
    unsigned long long Time_stamp;
    int64 lon;
    int64 lat;
    long alt;
    int course;
    int pitch;
    int roll;
    int vel_N;
    int vel_E;
    int vel_sky;
    long Gypo_x;
    long Gypo_y;
    long Gypo_z;
    long acc_x;
    long acc_y;
    long acc_z;
    unsigned char stat;
    unsigned char mode_change;

};
//
//-------------------------------�������ڵ�֮��ͨ�ŵ����ݽṹ---------------------------------
//

struct DATA_C3
{
    unsigned char Driv_instruction[3];
    signed int Speed_instruction[2];
    unsigned int steer_instruction[2];
    unsigned char attack_instruction[3];
    signed int target_X;
    signed int target_Y;
    unsigned int target_delte_X;
    unsigned int target_delte_Y;
};

struct DATA_A5
{
    long local_instruction;
    long before_fbk;
    long acc_X;
    long acc_Y;
    long acc_Z;
    long omega_X;
    long omega_Y;
    long omega_Z;
};

struct DATA_A6
{
    long lon;
    long lat;
    long alt;
    long course;
    long pitch;
    long roll;
    long speed_sample_X;
    long speed_sample_Y;
    long speed_sample_Z;

};

struct DATA_9C
{
    int radar_data[12];
    int milemeter_sig;

};

//���֡�Ĺ�������60 63 65 66 69 6A 6C 6F   ��Ӧ�������
//           30 33 35 36 39 3A 3C 3F  ��Ӧ��������
struct DATA_motor_brake
{
    int pos_hub_mot;            //hub_mot������챵��
    int speed_hub_mot;
    int vol_hub_mot;
    int cur_hub_mot;
    int Tem_hub_mot;
    int pos_brake;
//    int pos_fbk_brake;
    int pos_fbk_sus;             //sus ��������
    int speed_sus;
    int pressure_sus;
    int vol_sus;
    int cur_sus;
    int Tem_sus;
    unsigned char stat_hub_mot;
    unsigned char stat_brake;
    unsigned char stat_sus;

};

struct DATA_5A
{
    unsigned int oil_tank;
    int speed_Eng;
    int Tem_Eng;
    int tor_Eng;                //ת��
    int speed_Gen;
    int vol_Gen;
    int cur_Gen;
    int Tem_Gen;
    unsigned char stat_Gen;
    int SOC_bat_1;
    int vol_bat_1;
    int cur_bat_1;
    int Tem_bat_1;
    unsigned char stat_bat_1;
    int SOC_bat_2;
    int vol_bat_2;
    int cur_bat_2;
    int Tem_bat_2;
    unsigned char stat_bat_2;

};

struct DATA_3C
{
    int pos_fbk;
    int speed;
    int vol;
    int cur;
    int Tem;
    unsigned char stat;
};

struct Com_DATA_Node
{
    unsigned char frame_ID;
    unsigned char address_ID;
    unsigned int frame_cnt;
//    unsigned int local_time;
    unsigned char Timer_100us;
    unsigned char Tx_TimeCNT;
    unsigned char  TimeSyn_Status;
    struct DATA_C3 C3;
    struct DATA_A5 A5;
    struct DATA_A6 A6;
    struct DATA_9C D_9C;
    struct DATA_motor_brake motor_brake[8];
    struct DATA_5A D_5A;
    struct DATA_3C D_3C;

};
//
//-------------------------�ڵ��ͨ�����ݽṹ������--------------------------------
//

//
//------------------------�ڵ�֮��ͨ�ŵģ��洢��Ҫ�õ�����-------------------------------
//  ȥ��ID��cnt��local_time

struct Var_C3
{
    unsigned char Driv_instruction[2];
    unsigned char Driv_mode_instruction;
    float Speed_instruction[2];
    float steer_instruction[2];
    unsigned char Ctrl_hub_instruction;
    unsigned char attack_instruction[2];
    unsigned char makeup_Control_Mode_sel;
    unsigned char OB_Mode_sel;
    unsigned char Cannon_Control_Mode;
    float Cannon_yaw;
    float Cannon_pitch;
    unsigned int sus_pos_cmd;
    Uint16 sus_ctrl_cmd;
    unsigned char underpan_set;
    int underpan_pitch;
    int underpan_roll;
    unsigned int vol_speed_delta_eng;
    float Eng_speed_cmd;
    unsigned char eng_cmd;
    Uint16 bat_stat_cmd:8;
    Uint16 bat_en_cmd:8;

};

struct Var_C9
{
    Uint16 fire_syn_cmd1;
    Uint16 fire_syn_cmd2;
    Uint16 fire_syn_cmd3;
    Uint16 fire_syn_cmd4;
    Uint16 fire_syn_cmd5;
    Uint16 fire_syn_cmd6;
    Uint16 fire_syn_cmd7;
    Uint16 fire_syn_cmd8;
};

struct Var_C5
{
    int32 acc_X;//���ٶ�X
    int32 acc_Y;//���ٶ�Y
    int32 acc_Z;//���ٶ�Z
    int32 lon;//����
    int32 lat;//γ��
    int32 alt;//�߶�
};

struct Var_C6
{
    int32 omega_X;//���ٶ�X
    int32 omega_Y;//���ٶ�Y
    int32 omega_Z;//���ٶ�Z
    int32 course;//ƫ����
    int32 pitch;//��ת��
    int32 roll;//������
    int32 speed_sample_X;//�ٶ�X
    int32 speed_sample_Y;//�ٶ�Y
    int32 speed_sample_Z;//�ٶ�Z

};

struct Var_A5
{
    float cannon_yaw;
    float cannon_pitch;
    float cannon_yaw_speed;
    float cannon_pitch_speed;
    Uint16 fire_cmd:8;
    Uint16 warnning:8;
    Uint16 shell_fir_local:8;
    Uint16 shell_quantity:8;
    Uint16 auto_angle:8;
    Uint16 shell_num:8;
    unsigned int fire_fault;
    Uint16 cannon_electrical1:8;
    Uint16 cannon_electrical2:8;
    Uint16 fire_control_stat:8;
    Uint16 fire_Control_Mode:8;//��������ģʽ
//        Uint16  Cannon_set0:8;//����΢��/���
    int16  Cannon_set1:8;//����΢��/�ߵ���
    int16  Cannon_set2:8;//����΢��/�ߵ���
    Uint16 dir_left:8;//��λ�����
    Uint16 dir_right:8;//��λ�����
    Uint16 pitch_ad:8;//�ߵ�΢��
    Uint16 yaw_ad:8;//��λ΢��
//    unsigned char fire_cmd;
//    unsigned char warnning;
//    unsigned char shell_fir_local;
//    unsigned char shell_quantity;
//    unsigned char auto_angle;
//    unsigned char shell_num;
//    unsigned int fire_fault;
//    unsigned char cannon_electrical1;
//    unsigned char cannon_electrical2;
//    unsigned char fire_control_stat;
//    long double target_lon;
//    long double target_lat;
//    float target_alt;
//    float target_v_N;
//    float target_v_E;
//    float target_v_G;
};

struct Var_9C
{
    float radar_data[12];
    float milemeter_sig;

};

//���֡�Ĺ�������60 63 65 66 69 6A 6C 6F   ��Ӧ�������
//
struct Var_motor_brake
{
	unsigned int pos_hub_mot;            //hub_mot������챵��
    int speed_hub_mot;
    int vol_hub_mot;
    int cur_hub_mot;
    int Tem_hub_mot;
    unsigned int pressure_brake;
    unsigned int pos_fbk_sus;             //sus ��������
    unsigned int speed_sus;
    unsigned int pressure_sus;
    int vol_sus;
    int cur_sus;
    int Tem_sus;
    unsigned int stat_hub_mot:8;
    unsigned int stat_brake:8;
    unsigned int stat_sus:8;
    unsigned int pres_brake_cmd:8;
    int speed_hub_cmd;
    int M_hub_cmd;
    unsigned int pos_sus_cmd;
};

struct Var_5A
{
    int oil;//��������
    int speed_dynamo;//�����ת��
    int Tem_Eng;//�������¶�
    Uint16 FVPCC;//ǰ��Ԥ�����
    Uint16 RVPCC;//��Ԥ�����
    int I_bus;//ĸ�ߵ���
    int V_bus;//ĸ�ߵ�ѹ
    int Tem_Gen;//������¶�
    unsigned char alarm_Eng1:8;//�����������Ϣ1
    unsigned char alarm_Eng2:8;//�����������Ϣ2
    unsigned char alarm_Eng3:8;//�����������Ϣ3
    Uint16 Rectifier;//�����������ʹ��״̬����������ˮ��ʹ��״̬
    int SOC_bat_1;//1�ŵ�������SOC
    int V_bat_1;//1�ŵ�ذ���ѹ
    int I_bat_1;//1�ŵ����ŵ����
    unsigned char Tem_H_bat_1:8;//1�ŵ��������¶�
    unsigned char Tem_L_bat_1:8;//1�ŵ��������¶�
    int SOC_bat_2;//2�ŵ�������SOC
    int V_bat_2;//2�ŵ�����ѹ
    int I_bat_2;//2�ŵ����ŵ����
    unsigned char Tem_H_bat_2:8;//2�ŵ��������¶�
    unsigned char Tem_L_bat_2:8;//2�ŵ��������¶�
    unsigned char Tem_A_bat_2:8;//2�ŵ����ƽ���¶�
    unsigned char Tem_A_bat_1:8;//1�ŵ����ƽ���¶�

};

struct Var_7A
{
    Uint16 fcar_AV_over_warnning:8;    //ǰ����ѹ���߱���
    Uint16 fcar_AV_low_warnning:8;     //ǰ����ѹ���ͱ���
    Uint16 fcar_SV_over_warnning:8;    //ǰ�������ѹ����
    Uint16 fcar_SV_low_warnning:8;     //ǰ������Ƿѹ����
    Uint16 fcar_tem_over_warnning:8;   //ǰ���¶ȹ��߱���
    Uint16 fcar_tem_low_warnning:8;    //ǰ���¶ȹ��ͱ���
    Uint16 fcar_charge_over_warnning:8;//ǰ��������
    Uint16 bcar_AV_over_warnning:8;    //����ѹ���߱���
    Uint16 bcar_AV_low_warnning:8;     //����ѹ���ͱ���
    Uint16 bcar_SV_over_warnning:8;    //�󳵵����ѹ����
    Uint16 bcar_SV_low_warnning:8;     //�󳵵���Ƿѹ����
    Uint16 bcar_tem_over_warnning:8;   //���¶ȹ��߱���
    Uint16 bcar_tem_low_warnning:8;    //���¶ȹ��ͱ���
    Uint16 bcar_charge_over_warnning:8;//ǰ��������
    int charge_A_bat_1;                //1�ŵ���������
    int H_SA_bat_1;                    //1�ŵ������ߵ����ѹ
    int L_SA_bat_1;                    //1�ŵ������͵����ѹ
    int A_SA_bat_1;                    //1�ŵ����ƽ�������ѹ
    int charge_A_bat_2;                //2�ŵ���������
    int H_SA_bat_2;                    //2�ŵ������ߵ����ѹ
    int L_SA_bat_2;                    //2�ŵ������͵����ѹ
    int A_SA_bat_2;                    //2�ŵ����ƽ�������ѹ
    Uint16 fcar_charge_stat:8;         //ǰ�����״̬
    Uint16 bcar_charge_stat:8;         //�󳵳��״̬
    Uint16 FVOutputVol;                //ǰ��������ص�ѹ
    Uint16 RVOutputVol;                //��������ص�ѹ
};
struct Var_9A
{
    int16 PhaseCurrentA;//A�������Ϣ,2�ֽ�
    int16 PhaseCurrentB;//B�������Ϣ,2�ֽ�
    int16 PhaseCurrentC;//C�������Ϣ,2�ֽ�
    int16 PhaseCurrentD;//D�������Ϣ,2�ֽ�
    int16 PhaseCurrentE;//E�������Ϣ,2�ֽ�
    int16 PhaseCurrentF;//F�������Ϣ,2�ֽ�

    int16 Id;           //Id������Ϣ,2�ֽ�
    int16 Iq;           //Iq������Ϣ,2�ֽ�

    Uint16 BusVoltage;  //ĸ�ߵ�ѹ,2�ֽ�
    int16  BusCurrent;  //ĸ�ߵ���,2�ֽ�
    Uint16 PhaseTempA:8;//A���¶�,1�ֽ�
    Uint16 PhaseTempB:8;//B���¶�,1�ֽ�
    Uint16 PhaseTempC:8;//C���¶�,1�ֽ�
    Uint16 PhaseTempD:8;//D���¶�,1�ֽ�
    Uint16 PhaseTempE:8;//E���¶�,1�ֽ�
    Uint16 PhaseTempF:8;//F���¶�,1�ֽ�

    Uint16 ModuleTemp1:8;//ģ��1�¶�,1�ֽ�
    Uint16 ModuleTemp2:8;//ģ��2�¶�,1�ֽ�
    Uint16 ModuleTemp3:8;//ģ��3�¶�,1�ֽ�
    Uint16 ModuleTemp4:8;//ģ��4�¶�,1�ֽ�
    Uint16 ModuleTemp5:8;//ģ��5�¶�,1�ֽ�
    Uint16 ModuleTemp6:8;//ģ��6�¶ȣ�1�ֽ�
    Uint16 alarminfor01:8;//������Ϣ1
    Uint16 alarminfor02:8;//������Ϣ2
    Uint16 alarminfor03:8;//������Ϣ3
};
struct Var_3C
{
    float pos_fbk;//�ڿط�λ����
    float speed_fbk;
    float vol;
    float cur;
    float Tem;
    unsigned char stat;
    int ACur;
    int BCur;
    int CCur;
    int DCur;
    int ECur;
    int FCur;
};

struct Var_3D
{
    float pos_fbk_1;//�ڿظߵͷ���
    float speed_fbk_1;
    float vol_1;
    float cur_1;
    float Tem_1;
    unsigned char stat_1;

};

//����վ����//
struct Var_D0
{
    unsigned char way_num;
    unsigned char waypoint_num;
    long lon;
    long lat;
    unsigned char waypoint_sig;
};

struct Var_D1//�Ѿ����F3��F9
{
    unsigned char sil_forc_cmd;//��Ĭǿ������
    float Cannon_pitch_cmd;//�����ߵͽǶ�/���ٶ�
    float Cannon_yaw_cmd;//������λ�Ƕ�/���ٶ�
    float OB_pitch_cmd;//����ߵͽǶ�/���ٶ�
    float OB_yaw_cmd;//���鷽λ�Ƕ�/���ٶ�
    unsigned char OB_Control_Mode;//�������ģʽ
    unsigned char fire_Control_Mode;//��������ģʽ
    unsigned char makeup_Control;//��װ����ģʽ
    unsigned char Servo_Open_Close;//�ŷ�����
    unsigned char Heat_Open_Close;//���񿪹�
    unsigned char AutomaticRanging;//�Զ���࿪��
    unsigned char Large_Small_View;//��С�ӳ��л�
};

struct Var_D2
{
    unsigned char path_ID;
    unsigned char path_point_ID;

};

struct Var_D8
{
    unsigned int vol_speed_delta_eng;
    unsigned int Eng_speed_cmd;
    unsigned char bat_stat_cmd;
    unsigned char bat_en_cmd;
    unsigned int sus_pos_cmd;
    unsigned char sus_ctrl_cmd;
};

struct Var_F3
{
    unsigned char OB_Control_Mode;//�������ģʽ
    float  OB_pitch_cmd;//����ߵͽǶ�/���ٶ�
    float  OB_yaw_cmd;//���鷽λ�Ƕ�/���ٶ�
    unsigned char OB_cmd;//�������ָ��
    unsigned char makeup_mode;//��װ����ģʽ
    unsigned char FSM_cmd;//FSM��������
};

struct Var_F5
{
    unsigned int target_x0;
    unsigned int target_y0;
    unsigned char target_ID;
    unsigned long Pic_Num;
};

struct Var_F9
{
    unsigned char fire_Control_Mode;//��������ģʽ
    int  Cannon_pitch_cmd;//�����ߵͽǶ�/���ٶ�
    int  Cannon_yaw_cmd;//������λ�Ƕ�/���ٶ�
    unsigned char  Cannon_set0;//����΢��/���
    char  Cannon_set1;//����΢��/�ߵ���
    char  Cannon_set2;//����΢��/�ߵ���
    unsigned char dir_left;//��λ�����
    unsigned char dir_right;//��λ�����
};

//
struct Var_D5
{
    unsigned char atk_cmd[5];
    unsigned char target_ID;
    unsigned char target_stat;

};

//struct Var_E3
//{
////    unsigned char target_ID;
//    long double target_lon;
//    long double target_lat;
//    float target_alt;
////    unsigned char target_stat;
//
//};

struct Var_E4
{
//    unsigned char target_ID;
    float target_v_N;
    float target_v_E;
    float target_v_G;
//    unsigned char target_stat;
};

//��ظ�֪����
struct Var_E5
{
    unsigned char target_ID;
    unsigned char target_stat;
    unsigned char kind;
    unsigned char identi_fri_foe;
    unsigned char threat_level;
    unsigned char destroy_evaluate;
    unsigned char atk_advice;
    unsigned char drive_advice;
    unsigned char target_x0;
    unsigned char target_y0;
    long Pic_Num;
    unsigned char makeup_mes;
    unsigned char makeup_control;
};
struct Var_E3
{
    Uint16 yaw_gyroscope1:8;
    Uint16 yaw_gyroscope2:8;
    Uint16 yaw_gyroscope3:8;
    unsigned char OB_stat_1:8;
    Uint16 pitch_gyroscope1:8;
    Uint16 pitch_gyroscope2:8;
    Uint16 pitch_gyroscope3:8;
    unsigned char OB_stat_2:8;
    unsigned int yaw;
    unsigned int pitch;
    unsigned int laser_range;
    unsigned int yaw_offset;
    unsigned int pitch_offset;
};
struct Var_E6
{
    unsigned char target_ID;
    unsigned char target_hight;//
    long double target_lon;
    long double target_lat;
    float target_alt;//
    float target_v_N;
    float target_v_E;
    float target_v_G;
};

struct Var_IN
{
    unsigned int Timestamp1;
    unsigned long Timestamp2;
    long double lon;
    long double lat;
    long double alt;
    long double course;//Э������4���ֽ�
    long double pitch;
    long double roll;
    long double vel_N;
    long double vel_E;
    long double vel_sky;//Э������4���ֽ�
    long double Gypo_x;
    long double Gypo_y;
    long double Gypo_z;
    long double acc_x;
    long double acc_y;
    long double acc_z;
    unsigned char stat;
    unsigned char mode_change;

};


struct Var_Com_DATA_Node
{
    struct Var_C3 Var_C3;//����������
    struct Var_C9 Var_C9;//����������
    struct Var_C5 Var_C5;//����������
    struct Var_C6 Var_C6;//����������
    struct Var_A5 Var_A5;//����������
    struct Var_9C Var_D_9C;//����������
    struct Var_motor_brake Var_motor_brake[8];//����������
    struct Var_5A Var_D_5A;//����������
    struct Var_7A Var_D_7A;//����������
    struct Var_9A Var_D_9A;//����������
    struct Var_3C Var_D_3C;//����������
    struct Var_3D Var_D_3D;//����������
    struct Var_D0 Var_D0;//����վ����
    struct Var_D1 Var_D1;//����վ����//
    struct Var_D2 Var_D2;//����վ����
    struct Var_D8 Var_D8;//����վ����
    struct Var_F3 Var_F3;//����վ����
    struct Var_F5 Var_F5;//����վ����
    struct Var_F9 Var_F9;//����վ����
    struct Var_E3 Var_E3;//��ؽ���
    struct Var_E5 Var_E5;//��ؽ���
    struct Var_E6 Var_E6;//��ؽ���
    struct Var_IN Var_IN;//�ߵ�����//
};
//
//------------------------------------------------------------------
//

struct RxA_D0
{

    unsigned char way_num;
    unsigned char waypoint_num;
    float lon;
    float lat;
    unsigned char waypoint_sig;    //Э����δ�����������Ͷ���
};

struct RxA_D1
{

    unsigned char sil_forc_cmd;

    float Cannon_pitch_cmd;
    float Cannon_yaw_cmd;
    unsigned char Up_Control_Mode;


};

struct RxA_D2
{

    unsigned char path_ID;
    unsigned char path_point_ID;
};

struct RxA_D4
{

    unsigned char atk_cmd;
    signed int target_x0;
    unsigned int target_y0;
    unsigned int delta_x;
    unsigned int delta_y;
};

struct DATA_RxA
{

    struct RxA_D0 D0;
    struct RxA_D1 D1;
    struct RxA_D2 D2;
    struct RxA_D4 D4;
};









extern volatile struct Com_DATA_Node ComDataNode ;             //�Ĵ棬���壬


#define ENABLE1 GpioDataRegs.GPCCLEAR.bit.GPIO92
#define ENABLE2 GpioDataRegs.GPCCLEAR.bit.GPIO90
#define ENABLE3 GpioDataRegs.GPBCLEAR.bit.GPIO44
#define DISABLE1 GpioDataRegs.GPCSET.bit.GPIO92
#define DISABLE2 GpioDataRegs.GPCSET.bit.GPIO90
#define DISABLE3 GpioDataRegs.GPBSET.bit.GPIO44

//#define DAT1 GpioDataRegs.GPADAT.bit.GPIO15
//#define DAT2 GpioDataRegs.GPADAT.bit.GPIO19
//#define DAT3 GpioDataRegs.GPADAT.bit.GPIO27
//#define DAT4 GpioDataRegs.GPBDAT.bit.GPIO41
//#define DAT5 GpioDataRegs.GPBDAT.bit.GPIO39
//#define DAT6 GpioDataRegs.GPBDAT.bit.GPIO52
//#define DAT7 GpioDataRegs.GPBDAT.bit.GPIO53
//#define DAT8 GpioDataRegs.GPBDAT.bit.GPIO54
//#define DAT9 GpioDataRegs.GPBDAT.bit.GPIO55
//#define DAT10 GpioDataRegs.GPBDAT.bit.GPIO57
//#define DAT11 GpioDataRegs.GPBDAT.bit.GPIO42
//#define DAT12 GpioDataRegs.GPBDAT.bit.GPIO43

//#define DE_B GpioDataRegs.GPADAT.bit.GPIO24
//#define RE_B GpioDataRegs.GPADAT.bit.GPIO25
//#define DE_A GpioDataRegs.GPCDAT.bit.GPIO93
//#define RE_A GpioDataRegs.GPCDAT.bit.GPIO91
#define _BUS_FPGA
//#define _BUS_DSP
#define _DSP_SOM
//#define _DSP_C


#define NODE_MASTER             //�궨��������ͻ��Dͨ��
//#define NODE_GCS
//#define NODE_MASTER_standby_1   //��һ���ýڵ�

#ifdef _DSP_SOM
#define RX_A_EN GPIO_WritePin(7, 0)
#define RX_A_DIS GPIO_WritePin(7, 1)
#define TX_A_DIS GPIO_WritePin(94, 0)
#define TX_A_EN GPIO_WritePin(94, 1)
#define RX_B_EN GPIO_WritePin(168, 0)
#define RX_B_DIS GPIO_WritePin(168, 1)
#define TX_B_DIS GPIO_WritePin(167, 0)
#define TX_B_EN GPIO_WritePin(167, 1)
#define RX_C_EN GPIO_WritePin(67, 0)
#define RX_C_DIS GPIO_WritePin(67, 1)
#define TX_C_DIS GPIO_WritePin(66, 0)
#define TX_C_EN GPIO_WritePin(66, 1)
#define RX_D_EN GPIO_WritePin(65, 0)
#define RX_D_DIS GPIO_WritePin(65, 1)
#define TX_D_DIS GPIO_WritePin(64, 0)
#define TX_D_EN GPIO_WritePin(64, 1)
#endif

#ifdef _DSP_C
#define RX_A_EN GPIO_WritePin(41, 0)
#define RX_A_DIS GPIO_WritePin(41, 1)
#define TX_A_DIS GPIO_WritePin(40, 0)
#define TX_A_EN GPIO_WritePin(40, 1)
#define RX_B_EN GPIO_WritePin(57, 0)
#define RX_B_DIS GPIO_WritePin(57, 1)
#define TX_B_DIS GPIO_WritePin(56, 0)
#define TX_B_EN GPIO_WritePin(56, 1)
#define RX_C_EN GPIO_WritePin(60, 0)
#define RX_C_DIS GPIO_WritePin(60, 1)
#define TX_C_DIS GPIO_WritePin(58, 0)
#define TX_C_EN GPIO_WritePin(58, 1)
#define RX_D_EN GPIO_WritePin(69, 0)
#define RX_D_DIS GPIO_WritePin(69, 1)
#define TX_D_DIS GPIO_WritePin(68, 0)
#define TX_D_EN GPIO_WritePin(68, 1)
#endif


#define WDT_TOG_Right GpioDataRegs.GPCTOGGLE.bit.GPIO66
#define WDT_TOG_Left  GpioDataRegs.GPDTOGGLE.bit.GPIO99


#define xor(a,b) ((~a && b) || (a && ~b))

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);
interrupt void cpu_timer1_isr(void);
interrupt void cpu_timer2_isr(void);
interrupt void xint1_isr(void);
__interrupt void CPU01toCPU02IPC0IntHandler(void);
__interrupt void CPU01toCPU02IPC1IntHandler(void);

Uint32 crc32(unsigned char *, unsigned char );
void SCIA_ReadFIFO(void);
void SCIB_ReadFIFO(void);
void SCIC_ReadFIFO(void);
void SCID_ReadFIFO(void);

void Rx_Proc();
void Tx_Proc(unsigned int ch);
void SCI_RST(void);
void SCIA_TX(void);
void SCIA_RX(void);
void SCIB_TX(void);
void SCIB_RX(void);
void SCIC_TX(void);
void SCIC_RX(void);
void SCID_TX(void);
void SCID_RX(void);

int fsin( long );

extern void delayns(unsigned long num);
extern void setup_emif1_pinmux_async_16bit(Uint16);


//===========================================================================
// End of file.
//===========================================================================

