/**********************************
      
**********************************/
#ifndef __FOC_DEFINE_PARAMETER_H_
#define __FOC_DEFINE_PARAMETER_H_


#define MOTOR_STARTUP_CURRENT   1.0f   //������������������Լ�ʵ�ʸ������� 
#define SPEED_LOOP_CLOSE_RAD_S  50.0f  //�ٶȻ�����ջ����ٶ�  ��λ: rad/s

//�и�FOC �� �޸�FOCѡ���ܵ�ע�͵�����һ��
//#define HALL_FOC_SELECT          //����ע�͵��Ͳ�ʹ���и�FOC����
#define SENSORLESS_FOC_SELECT    //����ע�͵��Ͳ�ʹ���޸и�FOC����


//����������ã����裬��У�������
#define RS_PARAMETER     0.59f           //����
#define LS_PARAMETER     0.001f          //���
#define FLUX_PARAMETER   0.01150f        //����

#endif
