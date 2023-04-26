#include "u_sys.h"
#include "u_delay.h"
// #include <string.h>
unsigned char rf_data[4];           //���յ����ݣ����4bit��ʾ��ֵ
unsigned char decode_ok;            //����ɹ���־λ
unsigned char RF;
unsigned char hh_w,ll_w;           //��,�͵�ƽ���
unsigned char ma_x;                //���յ��ڼ�λ������
unsigned char bma1,bma2,bma3,bma4; //���ڽ��չ��̴��ң�ر��룬����Ƚ����Σ����ǵ�һ��
unsigned char mma1,mma2,mma3,mma4;
unsigned char mmb1,mmb2,mmb3,mmb4;  // ���ڽ��չ��̴��ң�ر��룬�ڶ���
unsigned char rf_ok1,rf_ok2,rf_ok;  //��������е���ʱ���ճɹ���־,���յ�һ��������ң���������1,֪ͨ���������Խ�����
unsigned char old_rc5;              //������һ�β�ѯ���ĵ�ƽ״̬
unsigned char tb_ok;                //���յ�ͬ������ʱ��1   
unsigned char D0,D1,D2,D3;
unsigned char bt_auto;              //�Զ�����ң�ؽ��ղ����ʱ�־
unsigned short s,s1;
void TIM6_1527_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStruct.GPIO_Pin     = GPIO_Pin_11; 
  GPIO_InitStruct.GPIO_Mode    = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Speed   = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
  // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;//TIM6�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ռ���ȼ�2��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�����ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ����ʹ��
  NVIC_Init(&NVIC_InitStructure);//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//ʹ��TIM6ʱ��
  TIM_TimeBaseStructure.TIM_Period = 100;//�趨�������Զ���װֵ 
  TIM_TimeBaseStructure.TIM_Prescaler =71;//ʱ��Ԥ��Ƶϵ�� CK_CNT=CK_INT/(71+1) = 1M 1��ʱ�����1/CK_CNT = 1us
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);//��������ж� 
  TIM_ClearFlag(TIM6,TIM_FLAG_Update);
  TIM_Cmd(TIM6,ENABLE );//ʹ�ܶ�ʱ��6
}
void TIM6_IRQHandler()
{
  if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update); 
    RF = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11); 
    if (!RF)
    { 
      ll_w++;   // ��⵽�͵�ƽ �͵�ƽʱ���1,��¼���ε�ƽ״̬old_rc5
      old_rc5=0;
    }               
    else         // ��⵽�ߵ�ƽ
    { 
      hh_w++;
      if (!old_rc5)  // ��⵽�ӵ͵��ߵ�����,�Ѽ�⵽һ������(��-��)��ƽ����
      {   
        //��ͬ����    2/5 100/130
        if (((hh_w>=2)&&(hh_w<=5))&&((ll_w>=100)&&(ll_w<=130))) 
        { 
          tb_ok = 1 ;
          ma_x = 0;
          bma1=0; bma2=0; bma3=0; bma4=0;    
        }
        else if ((tb_ok)&&((ll_w>=8)&&(ll_w<=13))) //8/13
        {   
          ma_x++; //�Ѿ����յ�ͬ����,��0
          if (ma_x>23)
          {
            if(!rf_ok1) //rf_ok1��ʱ���ճɹ�
            {
              //�����յ��ı��븴�Ƶ�����Ĵ����� 
              mma1=bma1;
              mma2=bma2;
              mma3=bma3;
              mma4=bma4; 
              // ֪ͨ�����ӳ�����Խ�����                                                        
              rf_ok1=1;                    
              tb_ok=0;
              s=1000;                             
            }
            else
            {
              //�����յ��ı��븴�Ƶ�����Ĵ�����  
              mmb1=bma1;
              mmb2=bma2;
              mmb3=bma3;
              mmb4=bma4;  
              // ֪ͨ�����ӳ�����Խ�����                                             
              rf_ok2=1;                     
              tb_ok=0;                                                                        
            }
          }
        }  
        else if ((tb_ok)&&((ll_w>=2)&&(ll_w<=7)))   // 2/7
        { 
          switch (ma_x)
          {
            case 0 :
              bma1=bma1 | 0x80;//ң�ر����1λ
              break;   
            case 1 :
              bma1=bma1 | 0x40;  
              break;
            case 2 :
              bma1=bma1 | 0x20; 
              break;
            case 3 :
              bma1=bma1 | 0x10; 
              break;
            case 4 :
              bma1=bma1 | 0x08; 
              break;
            case 5 :
              bma1=bma1 | 0x04; 
              break;
            case 6 :
              bma1=bma1 | 0x02; 
              break;
            case 7 :
              bma1=bma1 | 0x01; 
              break;
            case 8 :
              bma2=bma2 | 0x80; 
              break;
            case 9 :
              bma2=bma2 | 0x40; 
              break;
            case 10:
              bma2=bma2 | 0x20; 
              break;
            case 11:
              bma2=bma2 | 0x10; 
              break;
            case 12:
              bma2=bma2 | 0x08; 
              break;
            case 13:
              bma2=bma2 | 0x04; 
              break;
            case 14:
              bma2=bma2 | 0x02; 
              break;
            case 15:
              bma2=bma2 | 0x01; 
              break;
            case 16:
              bma3=bma3 | 0x80; 
              break;
            case 17:
              bma3=bma3 | 0x40; 
              break;
            case 18:
              bma3=bma3 | 0x20; 
              break;
            case 19:
              bma3=bma3 | 0x10; 
              break;
            case 20:
              bma3=bma3 | 0x08;//����״̬��1λ
              break;
            case 21:
              bma3=bma3 | 0x04; 
              break;
            case 22:
              bma3=bma3 | 0x02; 
              break;
            case 23:
              bma3=bma3 | 0x01;              
              if(!rf_ok1)
              {
                mma1=bma1;
                mma2=bma2;
                mma3=bma3;
                //mma4=bma4;                                           // �����յ��ı��븴�Ƶ�����Ĵ�����                             
                rf_ok1=1;         // ֪ͨ�����ӳ�����Խ�����
                tb_ok=0;
                // bt_auto=0;
                s=1000;
              }
              else
              {
                mmb1=bma1;
                mmb2=bma2;
                mmb3=bma3;
                //mmb4=bma4;               // ���ٴν��յ��ı��븴�Ƶ�����Ĵ����У�                             
                rf_ok2=1;                                      // ֪ͨ�����ӳ�����Խ�����
                tb_ok=0;
              }
              break;
          } 
          ma_x++;
        }
        else
        {
          ma_x=0; 
          tb_ok=0;
          bt_auto=0;
          bma1=0;
          bma2=0; 
          bma3=0; 
          hh_w=1;
          ll_w=0;
        }                                      //���յ������ϵĸ�-�͵�ƽ����
        ll_w=0;
        hh_w=1; 
      }          
      old_rc5=1;      // ��¼���ε�ƽ״̬
    }
    if(rf_ok1)  //�涨ʱ���ڽ��ܵ�2֡��ͬ�ı������ݲ���Ч
    {
      s--;
      if(!s)
      {
        rf_ok1=0;
      }
      if(rf_ok2) 
      {
        if((mma1==mmb1)&&(mma2==mmb2)&&(mma3==mmb3))
        {
          rf_ok=1;
          rf_ok1=0;
          rf_ok2=0;                    
        }
        else
        {
          rf_ok=0;
          rf_ok1=0;
          rf_ok2=0;
        }
      }                   
    }
    if((rf_ok))      //�ж��Ƿ���ճɹ�
    {   
      TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);
      rf_ok=0; 
      rf_data[0]=mma1;
      rf_data[1]=mma2;
      rf_data[2]=mma3;//A-0x61 B-0x62
      decode_ok=1;
      TIM_ITConfig(TIM6  , TIM_IT_Update, ENABLE);
    }
  }
}
int main(void)
{
  delay_init();	    	                            //��ʱ������ʼ��
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
  TIM6_1527_Init();
  while(1)
  {

  }
}
