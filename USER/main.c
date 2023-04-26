#include "u_sys.h"
#include "u_delay.h"
// #include <string.h>
unsigned char rf_data[4];           //接收的数据，最后4bit表示键值
unsigned char decode_ok;            //解码成功标志位
unsigned char RF;
unsigned char hh_w,ll_w;           //高,低电平宽度
unsigned char ma_x;                //接收到第几位编码了
unsigned char bma1,bma2,bma3,bma4; //用于接收过程存放遥控编码，编码比较两次，这是第一次
unsigned char mma1,mma2,mma3,mma4;
unsigned char mmb1,mmb2,mmb3,mmb4;  // 用于接收过程存放遥控编码，第二次
unsigned char rf_ok1,rf_ok2,rf_ok;  //解码过程中的临时接收成功标志,接收到一个完整的遥控命令后置1,通知解码程序可以解码了
unsigned char old_rc5;              //保存上一次查询到的电平状态
unsigned char tb_ok;                //接收到同步的马时置1   
unsigned char D0,D1,D2,D3;
unsigned char bt_auto;              //自动设置遥控接收波特率标志
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
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;//TIM6中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//先占优先级2级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//从优先级0级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道被使能
  NVIC_Init(&NVIC_InitStructure);//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//使能TIM6时钟
  TIM_TimeBaseStructure.TIM_Period = 100;//设定计数器自动重装值 
  TIM_TimeBaseStructure.TIM_Prescaler =71;//时钟预分频系数 CK_CNT=CK_INT/(71+1) = 1M 1次时间等于1/CK_CNT = 1us
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//设置时钟分割:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);//根据指定的参数初始化TIMx的时间基数单位
  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);//允许更新中断 
  TIM_ClearFlag(TIM6,TIM_FLAG_Update);
  TIM_Cmd(TIM6,ENABLE );//使能定时器6
}
void TIM6_IRQHandler()
{
  if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update); 
    RF = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11); 
    if (!RF)
    { 
      ll_w++;   // 检测到低电平 低电平时间加1,记录本次电平状态old_rc5
      old_rc5=0;
    }               
    else         // 检测到高电平
    { 
      hh_w++;
      if (!old_rc5)  // 检测到从低到高的跳变,已检测到一个完整(高-低)电平周期
      {   
        //判同步码    2/5 100/130
        if (((hh_w>=2)&&(hh_w<=5))&&((ll_w>=100)&&(ll_w<=130))) 
        { 
          tb_ok = 1 ;
          ma_x = 0;
          bma1=0; bma2=0; bma3=0; bma4=0;    
        }
        else if ((tb_ok)&&((ll_w>=8)&&(ll_w<=13))) //8/13
        {   
          ma_x++; //已经接收到同步码,判0
          if (ma_x>23)
          {
            if(!rf_ok1) //rf_ok1临时接收成功
            {
              //将接收到的编码复制到解码寄存器中 
              mma1=bma1;
              mma2=bma2;
              mma3=bma3;
              mma4=bma4; 
              // 通知解码子程序可以解码了                                                        
              rf_ok1=1;                    
              tb_ok=0;
              s=1000;                             
            }
            else
            {
              //将接收到的编码复制到解码寄存器中  
              mmb1=bma1;
              mmb2=bma2;
              mmb3=bma3;
              mmb4=bma4;  
              // 通知解码子程序可以解码了                                             
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
              bma1=bma1 | 0x80;//遥控编码第1位
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
              bma3=bma3 | 0x08;//按键状态第1位
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
                //mma4=bma4;                                           // 将接收到的编码复制到解码寄存器中                             
                rf_ok1=1;         // 通知解码子程序可以解码了
                tb_ok=0;
                // bt_auto=0;
                s=1000;
              }
              else
              {
                mmb1=bma1;
                mmb2=bma2;
                mmb3=bma3;
                //mmb4=bma4;               // 将再次接收到的编码复制到解码寄存器中，                             
                rf_ok2=1;                                      // 通知解码子程序可以解码了
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
        }                                      //接收到不符合的高-低电平序列
        ll_w=0;
        hh_w=1; 
      }          
      old_rc5=1;      // 记录本次电平状态
    }
    if(rf_ok1)  //规定时间内接受到2帧相同的编码数据才有效
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
    if((rf_ok))      //判断是否接收成功
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
  delay_init();	    	                            //延时函数初始化
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
  TIM6_1527_Init();
  while(1)
  {

  }
}
