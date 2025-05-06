
// #include "task.h"


#define yaw_error_allow 50 //允许yaw误差5度

/*************************以下是关于构建地图的方法***************************/
#define yunxvwucha 10    						//允许误差    单位cm 
int gogo[100], g_t;      						//最短路径，最终顺序取出，指针g_t为路径的长度 ,保存路径长度
int new_point, all_point, p_point; 	//新点，所有点数量，在遍历循环判断是否重复时的当前点（另一个局部使用的循环变量）
int i_t, n_t,  nomin;  							//循环变量（用于初始化所有的节点），循环变量，最小值
int p_t; 														//当前点的标号
int direction_t=2;										//保存下一步移动的方向1：左，2：前，3：右，4：后（绝对方向，与绘制地图的方向
float distance_t, distance_min_t;   //路径长度，最小路径长度

struct map													//地图结构体
{
    int x;														//x坐标
    int y;														//y坐标
    uint8_t qian;											//前方是否有路
    uint8_t hou;											//后是否有路
    uint8_t left;											//左是否有路
    uint8_t right;										//右方是否有路 
    uint8_t qian_view;
    uint8_t hou_view;
    uint8_t left_view;
    uint8_t right_view;
    uint8_t node_num;									//分路数量
    uint8_t no;												//编号
    struct map *next_qian;						//下一个前方的点
    struct map *next_hou;							//下一个后方的点
    struct map *next_left;						//下一个左方的点
    struct map *next_right;						//下一个右方的点
    struct map *next;									//下一个点
//	struct map *pervious;
};
struct map point[100]; 							//存放地图节点数组
struct map save;       							//存放当前点信息
struct map *go, *now, *last, *now2, *head, *hp, *tail; //下一个点，当前点，上一个点，当前点，头节点，当前点，尾节点

//***硬件相关变量/标志***//
extern  PARA_Moto PARA_MOTO;  			//电机参数
extern short Target_Velocity_L,Target_Velocity_R;  //左右电机目标速度
uint8_t START_FLAG=0;								//开始标志
uint32_t Read_pin, Read_pin_pre;	//
uint8_t Key_count;									//按键计数器
uint8_t STATE = 0,STATE2= 0;									//task函数state状态，初始状态为0
int find_exit2;                     //找到终点的标志位 
extern uint8_t HCSR04_WORK_FLAG;		//超声波正在工作标志
extern uint8_t HC_SR04_ALLOW_FLAG;	//超声波允许工作标志
extern short INTERRUPT_FLAG,INTERRUPT_FLAG2;	//20ms中断控制部分工作位
int delay_count;  									//识别路口后延时计数

//***地图相关变量/标志***//
uint8_t cflag_right=0,cflag_left=0,cflag_tn180=0,cflag_stop,cflag_up;		    //右左转标志，掉头，停止
uint8_t cflag_count=0;                      //??此处作用未知
uint8_t cflag_turn =1 ;                      //允许转弯标志
extern uint8_t cflag_right,cflag_left;								//右左转标志
extern int LocX,LocY,last_locx,last_locy,RE_LocX,RE_LocY;             //位置参数 初始点为基准点，x轴距离代的距离参数   //用于记录临Locx和LocY  
extern uint8_t direct;   
int yaw_record; //记录yaw值
int yaw_target; //设定目标yaw值，用于转弯
extern int yaw_adjust ;
extern int yaw_int;
//uint8_t dir; //方向
extern uint8_t RECORED_DOWN;  //节点记录完成标志
extern uint8_t END_FLAG;			//终点标志
int ab_direction;  //当前节点行进方向
extern uint8_t NODE_DETECT_FLAG,TURN_RIGHT_FLAG,TURN_LEFT_FLAG ,BOTH_FLAG,TURN_UP_FLAG,TURN_BACK_FLAG;
uint8_t Start_Command;    //调试命令可从中出改变
uint8_t nining_flag=0,rain_flag=0,ban_flag = 0;  //标识头识别到泥路等
int node2_list[50];    //task2的转弯数组
int node2_num=0;
uint8_t blue_node_num = 0;
//...函数...//
void task1(void){  //测试游走，记录

    if(send_data_flag){
                send_data_flag = 0;
                ANO_Direct_distance(direct,blue_node_num);
        }

    switch (STATE){
    case 0:{  //初始化
        Read_pin = 0;
        Read_pin_pre = 0;
        Key_count = 0;
        PARA_MOTO.TVL = 0;
        PARA_MOTO.TVR = 0;
        INTERRUPT_FLAG = 1;
        cflag_turn = 1;
        STATE ++;
        break;}
    
    case 1:{ //等待启动、转状态
        if(START_FLAG == 1)
        {
            STATE ++;
        }
        if( DL_GPIO_readPins(KEY_Group_PORT,KEY_Group_P21_PIN) > 0)
        {
            Read_pin = 0;
        }else{
            Read_pin = 1;
        }
        if (Read_pin==1 && Read_pin_pre==1)  //改成用宏开始，或Start_Command 可能要写一个校准，以是常量                           
        {
            Key_count += 1;
            if(Key_count == 3)
            {	
                START_FLAG = 1;
                Key_count = 0;
                Read_pin = 0;
                Read_pin_pre = 0;
                STATE++;
                break;
            }
        }
        Read_pin_pre = Read_pin;
        break;}
        
        
    case 2:{  //收到开始开始行走状态
        if(START_FLAG == 1)            
        {	
            blue_node_num++;
            START_FLAG =0 ;
            yaw_adjust = yaw_int;  //记录初始角度
            start_point();
            PARA_MOTO.TVL = 30;  
            PARA_MOTO.TVR = 30;
            PID_init();
            cflag_turn = 0;
            Set_Target_Veclocity(PARA_MOTO.TVL, PARA_MOTO.TVR);
            //start_point();
            OLED_ShowString(0,32,(uint8_t *)"start",8,1);
            OLED_Refresh();
            delay_ms(100);
            INTERRUPT_FLAG = 0;
            HC_SR04_ALLOW_FLAG = 0;
            delay_count = 0;
            STATE++;
        }
        break;}
                
    case 3:{  //循环状态 1:检测障碍物 2：识别终点  3：识别到黑复合道路 
            if(HCSR04_WORK_FLAG == 0)  //避障器
            {                           
                HCSR04_WORK_FLAG =1;
                distance = Hcsr04GetLength();
                if(distance < 30)
                {
                    INTERRUPT_FLAG = 1;
                    distance = Hcsr04GetLength();
                    Set_Pwm(0,0);
                }
                else{
                INTERRUPT_FLAG = 0;
                }
            }
            
            if(nining_flag){
            INTERRUPT_FLAG2 = 1;
            Set_Pwm(1000,1000);
//		Set_Target_Veclocity(20,20);
//		clear_pid(&Motor_velocity_L)  ;
//		clear_pid(&Motor_velocity_R)  ;
//		pid_init(&Motor_velocity_L, 90, 4, 16); 
//		pid_init(&Motor_velocity_R, 90, 4, 16);
    }
            
            if(rain_flag){	
        INTERRUPT_FLAG = 1;
        cflag_turn = 1;
        Set_Pwm(0,0);
        while(rain_flag)
        {
                buzzer_turn_on_delay(10);
                delay_ms(100);
        }
        INTERRUPT_FLAG = 0;
        cflag_turn = 0;
    }
            //printf("%.2f\r\n",distance);
            //INFRARED_cal_error();
            if(ban_flag){
            INTERRUPT_FLAG = 1;
            cflag_turn=1;
            Set_Pwm(0,0);
            while(ban_flag){
                buzzer_turn_on_delay(50);
                delay_ms(50);
            }
            cflag_turn = 0;
            INTERRUPT_FLAG = 0;
        }	
        
            if(NODE_DETECT_FLAG){	
                            
                            NODE_DETECT_FLAG = 0;
                            buzzer_turn_on_delay(10);
                            INTERRUPT_FLAG = 1;
                            cflag_turn = 1;  //暂停巡黑边
                            INFRARED_ERROR = 0;
                            Pwm_diff_zero();
                            delay_ms(100);    		//往前走走0.5s  //可能增加一个延时计数，定时器使用
                            Locationhold();				//更新方向与节点参数
        //				Turn_back();//
        //				break;
                            if(STOP_FLAG){
                                STATE++;
                                break;
                            }
                            int levels = read_infrared_levels();
                            if(levels&0b01110){   					//此时levels不为零，说明前方有路
                                    TURN_UP_FLAG =1;  //允许前进
                                    buzzer_turn_on_delay(10);
                                    STATE ++;
                            }
                            else{
                                    TURN_UP_FLAG = 0;
                                    STATE ++;
                            }
                    }	
                break;}
    
    case 4:{ //循环接收节点，记录并执行转弯	
            printf("state4\r\n");	
            NODE_DETECT_FLAG = 0; //清除收节点标志
            blue_node_num++;
            if(STOP_FLAG == 1){
                    find_exit2 = 1;
                    INTERRUPT_FLAG = 1;
                    Set_Pwm(0,0);
                    STATE =5;
                    Clear_NODEflag();//清除左右方向
            }
            //judge_dir();
            set_corner_dir(direct+2); //此时路口为通
            if(TURN_BACK_FLAG == 1){   //掉头
                TURN_BACK_FLAG = 0;
                set_corner_dir(direct+2);
                printf("back\r\n");
            //break;
            }
            if (BOTH_FLAG == 1){			//双
                    BOTH_FLAG = 0;
                    buzzer_turn_on_delay(10);
                    set_corner_dir(direct+1);		
                    set_corner_dir(direct-1);
                    printf("both\r\n");
                //break;
            }	
            
            if (TURN_LEFT_FLAG == 1){	  //左
                    TURN_LEFT_FLAG = 0;
                    set_corner_dir(direct-1);  //设置当前左边的绝对方向可行
                    printf("left\r\n");
                //break;
            }
            if (TURN_UP_FLAG == 1) {
                    TURN_UP_FLAG = 0;
                    set_corner_dir(direct);
                    printf("up\r\n");
                //break;
            }
            if (TURN_RIGHT_FLAG == 1){	//右
                    TURN_RIGHT_FLAG = 0;
                    set_corner_dir(direct+1);	//设置当前右边的绝对方向可行
                    printf("right\r\n");
                //break;
            }
            Clear_NODEflag();
            
            if(END_FLAG == 0){ //如果没有到终点
                    ab_direction = add_point(RE_LocY,RE_LocX,ab_fy,ab_x,ab_y,ab_fx); //记录节点，并返回行驶方向（返回的是绝对方向
            }
            else{  //到达终点
                    ab_direction = gogogo(RE_LocY,RE_LocX,ab_fy,ab_x,ab_y,ab_fx);    //回溯路径
                    printf("g_t:%d",g_t);
            }
            //绝对方向转换为当前转向（绝对方向
            if(END_FLAG==0){ //找到终点前的转弯
                 switch (ab_direction){            //get_dir(ab_direction)
                    case turn_left:{ //向左 1
                            printf("turn_left\r\n");	
                            cflag_left =1;
                            break;}	
                    case turn_right:{ //向右  3
                            printf("turn_right\r\n");
                            cflag_right = 1;
                            break;}
                    case turn_back:{ //后退
                            printf("turn_back\r\n");
                            cflag_tn180 = 1;                    
                            break;}		
                    case turn_up:{
                            printf("turn_up\r\n");
                            cflag_up=1;				
                            break; }
//					case 7:{
//					
//					break;}
                    default:
                            printf("other\r\n");
                            break;}
            clear_corner_dir();
            }
            else{
                 switch (get_dir(ab_direction)){            
                    case turn_left:{ //向左 1
                            printf("l\r\n");
                            cflag_left =1;
                            break;}	
                    case turn_right:{ //向右  3
                            printf("r\r\n");
                            cflag_right = 1;
                            break;}
                    case turn_back:{ //后退
                            cflag_tn180 = 1;
                            printf("b\r\n");
                            break;}		
                    case turn_up:{
                            printf("u\r\n");
                            cflag_up=1;				
                            break; }
//					case 7:{
//					
//					break;}
                    default:
                            break;}
            clear_corner_dir();
            }
            
        //执行转动作	
            printf("go:%d\r\n",go->no);
            if(STOP_FLAG==0){  //没收到停止标识  、或者节点不是终点时继续走		
            //printf("turn");
                    //***执行各转动作***//  以yaw角度为目标  ,实际测试，由玩车的安装方式导致转角度比小
                if(cflag_left == 1){
                        Turn_left();}			
                else if(cflag_right == 1){
                        Turn_right();}	
                //Locationhold();
                else if(cflag_tn180 ==1){
                        Turn_back();}
                else if(cflag_up == 1){
                        Turn_up();}
                Clear_FLAG();
                STATE --; //回到循环状态
        }else{ //到终点时，不需要置cflag，暂停turn，只用黑边
                    END_FLAG = 1;
                    STATE=5;
                    //printf("STOP");
            }
        break;}
        
    case 5:{  //到达终点置位置标志
        //printf("state5\r\n");
        cflag_turn=1;  	//停止巡黑边
        INTERRUPT_FLAG =1;
        lock_Loc();
        Set_Pwm(0,0); 
        //find_exit2 = 1;                        
        END_FLAG = 1;       //设为终点标志
        //STOP_FLAG = 0;
        HC_SR04_ALLOW_FLAG =1;
        buzzer_turn_on_delay(200);
        START_FLAG = 0;
        STATE++;
        break;}
    
    case 6:{  //到达终点等待启动状态
        //printf("state6\r\n");
    if(START_FLAG==1){
        START_FLAG = 0;
        STATE++ ;
    }
    if( DL_GPIO_readPins(KEY_Group_PORT,KEY_Group_P21_PIN) > 0){
            Read_pin = 0;}
        else{
            Read_pin = 1;
        }
        if (Read_pin==1 && Read_pin_pre==1){  //改成用宏开始，或Start_Command 可能要写一个校准，以是常量                           
            Key_count +=1;
            if(Key_count == 3){	
                START_FLAG = 1;
                Key_count = 0;
                Read_pin = 0;
                Read_pin_pre = 0;
                //cflag_turn = 0;
                STATE++;
                break;
            }}
        Read_pin_pre = Read_pin;
        break; }
        
    case 7:{  //从终点脱离
            //printf("state7\r\n");
        STOP_FLAG = 0;
        find_exit2 = 0;
        HC_SR04_ALLOW_FLAG = 0;
        //judge_dir();
        // NODE_DETECT_FLAG=1;//重置终点标志
        lock_Loc(); //这里和turn函数back中也都有用，所以在此调用
        Turn_back();
        Set_Pwm(0,0);
        Clear_levels();
        INTERRUPT_FLAG = 0;
        PID_init();
        cflag_turn = 0;
        //judge_dir();
        STATE = 3;
        break;
        }
    
    case 8:{  //从终点返回巡线
        break;}
    
    default:
        break;
    }
void task2(void){
    if(send_data_flag){
        send_data_flag = 0;
        ANO_Direct_distance(direct,blue_node_num);
    }

    switch (STATE2){
    case 0:{  //初始化
        Read_pin = 0;
        Read_pin_pre = 0;
        Key_count = 0;
        PARA_MOTO.TVL = 0;
        PARA_MOTO.TVR = 0;
        INTERRUPT_FLAG = 1;
        cflag_turn = 1;
        for(int i = 0; i < 50; i++){
            node2_list[i] = 0;
        }
        STATE2++;
        break;}
    
    case 1:{ //等待启动、转状态
        if(START_FLAG == 1)
        {
            STATE2++;
        }
        if(DL_GPIO_readPins(KEY_Group_PORT,KEY_Group_P21_PIN) > 0)
        {
            Read_pin = 0;
        }else{
            Read_pin = 1;
        }
        if (Read_pin==1 && Read_pin_pre==1)  //改成用宏开始，或Start_Command 可能要写一个校准，以是常量                           
        {
            Key_count += 1;
            if(Key_count == 3)
            {	
                START_FLAG = 1;
                Key_count = 0;
                Read_pin = 0;
                Read_pin_pre = 0;
                STATE2++;
                break;
            }
        }
        Read_pin_pre = Read_pin;
        break;}
        
        
    case 2:{  //收到开始开始行走状态
        if(START_FLAG == 1)            
        {	
            blue_node_num++;
            START_FLAG = 0;
            //yaw_adjust = yaw_int;  //记录初始角度
            //start_point();
            PARA_MOTO.TVL = 30;  
            PARA_MOTO.TVR = 30;
            PID_init();
            cflag_turn = 0;
            Set_Target_Veclocity(PARA_MOTO.TVL, PARA_MOTO.TVR);
            //start_point();
            OLED_ShowString(0,32,(uint8_t *)"start",8,1);
            OLED_Refresh();
            delay_ms(100);
            INTERRUPT_FLAG = 0;
            HC_SR04_ALLOW_FLAG = 0;
            delay_count = 0;
            STATE2++;
        }
        break;}
                
    case 3:{  //循环状态 1:检测障碍物 2：识别终点  3：识别到黑复合道路
        if(HCSR04_WORK_FLAG == 0){  //避障器                        
            HCSR04_WORK_FLAG = 1;
            distance = Hcsr04GetLength();
            if(distance < 30)
            {
                INTERRUPT_FLAG = 1;
                distance = Hcsr04GetLength();
                Set_Pwm(0,0);
            }
            else{
                INTERRUPT_FLAG = 0;
            }
        }
            
        if(nining_flag){
            INTERRUPT_FLAG2 = 1;
            Set_Pwm(1000,1000);
        }
            
        if(rain_flag){	
            INTERRUPT_FLAG = 1;
            cflag_turn = 1;
            Set_Pwm(0,0);
            while(rain_flag)
            {
                buzzer_turn_on_delay(10);
                delay_ms(100);
            }
            INTERRUPT_FLAG = 0;
            cflag_turn = 0;
        }
            
        if(ban_flag){
            INTERRUPT_FLAG = 1;
            cflag_turn = 1;
            Set_Pwm(0,0);
            while(ban_flag){
                buzzer_turn_on_delay(50);
                delay_ms(100);
            }
            cflag_turn = 0;
            INTERRUPT_FLAG = 0;
        }
        //printf("%.2f\r\n",distance);
        //INFRARED_cal_error();
        if(NODE_DETECT_FLAG){	
            NODE_DETECT_FLAG = 0;
            buzzer_turn_on_delay(10);
            INTERRUPT_FLAG = 1;
            cflag_turn = 1;  //暂停巡黑边
            INFRARED_ERROR = 0;
            Pwm_diff_zero();
            delay_ms(100);    //往前走走0.5s  //可能增加一个延时计数，定时器使用
            Locationhold();   //更新方向与节点参数
            if(STOP_FLAG){
                STATE2++;
                break;
            }
            int levels = read_infrared_levels();
            if(levels&0b01110){   //此时levels不为零，说明前方有路
                TURN_UP_FLAG = 1;  //允许前进
                buzzer_turn_on_delay(10);
                STATE2++;
            }
            else{
                TURN_UP_FLAG = 0;
                STATE2++;
            }
        }	
        break;}
    
    case 4:{ //循环接收节点，记录并执行转弯	
        NODE_DETECT_FLAG = 0; //清除收节点标志
        blue_node_num++;
        if(node2_list[node2_num])
        {
            switch (node2_list[node2_num++]){            //get_dir(ab_direction)
            case 1:{ //向左 1
                cflag_left = 1;
                break;}	
            case 3:{ //向右  3
                cflag_right = 1;
                break;}	
            case 2:{
                cflag_up = 1;				
                break; }
            default:
                break;}
        }else if(node2_list[node2_num] == 0){
            STOP_FLAG = 1;	
        }
                        
        if(STOP_FLAG == 1){
            INTERRUPT_FLAG = 1;
            Set_Pwm(0,0);
            STATE2 = 5;
            Clear_NODEflag();//清除左右方向
        }	
        //执行转弯动作	
        if(STOP_FLAG == 0){  //没收到停止标识  、或者节点不是终点时继续走		
            //***执行各转动作***//  以yaw角度为目标  ,实际测试，由玩车的安装方式导致转角度比小
            if(cflag_left == 1){
                Turn_left();}			
            else if(cflag_right == 1){
                Turn_right();}	
            //Locationhold();
            else if(cflag_up == 1){
                Turn_up();}
            Clear_FLAG();
            STATE2--; //回到循环状态
        }else{ //到终点时，不需要置cflag，暂停turn，只用黑边
            END_FLAG = 1;
            STATE2 = 5;
            printf("STOP");
        }
        break;}
        
    case 5:{  //到达终点置位置标志
        printf("state5\r\n");
        cflag_turn = 1;  //停止巡黑边
        lock_Loc();
        Set_Pwm(0,0); 
        //find_exit2 = 1;                        
        END_FLAG = 1;       //设为终点标志
        //STOP_FLAG = 0;
        HC_SR04_ALLOW_FLAG = 1;
        buzzer_turn_on_delay(50);
        STATE2++;
        break;}
    
    default:
        break;
    }
}

//****处理各个路口判断进行继续行走****//
int level;
void Turn_right(void){    //右转
    cflag_turn = 1;
    delay_ms(200);
    //judge_dir();
    INTERRUPT_FLAG = 1; //停止PID中断计算速度
    cflag_turn = 1;  //转动作在执行
    lock_Loc(); 
    direct++;	
    Set_Pwm(2000,-2000);

    delay_ms(300);
    level = read_infrared_levels();
    
    while(((level&0b01110) == 0))
    {
        level = read_infrared_levels();
    }
    lock_Loc();
    
    Set_Pwm(0,0);
    cflag_turn = 0;
    PID_init(); //重新速度设置，恢复PID速度调节
    INTERRUPT_FLAG = 0;	
}

void Turn_left(void){    //左转
    cflag_turn = 1;
    delay_ms(150);
    //judge_dir();
    INTERRUPT_FLAG = 1; //停止PID中断计算速度
    
    lock_Loc();
    direct--;
    Set_Pwm(-2000,2000);
    delay_ms(300);
    level = read_infrared_levels();
    
    while(((level&01110) == 0))
    {
        level = read_infrared_levels();
    }
    lock_Loc();
    Set_Pwm(0,0);
    
    cflag_turn = 0;
    PID_init(); //重新速度设置，恢复PID速度调节
    INTERRUPT_FLAG = 0;	
}

void Turn_back(void){
    cflag_turn = 1;
    delay_ms(150);
    //judge_dir();
    INTERRUPT_FLAG = 1; //停止PID中断计算速度
    lock_Loc();
    direct += 2;
    Set_Pwm(1800,-1800);
    delay_ms(1000);
    level = read_infrared_levels();
    while(((level&01110) == 0))
    {
        level = read_infrared_levels();
    }
    lock_Loc();
    Set_Pwm(0,0);
    
    cflag_turn = 0;
    PID_init(); //重新速度设置，恢复PID速度调节
    INTERRUPT_FLAG = 0;	
}

void Turn_up(void){
    cflag_turn = 1;
    //judge_dir();
    level = read_infrared_levels();
    delay_ms(150);
    cflag_turn = 0;
}

void start_point(void)                            //初始化第一个节点
{
    point[0].x = 0;
    point[0].y = 0;
    point[0].qian = 1;
    point[0].hou = 0;
    point[0].left = 0;
    point[0].right = 0;
    point[0].node_num = 1;
    point[0].qian_view = 0;
    point[0].hou_view = 0;
    point[0].left_view = 0;
    point[0].right_view = 0;
    point[0].no = 0;
    point[0].next_qian = &save;
    point[0].next_hou = &save;
    point[0].next_left = &save;
    point[0].next_right = &save;
    point[0].next = &save;
    save = point[0];
    save.qian = 0;
    point[0].qian_view = 1;
    all_point = 1;                            //此时所有的点数量 就只有1个 也就是第一个点
    direction_t = 2;                        //方向为2，朝前，初始的绝对方向为向前
    find_exit2  = 0;                        //找到终点的标志位为0
    distance_t = 0;                        //路径长度为0
    distance_min_t = 9999999;        //最小路径长度为9999999
    p_t = 0;                                //当前点的标号为0
    i_t = 0;                                //循环变量为0
    for ( ; i_t < 100; i_t++) {
        point[i_t] = save;
    }
    last = &point[0];     		//上一个点为第一个点 因为第一个节点没有上一个点
    go = last->next_qian;			//当前点为第一个点的前方的下一个点 ,此时的go==save
    for (i_t=0 ; i_t < 100; i_t++) {
        gogo[i_t] = 7;					//7表示无用
    }
    buzzer_turn_on_delay(50);
}
/**
 * @brief 添加一个点到路径中，并根据方向更新路径信息。
 * @param get_x 点的x坐标
 * @param get_y 点的y坐标
 * @param get_left 是否可以向左
 * @param get_qian 是否可以向前
 * @param get_right 是否可以向右
 * @param get_hou 是否可以向后
 * @return int 返回当前方向
 */
int add_point (float get_y, float get_x, int get_left, int get_qian, int get_right, int get_hou)
{
    //printf("x:%.1f,y:%.1f",get_x,get_y);
    direction_t = direct;
    //judge_dir();
    if (go == &save) {     //save是map结构初始化的结构体，相当于go==一个初始化节点，且未记录的节点时
     // printf("save\r\n");  
        p_point = 0;
        // 遍历所有的点，检查是否存在相同坐标的点 
        while (p_point < all_point-1) {
            if ((get_x > (point[p_point].x - yunxvwucha) )&& (get_x < (point[p_point].x + yunxvwucha))) {	  //get_x 在当前点范围内
                if ((get_y > (point[p_point].y - yunxvwucha)) && (get_y < (point[p_point].y + yunxvwucha))) {  //在某点的范围内
                    //printf("cf\r\n");	
                    last_locx = get_y;   //记录最新的坐标
                    last_locy = get_x;   //last_locy即是当此坐标
                    if (p_t >= point[p_point].no - 1) {  //p_t为当前点的标号 找到p_point（点在检索点中）
                        now = &point[p_point];						//当前点设置为找到的点
                        now->x = get_x;
                        now->y = get_y;
                        go = last;												//回头，go的目标变成上一个点												
                        printf("dir:%d",direction_t);
                        switch (direction_t%4) {						//判断上一次的转向，根据上次的转向，记录当前点（重复点）与上一个点的链接关系
                            case 1:												
                                last->next_left = now;		
                                now->next_right = last;
                                now->right_view = max_node_view();
                                //direction_t = 0;					
                                distance_t += abs(now->x - go->x);								
                                break;
                            case 2:
                                last->next_qian = now;		//上一点的前方的下一个点为当前点
                                now->next_hou = last;			//反向的，当前的后方的下一个点为上一个点
                                //direction_t = 0;					//方向为零
                                now->hou_view =  max_node_view();
                                distance_t += abs(now->y - go->y);
                                break;
                            case 3:
                                last->next_right = now;		
                                now->next_left = last;
                                now->left_view = max_node_view();
                                direction_t = 0;
                                distance_t += abs(now->x - go->x);
                                break;
                            case 0:
                                last->next_hou = now;
                                now->next_qian = last;
                                now->qian_view = max_node_view();
                                direction_t = 0;
                                distance_t += abs(now->y - go->y);
                                break;
                        }
                        int t = get_min_node_view();
                        switch(direct%4){
                            case 0:{
                                if(t == 0){
                                    direction_t = 2;
                                    go = now->next_hou;
                                    now->hou_view = max_node_view();
                                    break;}
                                if(t == 1){
                                    direction_t = 3;
                                    go = now->next_left;
                                    now->left_view = max_node_view();
                                    break;}
                                if(t == 2){
                                    direction_t = 0;
                                    go = now->next_qian;
                                    now->qian_view = max_node_view();
                                    break;}
                                if (t == 3){
                                    direction_t = 1;
                                    go = now->next_right;
                                    now->right_view = max_node_view();
                                    break;}		
                            }
                            case 1:{
                                if(t == 0){
                                    direction_t = 1;
                                    go = now->next_hou;
                                    now->hou_view = max_node_view();
                                    break;}
                                if(t == 1){
                                    direction_t = 2;
                                    go = now->next_left;
                                    now->left_view = max_node_view();
                                    break;}	
                                if(t == 2){
                                    direction_t = 3;
                                    go = now->next_qian;
                                    now->qian_view = max_node_view();
                                    break;}
                                if (t == 3){
                                    direction_t = 0;
                                    go = now->next_right;
                                    now->right_view = max_node_view();
                                    break;}	
                            }
                            case 2:{
                                if(t == 0){
                                    direction_t = 0;
                                    go = now->next_hou;
                                    now->hou_view = max_node_view();
                                    break;}
                                if(t == 1){
                                    direction_t = 1;
                                    go = now->next_left;
                                    now->left_view = max_node_view();
                                    break;}	
                                if(t == 2){
                                    direction_t = 2;
                                    go = now->next_qian;
                                    now->qian_view = max_node_view();
                                    break;}
                                if (t == 3){
                                    direction_t = 3;
                                    go = now->next_right;
                                    now->right_view = max_node_view();
                                    break;}	
                            }
                            case 3:{
                                if(t == 0){
                                    direction_t = 3;
                                    go = now->next_hou;
                                    now->hou_view = max_node_view();
                                    break;}
                                if(t == 1){
                                    direction_t = 0;
                                    go = now->next_left;
                                    now->left_view = max_node_view();
                                    break;}	
                                if(t == 2){
                                    direction_t = 1;
                                    go = now->next_qian;
                                    now->qian_view = max_node_view();
                                    break;}
                                if (t == 3){
                                    direction_t = 2;
                                    go = now->next_right;
                                    now->right_view = max_node_view();
                                    break;}	
                            }
                        }						
                        p_t++;							
                        now->no--;  				
                        last = now;	
                        printf("old_point:%d\r\n",now->no);
                        return direction_t;   //返回转向
                    }
                }
            }
            p_point++;  									
        }
        // 若点的数量达到上限，返回错误码
        if (all_point == 100) {  					//若所有的节点数量达到极限100，则返回标号99
            return 99;
        }
        //遍历循环没找到重复点， 创建新点
        //printf("new_p");
        // OLED_ShowString(36,36,(uint8_t *)"np",24,1);
        new_point = all_point;						//进入这里即判断是自己点在布所在，且点不存在，添加该节点，该节点即为当前点。若点标号为节点0此时所有点数为1（从ALL=new+1）
        all_point++;											//所有节点数量加1
        now = &point[new_point];					//map类型的now指针指向当前节点
        now->x = get_x;										//记录当前节点的x坐标
        now->y = get_y;										//记录当前节点的y坐标
        now->qian = get_qian;							//记录当前节点是否可以向前走
        now->hou = get_hou;								//记录当前节点是否可以向后走
        now->left = get_left;							
        now->right = get_right;
        calc_node_num();

        p_t++;														//当前节点最终加1
        now->no = p_t;										//当前节点的标号为当前最终
        printf("new_point:%d",p_t);
        // 更新当前点相连的路径信息
        //judge_dir();
        switch (direction_t%4) {						//依据下一次移动的方向更新路径信息
            case 1:												//若是上次转向为左
                last->next_left = now;		//上一点的左方的下一个点为当前点	
                now->next_right = last;		//当前点的右方的下一个点为上一个点
                now->right_view = max_node_view();				//当前点的右方已尝试过
                distance_t += abs(now->x - last->x); //路径长度加上当前点的x坐标减去上一个点的x坐标的绝对值
                break;
            case 2:
                last->next_qian = now;
                now->next_hou = last;
                now->hou_view = max_node_view();
                distance_t += abs(now->y - last->y);
                break;
            case 3:
                last->next_right = now;
                now->next_left = last;
                now->left_view = max_node_view();
                distance_t += abs(now->x - last->x);
                break;
            case 0:
                last->next_hou = now;
                now->next_qian = last;
                now->qian_view = max_node_view();
                distance_t += abs(now->y - last->y);
                break;
        }
        // 若是找到终点，保存路径信息
      
        if (find_exit2 == 1) {//找到终点了
            printf("find\r\n");
            find_exit2 = 0;
            cflag_foundd = 1;
            cflag_found = 1;
            g_t = 0;
            if (distance_t < distance_min_t) {
                distance_min_t = distance_t;
                tail = now;
                n_t = p_t;
                //all_point --;
                while (1) {
                    p_point = 0;
                    nomin = n_t;
                    while (p_point < all_point) {
                        if ((point[p_point].next_hou == tail) || (point[p_point].next_right == tail) || (point[p_point].next_qian == tail) || (point[p_point].next_left == tail)) {
                            if (point[p_point].no < nomin) {
                                nomin = point[p_point].no;
                                point[p_point].next = tail;
                                hp = &point[p_point];
                            }
                        }
                        p_point ++;
                    }
                    n_t = nomin;
                    tail = hp;
                    if (hp->next_left == tail->next) {
                        gogo[g_t] = 3;
                    }
                    if (hp->next_qian == tail->next) {
                        gogo[g_t] = 0;
                    }
                    if (hp->next_right == tail->next) {
                        gogo[g_t] = 1;
                    }
                    if (hp->next_hou == tail->next) {
                    gogo[g_t] = 2;	
                    }
                    length++;
                    g_t ++ ;
                    if (tail == &point[0]) {
                        break;
                    }
                }
                tail = &point[1];
                hp = tail;
                g_t =0;
                printf("p_t:%d\r\n",p_t);//pt=4
    
                for(int i =0;gogo[i]!=7;i++)
                {
                    printf("%d",gogo[i]);
                }
                printf("\n");
                return 7;
            }
            buzzer_turn_on_delay(200);
        }
    } 
    //若此时go记录不是初始go（=save初始值）。。
    else { //若没到达终点
        if (p_t < go->no - 1) {
            if (all_point == 100) {
                return 99;
            }
            new_point = all_point;
            all_point++;
            now = &point[new_point];
            now->x = get_x;
            now->y = get_y;
            now->qian = get_qian;
            now->hou = get_hou;
            now->left = get_left;
            now->right = get_right;
            calc_node_num();
            p_t++;
            now->no = p_t;
            printf("go_point:%d\r\n",p_t);
            // 根据当前点更新的路径信息
            switch (direction_t%4) {
                case 1:
                    last->next_left = now;
                    now->next_right = last;
                    now->right_view = max_node_view();
                    distance_t += abs(now->x - last->x);
                    break;
                case 2:
                    last->next_qian = now;
                    now->next_hou = last;
                    now->hou_view = max_node_view();
                    distance_t += abs(now->y - last->y);
                    break;
                case 3:
                    last->next_right = now;
                    now->next_left = last;
                    now->left_view = max_node_view();
                    distance_t += abs(now->x - last->x);
                    break;
                case 0:
                    last->next_hou = now;
                    now->next_qian = last;
                    now->qian_view = max_node_view();
                    distance_t += abs(now->y - last->y);
                    break;
            }
        } 
        else {
            last_locx = go->x;
            last_locy = go->y;
            p_t--;
            last->no++;
            now = go;
            printf("delete\r\n");
            // 根据当前点更新距离
            switch (direction_t%4) {
                case 1:
                    distance_t -= abs(now->x - last->x);
                    break;
                case 2:
                    distance_t -= abs(now->y - last->y);
                    break;
                case 3:
                    distance_t -= abs(now->x - last->x);
                    break;
                case 0:
                    distance_t -= abs(now->y - last->y);
                    break;
            }
        }
    }

    if(judge_node(now) == 1){
        //拐弯节点
        switch (direct%4)
        {
        case 0:
            go = now->next_hou;
            now->hou_view = max_node_view();
            last = now;
            break;
        case 1:
            go = now->next_left;
            now->left_view = max_node_view();
            last = now;
            break;
        case 2:
            go = now->next_qian;
            now->qian_view = max_node_view();
            last = now;	
            break;
        case 3:
            go = now->next_right;
            now->right_view = max_node_view();
            last = now;
            break;
        
        default:
            break;
        }
        direction_t = 2;
        return direction_t;
    }

    switch(direct%4){
        case 0:{ //当前方向是后
            if(now->right == 1){
                if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
                    direction_t = 1;  //向右转
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return direction_t;}
            }
            if(now->hou==1){
                if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
                    direction_t = 2;  //向前直转
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;					
                    return direction_t;}
            }
            if(now->left== 1){
                if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
                    direction_t = 3;  //向左转
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;}
            }
            if(now->qian == 1){
                if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
                    direction_t = 0;  //向右掉头
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;}
            }
            break;}
        
        case 1:{ //当前方向是左侧
            if(now->hou == 1){
                if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
                    direction_t = 1;  //向右转
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return direction_t;
                }
            }
            if(now->left==1){
                if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
                    direction_t = 2;  //向前直转
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
            }
            if(now->qian== 1){
                if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
                    direction_t = 3;  //向左转
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
            }
            if(now->right == 1){
                if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
                    direction_t = 0;  //向右掉头
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
            }
            break;}
        case 2:{ //当前方向是前qian
            if(now->left == 1){
                if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
                    direction_t = 1;  //向右转
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return direction_t;
                }
            }
            if(now->qian==1){
                if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
                    direction_t = 2;  //向前直转
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
            }
            if(now->right== 1){
                if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
                    direction_t = 3;  //向左转
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            if(now->hou == 1){
                if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
                    direction_t = 0;  //向右掉头
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
            }
            break;}
        case 3:{ //当前方向是右侧
            if(now->qian == 1){
                if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
                    direction_t = 1;  //向右转
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return direction_t;
                }
            }
            if(now->right==1){
                if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
                    direction_t = 2;  //向前直转
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
            }
            if(now->hou== 1){
                if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
                    direction_t = 3;  //向左转
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            if(now->left == 1){
                if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
                    direction_t = 0;  //向右掉头
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
            }
            break;}
        
        default:
            break;
    }
    now2 = now;
    switch(direct%4){
        case 0:{ //当前方向是后
            if(now->right == 1){
                if (now->no - 1 == now2->next_right->no) {
                    direction_t = 1;  //向右转
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;	
                    return direction_t;
                }}
            if(now->hou==1){
                if (now->no - 1 == now2->next_hou->no) {
                    direction_t = 2;  //向前直转
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;					
                    return direction_t;
                }}
            if(now->left== 1){
                if (now->no - 1 == now2->next_left->no) {
                    direction_t = 3;  //向左转
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            if(now->qian == 1){
                if (now->no - 1 == now2->next_qian->no) {
                    direction_t = 0;  //向右掉头
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            break;}
        
        case 1:{ //当前方向是左侧
            if(now->hou == 1){
                if (now->no - 1 == now2->next_hou->no) {
                    direction_t = 1;  //向右转
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return direction_t;
                }}
            if(now->left==1){
                if (now->no - 1 == now2->next_left->no) {
                    direction_t = 2;  //向前直转
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;					
                    return direction_t;
                }}
            if(now->qian== 1){
                if (now->no - 1 == now2->next_qian->no) {
                    direction_t = 3;  //向左转
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            if(now->right == 1){
                if (now->no - 1 == now2->next_right->no) {
                    direction_t = 0;  //向左转
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            break;}
        case 2:{ //当前方向是前qian
            if(now->left == 1){
                if (now->no - 1 == now2->next_left->no) {
                    direction_t = 1;  //向右转
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return direction_t;
                }}
            if(now->qian==1){
                if (now->no - 1 == now2->next_qian->no) {
                    direction_t = 2;  //向前直转
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;					
                    return direction_t;
                }}
            if(now->right== 1){
                if (now->no - 1 == now2->next_right->no) {
                    direction_t = 3;  //向左转
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            if(now->hou == 1){
                if ((now->no - 1 == now2->next_hou->no)) {
                    direction_t = 0;  //向右掉转
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            break;}
        case 3:{ //当前方向是右侧
            if(now->qian == 1){
                if ((now->no - 1 == now2->next_qian->no)) {
                    direction_t = 1;  //向右转
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return direction_t;
                }}
            if(now->right==1){
                if ((now->no - 1 == now2->next_right->no)) {
                    direction_t = 2;  //向前直转
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;					
                    return direction_t;
                }}
            if(now->hou== 1){
                if (now->no - 1 == now2->next_hou->no) {
                    direction_t = 3;  //向左转
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            if(now->left == 1){
                if(now->no - 1 == now2->next_left->no) {
                    direction_t = 0;  //向左转
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            break;}
        
        default:
            break;
    }

    switch(direct%4){
        case 0:{ //当前方向是后
            if(now->right == 1){
                direction_t = 1;  //向右转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;	
                return direction_t;
            }
            if(now->hou==1){
                
                direction_t = 2;  //向前直转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;					
                return direction_t;
            }
            if(now->left== 1){
                
                direction_t = 3;  //向左转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return  direction_t;
            }
            if(now->qian == 1){
                
                direction_t = 0;  //向右掉头
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return  direction_t;
            }
            break;}
        
        case 1:{ //当前方向是左侧
            if(now->hou == 1){
                
                direction_t = 1;  //向右转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
            if(now->left==1){
            
                direction_t = 2;  //向前直转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;					
                return direction_t;
            }
            if(now->qian== 1){
                
                direction_t = 3;  //向左转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return  direction_t;
            }
            if(now->right == 1){
                
                direction_t = 0;  //向左转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return  direction_t;
            }
            break;}
        case 2:{ //当前方向是前qian
            if(now->left == 1){
                
                direction_t = 1;  //向右转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
            if(now->qian==1){
                
                direction_t = 2;  //向前直转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;					
                return direction_t;
            }
            if(now->right== 1){
                
                direction_t = 3;  //向左转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return  direction_t;
            }
            if(now->hou == 1){
                
                direction_t = 0;  //向右掉转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return  direction_t;
            }
            break;}
        case 3:{ //当前方向是右侧
            if(now->qian == 1){
                
                direction_t = 1;  //向右转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
            if(now->right==1){
                
                direction_t = 2;  //向前直转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;					
                return direction_t;
            }
            if(now->hou== 1){
                
                direction_t = 3;  //向左转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return  direction_t;
            }
            if(now->left == 1){
                
                direction_t = 0;  //向左转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return  direction_t;
            }
            break;}
        
        default:
            break;
    }
    return 777;//返回
}

int gogogo(float get_x, float get_y, int get_left, int get_qian, int get_right, int get_hou)
{
//	if (g_t >= 100) {
//		g_t = 0;
//	}
    if (find_exit2 == 1) {
        return 7;
    }
    g_t++;
    if(gogo[g_t]==7){
            STOP_FLAG =1;
    }
    return gogo[g_t];  // 返回对应的终点路
}

// 清除转弯flag
void Clear_FLAG(void){
    cflag_right=0;
    cflag_left=0;
    cflag_up=0;
    cflag_tn180=0;
}

void calc_node_num(void){           
    int num=0;
    if (now->qian){
            num++;
    }
    if(now->hou){
        num++;
    }
    if(now->left){
            num++;
    }
    if(now->right){
        num++;
    }
    now->node_num = num;
}

// 寻找当前节点最大值，返回值
int max_node_view(void){
    int temp=0;
    if(now->left_view > temp){
             temp = now->left_view;
    }
    if(now->qian_view>temp){
            temp = now->qian_view;
    }
    if(now->right_view > temp){
        temp = now->right_view;
    }
    if(now->hou_view > temp){
             temp = now->hou_view;
    }
    temp ++;
    return temp;
}

// 返回方向的最小节点
int get_min_node_view(void){
    int temp = 100,record = 1;
    if(now->left){
            if(temp > now->left_view){
                record = 1;
                temp = now->left_view;
            }
    }
    if(now->qian){
            if(temp >now->qian_view){
                    record = 2;
                    temp = now->qian_view;
            }
    }
    if(now->right){
            if(temp > now->right_view){
                record = 3;
                temp = now->right_view;
            }
    }
    if(now->hou){
            if(temp > now->hou_view){
                record = 0;
                temp = now->hou_view;
            }
    }
    return record;
}

int judge_node(struct map *node){
        uint8_t dir = direct;
        switch(dir%4){
            case 0:{
                if(last->hou == 1&& last->left == 1&&last->right == 0&&last->qian ==0){  // 判断节点L型路口
                    if(now->right == 1&&now-> hou == 1&&now->left == 0&&now->qian == 1){
                        return 1;
                    }}
                else 
                    return 0;
                break;}
            case 1:{
                if(last->left == 1&& last->qian == 1&&last->right == 0&&last->hou ==0){  // 判断节点L型路口
                    if(now->left == 1&&now-> hou == 1&&now->right == 1&&now->qian == 0){
                        return 1;
                    }}
                else 
                    return 0;
                break;}
            case 2:{
                if (last->qian == 1&& last->right == 1&&last->left == 0&&last->hou ==0){  // 判断节点L型路口
                    if(now->right == 0&&now-> hou == 1&&now->left == 1&&now->qian == 1){
                        return 1;
                    }}
                else 
                    return 0;
                break;}
            case 3:{
                if(last->right == 1&& last->hou == 1&&last->left == 0&&last->qian ==0){  // 判断节点L型路口
                    if(now->left == 1&&now-> hou == 0&&now->right == 1&&now->qian == 1){
                        return 1;   
                    }}
                else 
                    return 0;
                break;}

                default :
                    return 0;
        }
            return 0;
		}
			return 0;
	}#include "task.h"
    #include "infrared.h"
    #define yaw_error_allow 50 //����yaw���5��
    
    /*************************�����ǹ��ڹ�����ͼ�ķ���***************************/
    #define yunxvwucha 10    						//�������    ��λcm 
    int gogo[100], g_t;      						//�������飬����˳��ȡ��ָ�g_tΪ·���ĳ��� ,����·������
    int new_point, all_point, p_point; 	//�µ㣬���е�����������ѭ���ж��Ƿ����ʱ�ĵ�ǰ�㣨��һ���ֲ�ʹ�õ�ѭ��������
    int i_t, n_t,  nomin;  							//ѭ�����������ڳ�ʼ�����еĽڵ㣩��ѭ����������С��
    int p_t; 														//��ǰ��ı��
    int direction_t=2;										//�����һ���ƶ��ķ���1����2��ǰ��3���ң�4��󣨾��Է���������ͼ�ķ���
    float distance_t, distance_min_t;   //·�����ȣ���С·������
    
    struct map													//��ͼ�ṹ��
    {
        int x;														//x����
        int y;														//y����
        uint8_t qian;											//ǰ���Ƿ���·
        uint8_t hou;											//���Ƿ���·
        uint8_t left;											//���Ƿ���·
        uint8_t right;										//�ҷ��Ƿ���· 
        uint8_t qian_view;
        uint8_t hou_view;
        uint8_t left_view;
        uint8_t right_view;
        uint8_t node_num;									//��·����
        uint8_t no;												//���
        struct map *next_qian;						//��һ��ǰ���ĵ�
        struct map *next_hou;							//��һ���󷽵ĵ�
        struct map *next_left;						//��һ���󷽵ĵ�
        struct map *next_right;						//��һ���ҷ��ĵ�
        struct map *next;									//��һ����
    //	struct map *pervious;
    };
    struct map point[100]; 							//��ŵ�ͼ�ڵ������
    struct map save;       							//��ŵ�ǰ�����Ϣ
    struct map *go, *now, *last, *now2, *head, *hp, *tail; //��һ���㣬��ǰ�㣬��һ���㣬��ǰ�㣬ͷ�ڵ㣬��ǰ�㣬β�ڵ�
    
    //***Ӳ���йر���/��־***//
    extern  PARA_Moto PARA_MOTO;  			//�������
    extern short Target_Velocity_L,Target_Velocity_R;  //���ҵ��Ŀ���ٶ�
    uint8_t START_FLAG=0;								//��ʼ��־
    uint32_t Read_pin, Read_pin_pre;	//
    uint8_t Key_count;									//������ȡ����
    uint8_t STATE = 0,STATE2= 0;									//task����state���У���ʼ״̬Ϊ0
    int find_exit2;                     //�ҵ��յ�ı�־λ 
    extern uint8_t HCSR04_WORK_FLAG;		//���������ڹ�����־
    extern uint8_t HC_SR04_ALLOW_FLAG;	//����������������־
    extern short INTERRUPT_FLAG,INTERRUPT_FLAG2;	//20ms�жϿ��Ʋ�������λ
    int delay_count;  									//ʶ��·�ں����ʱ����
    
    //***��ͼ�йر���/��־***//
    uint8_t cflag_right=0,cflag_left=0,cflag_tn180=0,cflag_stop,cflag_up;		    //���������־����ת����ת����ͷ��ֹͣ
    uint8_t cflag_count=0;                      //??�ô���ʱδ֪
    uint8_t cflag_turn =1 ;                      //����ת���־
    extern uint8_t cflag_right,cflag_left;								//���������־
    extern int LocX,LocY,last_locx,last_locy,RE_LocX,RE_LocY;             //�������� ��ʼ����Ϊ��������x�����ݴ�ľ�������   //���ڼ�¼��Locx��LocY  
    extern uint8_t direct;   
    int yaw_record; //��¼yawֵ
    int yaw_target; //����Ŀ��yawֵ������ת��
    extern int yaw_adjust ;
    extern int yaw_int;
    //uint8_t dir; //����
    extern uint8_t RECORED_DOWN;  //�ڵ��¼��ɱ�־
    extern uint8_t END_FLAG;			//�յ��־
    int ab_direction;  //��ǰ�ڵ��н�����
    extern uint8_t NODE_DETECT_FLAG,TURN_RIGHT_FLAG,TURN_LEFT_FLAG ,BOTH_FLAG,TURN_UP_FLAG,TURN_BACK_FLAG;
    uint8_t Start_Command;    //���������ܳ����иı�
    uint8_t nining_flag=0,rain_flag=0,ban_flag = 0;  //����ͷʶ����Ţ·��
    int node2_list[50];    //task2��ת������
    int node2_num=0;
    uint8_t blue_node_num = 0;
    //...����...//
    void task1(void){  //�����Թ�����¼
    
        if(send_data_flag){
                    send_data_flag = 0;
                    ANO_Direct_distance(direct,blue_node_num);
            }
    
        switch (STATE){
        case 0:{  //��ʼ��
            Read_pin = 0;
            Read_pin_pre = 0;
            Key_count = 0;
            PARA_MOTO.TVL = 0;
            PARA_MOTO.TVR = 0;
            INTERRUPT_FLAG = 1;
            cflag_turn = 1;
            STATE ++;
            break;}
        
        case 1:{ //�ȴ�������ת״̬
            if(START_FLAG == 1)
            {
                STATE ++;
            }
            if( DL_GPIO_readPins(KEY_Group_PORT,KEY_Group_P21_PIN) > 0)
            {
                Read_pin = 0;
            }else{
                Read_pin = 1;
            }
            if (Read_pin==1 && Read_pin_pre==1)  //�ĳ��ֻ���ʼ����Start_Command ����Ҫдһ��У׼�����ǳ���                           
            {
                Key_count += 1;
                if(Key_count == 3)
                {	
                    START_FLAG = 1;
                    Key_count = 0;
                    Read_pin = 0;
                    Read_pin_pre = 0;
                    STATE++;
                    break;
                }
            }
            Read_pin_pre = Read_pin;
            break;}
            
            
        case 2:{  //�յ���ʼ��ʼ����״̬
            if(START_FLAG == 1)            
            {	
                blue_node_num++;
                START_FLAG =0 ;
                yaw_adjust = yaw_int;  //��¼��ʼ�Ƕ�
                start_point();
                PARA_MOTO.TVL = 30;  
                PARA_MOTO.TVR = 30;
                PID_init();
                cflag_turn = 0;
                Set_Target_Veclocity(PARA_MOTO.TVL, PARA_MOTO.TVR);
                //start_point();
                OLED_ShowString(0,32,(uint8_t *)"start",8,1);
                OLED_Refresh();
                delay_ms(100);
                INTERRUPT_FLAG = 0;
                HC_SR04_ALLOW_FLAG = 0;
                delay_count = 0;
                STATE++;
            }
            break;}
                    
        case 3:{  //ѭ��״̬ 1:���������� 2��ʶ���յ�  3��ʶ�𵽺�ͬ��·�� 
                if(HCSR04_WORK_FLAG == 0)  //������
                {                           
                    HCSR04_WORK_FLAG =1;
                    distance = Hcsr04GetLength();
                    if(distance < 30)
                    {
                        INTERRUPT_FLAG = 1;
                        distance = Hcsr04GetLength();
                        Set_Pwm(0,0);
                    }
                    else{
                    INTERRUPT_FLAG = 0;
                    }
                }
                
                if(nining_flag){
                INTERRUPT_FLAG2 = 1;
                Set_Pwm(1000,1000);
    //		Set_Target_Veclocity(20,20);
    //		clear_pid(&Motor_velocity_L)  ;
    //		clear_pid(&Motor_velocity_R)  ;
    //		pid_init(&Motor_velocity_L, 90, 4, 16); 
    //		pid_init(&Motor_velocity_R, 90, 4, 16);
        }
                
                if(rain_flag){	
            INTERRUPT_FLAG = 1;
            cflag_turn = 1;
            Set_Pwm(0,0);
            while(rain_flag)
            {
                    buzzer_turn_on_delay(10);
                    delay_ms(100);
            }
            INTERRUPT_FLAG = 0;
            cflag_turn = 0;
        }
                //printf("%.2f\r\n",distance);
                //INFRARED_cal_error();
                if(ban_flag){
                INTERRUPT_FLAG = 1;
                cflag_turn=1;
                Set_Pwm(0,0);
                while(ban_flag){
                    buzzer_turn_on_delay(50);
                    delay_ms(50);
                }
                cflag_turn = 0;
                INTERRUPT_FLAG = 0;
            }	
            
                if(NODE_DETECT_FLAG){	
                                
                                NODE_DETECT_FLAG = 0;
                                buzzer_turn_on_delay(10);
                                INTERRUPT_FLAG = 1;
                                cflag_turn = 1;  //��ͣ���Ҷ�
                                INFRARED_ERROR = 0;
                                Pwm_diff_zero();
                                delay_ms(100);    		//��ǰ����0.5s  //��������һ����ʱ��������ʹ��delaycount
                                Locationhold();				//���ݷ��������ڵ�����
            //				Turn_back();//
            //				break;
                                if(STOP_FLAG){
                                    STATE++;
                                    break;
                                }
                                int levels = read_infrared_levels();
                                if(levels&0b01110){   					//��ʱlevels������0��˵��ǰ����·
                                        TURN_UP_FLAG =1;  //����ǰ��
                                        buzzer_turn_on_delay(10);
                                        STATE ++;
                                }
                                else{
                                        TURN_UP_FLAG = 0;
                                        STATE ++;
                                }
                        }	
                    break;}
        
        case 4:{ //ѭ�����ֽڵ㣬��¼����ִ��ת��	
                printf("state4\r\n");	
                NODE_DETECT_FLAG = 0; //������ֽڵ��־
                blue_node_num++;
                if(STOP_FLAG == 1){
                        find_exit2 = 1;
                        INTERRUPT_FLAG = 1;
                        Set_Pwm(0,0);
                        STATE =5;
                        Clear_NODEflag();//������Ƿ������
                }
                //judge_dir();
                set_corner_dir(direct+2); //��ʱ·����Ϊͨ
                if(TURN_BACK_FLAG == 1){   //��ͷ
                    TURN_BACK_FLAG = 0;
                    set_corner_dir(direct+2);
                    printf("back\r\n");
                //break;
                }
                if (BOTH_FLAG == 1){			//˫
                        BOTH_FLAG = 0;
                        buzzer_turn_on_delay(10);
                        set_corner_dir(direct+1);		
                        set_corner_dir(direct-1);
                        printf("both\r\n");
                    //break;
                }	
                
                if (TURN_LEFT_FLAG == 1){	  //��
                        TURN_LEFT_FLAG = 0;
                        set_corner_dir(direct-1);  //���õ�ǰ��ߵľ��Է������
                        printf("left\r\n");
                    //break;
                }
                if (TURN_UP_FLAG == 1) {
                        TURN_UP_FLAG = 0;
                        set_corner_dir(direct);
                        printf("up\r\n");
                    //break;
                }
                if (TURN_RIGHT_FLAG == 1){	//��
                        TURN_RIGHT_FLAG = 0;
                        set_corner_dir(direct+1);	//���õ�ǰ�ұߵľ��Է������
                        printf("right\r\n");
                    //break;
                }
                Clear_NODEflag();
                
                if(END_FLAG == 0){ //���û�е��յ�
                        ab_direction = add_point(RE_LocY,RE_LocX,ab_fy,ab_x,ab_y,ab_fx); //��¼�ڵ㣬������ʻ���򣨷��ص��Ǿ��Է���
                }
                else{  //�����յ�
                        ab_direction = gogogo(RE_LocY,RE_LocX,ab_fy,ab_x,ab_y,ab_fx);    //��������
                        printf("g_t:%d",g_t);
                }
                //���Է���ת��Ϊ��ǰת�����Է���
                if(END_FLAG==0){ //�ҵ��յ�ǰ��ת��
                     switch (ab_direction){            //get_dir(ab_direction)
                        case turn_left:{ //���� 1
                                printf("turn_left\r\n");	
                                cflag_left =1;
                                break;}	
                        case turn_right:{ //����  3
                                printf("turn_right\r\n");
                                cflag_right = 1;
                                break;}
                        case turn_back:{ //���
                                printf("turn_back\r\n");
                                cflag_tn180 = 1;                    
                                break;}		
                        case turn_up:{
                                printf("turn_up\r\n");
                                cflag_up=1;				
                                break; }
    //					case 7:{
    //					
    //					break;}
                        default:
                                printf("other\r\n");
                                break;}
                clear_corner_dir();
                }
                else{
                     switch (get_dir(ab_direction)){            
                        case turn_left:{ //���� 1
                                printf("l\r\n");
                                cflag_left =1;
                                break;}	
                        case turn_right:{ //����  3
                                printf("r\r\n");
                                cflag_right = 1;
                                break;}
                        case turn_back:{ //���
                                cflag_tn180 = 1;
                                printf("b\r\n");
                                break;}		
                        case turn_up:{
                                printf("u\r\n");
                                cflag_up=1;				
                                break; }
    //					case 7:{
    //					
    //					break;}
                        default:
                                break;}
                clear_corner_dir();
                }
                
            //ִ��ת�䶯��	
                printf("go:%d\r\n",go->no);
                if(STOP_FLAG==0){  //û���յ�ֹͣ��ʶ  �����ڵ����յ�ʱ���ͣ��		
                //printf("turn");
                        //***ִ����ת����***//  ��yaw�Ƕ�ΪĿ��  ,ʵ�ʲ��ԣ��������еİ�װ��ʽ����ת�Ƕȱ�С
                    if(cflag_left == 1){
                            Turn_left();}			
                    else if(cflag_right == 1){
                            Turn_right();}	
                    //Locationhold();
                    else if(cflag_tn180 ==1){
                            Turn_back();}
                    else if(cflag_up == 1){
                            Turn_up();}
                    Clear_FLAG();
                    STATE --; //�ص�ѭ��״̬
            }else{ //����ʱ����Ҫ���cflag����turn���ûҶ�
                        END_FLAG = 1;
                        STATE=5;
                        //printf("STOP");
                }
            break;}
            
        case 5:{  //�����յ���λ��ر�־
            //printf("state5\r\n");
            cflag_turn=1;  	//ֹͣ���Ҷ�
            INTERRUPT_FLAG =1;
            lock_Loc();
            Set_Pwm(0,0); 
            //find_exit2 = 1;                        
            END_FLAG = 1;       //��Ϊ�յ��־
            //STOP_FLAG = 0;
            HC_SR04_ALLOW_FLAG =1;
            buzzer_turn_on_delay(200);
            START_FLAG = 0;
            STATE++;
            break;}
        
        case 6:{  //�����յ��ȴ�����״̬
            //printf("state6\r\n");
        if(START_FLAG==1){
            START_FLAG = 0;
            STATE++ ;
        }
        if( DL_GPIO_readPins(KEY_Group_PORT,KEY_Group_P21_PIN) > 0){
                Read_pin = 0;}
            else{
                Read_pin = 1;
            }
            if (Read_pin==1 && Read_pin_pre==1){  //�ĳ��ֻ���ʼ����Start_Command ����Ҫдһ��У׼�����ǳ���                           
                Key_count +=1;
                if(Key_count == 3){	
                    START_FLAG = 1;
                    Key_count = 0;
                    Read_pin = 0;
                    Read_pin_pre = 0;
                    //cflag_turn = 0;
                    STATE++;
                    break;
                }}
            Read_pin_pre = Read_pin;
            break; }
            
        case 7:{  //���յ��ͷ
                //printf("state7\r\n");
            STOP_FLAG = 0;
            find_exit2 = 0;
            HC_SR04_ALLOW_FLAG = 0;
            //judge_dir();
            // NODE_DETECT_FLAG=1;//�����յ��־
            lock_Loc(); //������turn����back��Ҳ����ã������ڴ˵���
            Turn_back();
            Set_Pwm(0,0);
            Clear_levels();
            INTERRUPT_FLAG = 0;
            PID_init();
            cflag_turn = 0;
            //judge_dir();
            STATE = 3;
            break;
            }
        
        case 8:{  //���յ��������Ѳ��
            break;}
        
        default:
            break;
        }
    }	
            
    void task2(void){
        if(send_data_flag){
                    send_data_flag = 0;
                    ANO_Direct_distance(direct,blue_node_num);
            }
    
        switch (STATE2){
        case 0:{  //��ʼ��
            Read_pin = 0;
            Read_pin_pre = 0;
            Key_count = 0;
            PARA_MOTO.TVL = 0;
            PARA_MOTO.TVR = 0;
            INTERRUPT_FLAG = 1;
            cflag_turn = 1;
            for(int i;i<50;i++){
                node2_list[i] =0;
            }
            STATE2 ++;
            break;}
        
        case 1:{ //�ȴ�������ת״̬
            if(START_FLAG == 1)
            {
                STATE2 ++;
            }
            if( DL_GPIO_readPins(KEY_Group_PORT,KEY_Group_P21_PIN) > 0)
            {
                Read_pin = 0;
            }else{
                Read_pin = 1;
            }
            if (Read_pin==1 && Read_pin_pre==1)  //�ĳ��ֻ���ʼ����Start_Command ����Ҫдһ��У׼�����ǳ���                           
            {
                Key_count += 1;
                if(Key_count == 3)
                {	
                    START_FLAG = 1;
                    Key_count = 0;
                    Read_pin = 0;
                    Read_pin_pre = 0;
                    STATE2++;
                    break;
                }
            }
            Read_pin_pre = Read_pin;
            break;}
            
            
        case 2:{  //�յ���ʼ��ʼ����״̬
            if(START_FLAG == 1)            
            {	
                blue_node_num++;
                START_FLAG =0 ;
                //yaw_adjust = yaw_int;  //��¼��ʼ�Ƕ�
                //start_point();
                PARA_MOTO.TVL = 30;  
                PARA_MOTO.TVR = 30;
                PID_init();
                cflag_turn = 0;
                Set_Target_Veclocity(PARA_MOTO.TVL, PARA_MOTO.TVR);
                //start_point();
                OLED_ShowString(0,32,(uint8_t *)"start",8,1);
                OLED_Refresh();
                delay_ms(100);
                INTERRUPT_FLAG = 0;
                HC_SR04_ALLOW_FLAG = 0;
                delay_count = 0;
                STATE2++;
            }
            break;}
                    
        case 3:{  //ѭ��״̬ 1:���������� 2��ʶ���յ�  3��ʶ�𵽺�ͬ��·�� 
                if(HCSR04_WORK_FLAG == 0){  //������                          
                    HCSR04_WORK_FLAG =1;
                    distance = Hcsr04GetLength();
                    if(distance < 30)
                    {
                        INTERRUPT_FLAG = 1;
                        distance = Hcsr04GetLength();
                        Set_Pwm(0,0);
                    }
                    else{
                    INTERRUPT_FLAG = 0;
                    }
                }
                
                if(nining_flag){
                INTERRUPT_FLAG2 = 1;
                Set_Pwm(1000,1000);
    //		Set_Target_Veclocity(20,20);
    //		clear_pid(&Motor_velocity_L)  ;
    //		clear_pid(&Motor_velocity_R)  ;
    //		pid_init(&Motor_velocity_L, 90, 4, 16); 
    //		pid_init(&Motor_velocity_R, 90, 4, 16);
        }
                
                if(rain_flag){	
            INTERRUPT_FLAG = 1;
            cflag_turn = 1;
            Set_Pwm(0,0);
            while(rain_flag)
            {
                    buzzer_turn_on_delay(10);
                    delay_ms(100);
            }
            INTERRUPT_FLAG = 0;
            cflag_turn = 0;
        }
                
            if(ban_flag){
                INTERRUPT_FLAG = 1;
                cflag_turn=1;
                Set_Pwm(0,0);
                while(ban_flag){
                    buzzer_turn_on_delay(50);
                    delay_ms(100);
                }
                cflag_turn = 0;
                INTERRUPT_FLAG = 0;
            }
            //printf("%.2f\r\n",distance);
                //INFRARED_cal_error();
                if(NODE_DETECT_FLAG){	
                    
                        NODE_DETECT_FLAG = 0;
                        buzzer_turn_on_delay(10);
                        INTERRUPT_FLAG = 1;
                        cflag_turn = 1;  //��ͣ���Ҷ�
                        INFRARED_ERROR = 0;
                        Pwm_diff_zero();
                        delay_ms(100);    		//��ǰ����0.5s  //��������һ����ʱ��������ʹ��delaycount
                        Locationhold();				//���ݷ��������ڵ�����
    //				Turn_back();//
    //				break;
                        if(STOP_FLAG){
                            STATE2++;
                            break;
                        }
                        int levels = read_infrared_levels();
                        if(levels&0b01110){   					//��ʱlevels������0��˵��ǰ����·
                                TURN_UP_FLAG =1;  //����ǰ��
                                buzzer_turn_on_delay(10);
                                STATE2 ++;
                        }
                        else{
                                TURN_UP_FLAG = 0;
                                STATE2 ++;
                        }
                }	
            break;}
        
        case 4:{ //ѭ�����ֽڵ㣬��¼����ִ��ת��	
                //printf("state4\r\n");	
                NODE_DETECT_FLAG = 0; //������ֽڵ��־
                blue_node_num++;
                 if(node2_list[node2_num])
                 {
                         switch (node2_list[node2_num++]){            //get_dir(ab_direction)
                        case 1:{ //���� 1
                                cflag_left =1;
                                break;}	
                        case 3:{ //����  3
                                cflag_right = 1;
                                break;}	
                        case 2:{
                                cflag_up=1;				
                                break; }
                        default:
                                break;}
                 }else if(node2_list[node2_num] == 0){
                     STOP_FLAG = 1;	
                 }
                             
            if(STOP_FLAG == 1){
                        INTERRUPT_FLAG = 1;
                        Set_Pwm(0,0);
                        STATE2 =5;
                        Clear_NODEflag();//������Ƿ������
                }	
            //ִ��ת�䶯��	
            if(STOP_FLAG==0){  //û���յ�ֹͣ��ʶ  �����ڵ����յ�ʱ���ͣ��		
                //printf("turn");
                        //***ִ����ת����***//  ��yaw�Ƕ�ΪĿ��  ,ʵ�ʲ��ԣ��������еİ�װ��ʽ����ת�Ƕȱ�С
                    if(cflag_left == 1){
                            Turn_left();}			
                    else if(cflag_right == 1){
                            Turn_right();}	
                    //Locationhold();
                    else if(cflag_up == 1){
                            Turn_up();}
                    Clear_FLAG();
                    STATE2 --; //�ص�ѭ��״̬
            }else{ //����ʱ����Ҫ���cflag����turn���ûҶ�
                        END_FLAG = 1;
                        STATE2=5;
                        printf("STOP");
                }
            break;}
            
        case 5:{  //�����յ���λ��ر�־
            printf("state5\r\n");
            cflag_turn=1;  //ֹͣ���Ҷ�
            lock_Loc();
            Set_Pwm(0,0); 
            //find_exit2 = 1;                        
            END_FLAG = 1;       //��Ϊ�յ��־
            //STOP_FLAG = 0;
            HC_SR04_ALLOW_FLAG =1;
            buzzer_turn_on_delay(50);
            STATE2++;
            break;}
        
        default:
            break;
        }
    }
    
    //printf("node2");  //���Խڵ㣬��һֱ��� �������׳�
    //****��������·���жϽ��о������****//
    int level;
    void Turn_right(void){    //��ת
        cflag_turn=1;
        delay_ms(200);
        //judge_dir();
        INTERRUPT_FLAG = 1; //ֹͣPID�жϼ����ٶ�
        cflag_turn = 1;  //ת������ִ��
    //	yaw_record = yaw;
    //	yaw_target = yaw_record - 90;
    //	if(yaw_target <= -180)
    //	{
    //			yaw_target += 360;
    //	}
    //||(abs_float(yaw_target - yaw) > yaw_error_allow)
        lock_Loc(); 
    direct++;	
        Set_Pwm(2000,-2000);
    //	OLED_ShowString(0,32,(uint8_t *)"Right",16,1);
    //	OLED_Refresh();
    
        delay_ms(300);
        level = read_infrared_levels();
    //	
        while(((level&0b01110) == 0 ))
        {
            level = read_infrared_levels();
        }
        lock_Loc();
        
        Set_Pwm(0,0);
        cflag_turn = 0;
        PID_init(); //�����ٶ����ã�����PID���µ���
        INTERRUPT_FLAG = 0;	
    }
    
    void Turn_left(void){    //��ת
        cflag_turn=1;
        delay_ms(150);
        //judge_dir();
        INTERRUPT_FLAG = 1; //ֹͣPID�жϼ����ٶ�
    //	yaw_record = yaw;
    //	yaw_target = yaw_record + 90;
    //	if(yaw_target > 180)
    //	{
    //			yaw_target -= 360;
    //	}else if(yaw_target <-180)
    //	{
    //			yaw_target += 360;
    //	}   || (abs_float(yaw_target - yaw) > yaw_error_allow)
        
        lock_Loc();
        direct--;
        Set_Pwm(-2000,2000);
    //	OLED_ShowString(0,32,(uint8_t *)"Left ",16,1);
    //	OLED_Refresh();
    //	while(abs_float(yaw_target - yaw) > yaw_error_allow);
        delay_ms(300);
        level = read_infrared_levels();
    //	
        while(((level&01110) == 0))
        {
            level = read_infrared_levels();
        }
        lock_Loc();
        Set_Pwm(0,0);
        
        cflag_turn = 0;
        PID_init(); //�����ٶ����ã�����PID���µ���
        INTERRUPT_FLAG = 0;	
    }
    
    void Turn_back(void){
        cflag_turn=1;
        delay_ms(150);
        //judge_dir();
        INTERRUPT_FLAG = 1; //ֹͣPID�жϼ����ٶ�
    //	yaw_record = yaw_int;
    //	yaw_target = yaw_record + 180;
    //	if(yaw_target > 180)
    //			yaw_target -= 360;
    //	if(yaw_target < -180)
    //			yaw_target += 360;
        lock_Loc();
        direct+=2;
        Set_Pwm(1800,-1800);
    //	OLED_ShowString(0,32,(uint8_t *)"Back ",16,1);
    //	OLED_Refresh();
        delay_ms(1000);
        //||(abs_int(yaw_target - yaw_int) > yaw_error_allow)
        level = read_infrared_levels();
        while(((level&01110) == 0))
        {
            level = read_infrared_levels();
        }
        lock_Loc();
        Set_Pwm(0,0);
        
        cflag_turn = 0;
        PID_init(); //�����ٶ����ã�����PID���µ���
        INTERRUPT_FLAG = 0;	
    }
    
    void Turn_up(void){
        cflag_turn=1;
    //	OLED_ShowString(0,32,(uint8_t *)"up   ",16,1);
    //	OLED_Refresh();
        //judge_dir();
        level = read_infrared_levels();
        delay_ms(150);
        cflag_turn = 0;
    }
    
    void start_point (void)							//��ʼ����һ���ڵ�
    {
        point[0].x = 0;
        point[0].y = 0;
        point[0].qian = 1;
        point[0].hou = 0;
        point[0].left = 0;
        point[0].right = 0;
        point[0].node_num = 1;
        point[0].qian_view = 0;
        point[0].hou_view = 0;
        point[0].left_view = 0;
        point[0].right_view = 0;
        point[0].no = 0;
        point[0].next_qian = &save;
        point[0].next_hou = &save;
        point[0].next_left = &save;
        point[0].next_right = &save;
        point[0].next = &save;
    //	point[0].pervious = &save;
        save = point[0];
        save.qian = 0;
        point[0].qian_view = 1;
        all_point = 1;							//��ʱ���е������ ��ֻ��1�� Ҳ���ǵ�һ����
        direction_t = 2;						//����Ϊ2����ǰ����ʼ�ľ��Է���Ϊ��ǰ
        find_exit2  = 0;							//�ҵ��յ�ı�־λΪ0
        distance_t = 0;							//·������Ϊ0
        distance_min_t = 9999999;		//��С·������Ϊ9999999
        p_t = 0;										//��ǰ��ı��Ϊ0
        i_t = 0;										//ѭ������Ϊ0	
        for ( ; i_t < 100; i_t++) {               //*for (i_t; i_t < 100; i_t++) {*//
    //		point[i_t].next_qian = &save;
    //		point[i_t].next_hou = &save;
    //		point[i_t].next_left = &save;
    //		point[i_t].next_right = &save;
    //		point[i_t].next = &save;
            point[i_t] = save;
        }
        last = &point[0];     		//��һ����Ϊ��һ���� ����Ϊ��һ���ڵ�û����һ����
        go = last->next_qian;			//��ǰ��Ϊ��һ�����ǰ����һ���� ,��ʱ��go==save
        for (i_t=0 ; i_t < 100; i_t++) {
            gogo[i_t] = 7;					//7��������
        }
        buzzer_turn_on_delay(50);
    }
    
    /**
     * @brief ����һ���㵽·���У������ݷ������·����Ϣ��
     * @param get_x 			���x����
     * @param get_y 			���y����
     * @param get_left 		�Ƿ����������
     * @param get_qian 		�Ƿ������ǰ��
     * @param get_right 	�Ƿ����������
     * @param get_hou 		�Ƿ���������
     * @return int 		���ص�ǰ����
     */
    int add_point (float get_y, float get_x, int get_left, int get_qian, int get_right, int get_hou)
    {
    
            //printf("x:%.1f,y:%.1f",get_x,get_y);
            direction_t = direct;
            //judge_dir();
        if (go == &save) {     //save��map���ڳ�ʼ���Ľṹ�壬���Ե�go==һ����ʼ���ڵ㣬��δ��¼�Ľڵ�ʱ
         // printf("save\r\n");  
                    p_point = 0;
            // �������е㣬����Ƿ������ͬ����ĵ� 
            while (p_point < all_point-1) {
                if ((get_x > (point[p_point].x - yunxvwucha) )&& (get_x < (point[p_point].x + yunxvwucha))) {	  //get_x �ڵ�ǰ�����Χ��
                    if ((get_y > (point[p_point].y - yunxvwucha)) && (get_y < (point[p_point].y + yunxvwucha))) {  //��ĳ�������Χ��
                                            //printf("cf\r\n");	
                                            last_locx = get_y;   //��¼���µ�����
                        last_locy = get_x;   //last_locy�����ݴ������
                        if (p_t >= point[p_point].no - 1) {  //p_tΪ��ǰ��ı�� ����p_point�����������е�
                            now = &point[p_point];						//��ǰ�����Ϊ�ҵ��ĵ�
                                                    now->x = get_x;
                                                    now->y = get_y;
                                                  go = last;												//��ͷ��go��Ŀ�������һ����												
                                                    printf("dir:%d",direction_t);
                                                    switch (direction_t%4) {						//�ж���һ�ε�ת���򣬸����ϴε�ת�����¼��ǰ�㣨�ظ��㣩����һ��������ӹ�ϵ
                                case 1:												
                                    last->next_left = now;		
                                    now->next_right = last;
                                                                    now->right_view = max_node_view();
                                    //direction_t = 0;					
                                    distance_t += abs(now->x - go->x);								
                                    break;
                                case 2:
                                    last->next_qian = now;		//��һ�����ǰ������һ����Ϊ��ǰ��
                                    now->next_hou = last;			//�෴�ģ������ĺ󷽵���һ����Ϊ��һ����
                                    //direction_t = 0;					//����Ϊ��
                                                                    now->hou_view =  max_node_view();
                                                                    distance_t += abs(now->y - go->y);
                                    break;
                                case 3:
                                    last->next_right = now;		
                                    now->next_left = last;
                                                                    now->left_view = max_node_view();
                                    direction_t = 0;
                                    distance_t += abs(now->x - go->x);
                                    break;
                                case 0:
                                    last->next_hou = now;
                                    now->next_qian = last;
                                                                    now->qian_view = max_node_view();
                                    direction_t = 0;
                                    distance_t += abs(now->y - go->y);
                                    break;
                            }
                                                    int t = get_min_node_view();
                                                    switch(direct%4){
                                                        case 0:{
                                                                if(t == 0){
                                                                        direction_t = 2;
                                                                        go = now->next_hou;
                                                                        now->hou_view = max_node_view();
                                                                    break;}
                                                                if(t == 1){
                                                                        direction_t = 3;
                                                                        go = now->next_left;
                                                                        now->left_view = max_node_view();
                                                                    break;}
                                                                if(t == 2){
                                                                        direction_t = 0;
                                                                        go = now->next_qian;
                                                                        now->qian_view = max_node_view();
                                                                    break;}
                                                                if (t == 3){
                                                                        direction_t = 1;
                                                                        go = now->next_right;
                                                                        now->right_view = max_node_view();
                                                                    break;}		
                                                        }
                                                        case 1:{
                                                                if(t == 0){
                                                                        direction_t = 1;
                                                                        go = now->next_hou;
                                                                        now->hou_view = max_node_view();
                                                                    break;}
                                                                if(t == 1){
                                                                        direction_t = 2;
                                                                        go = now->next_left;
                                                                        now->left_view = max_node_view();
                                                                    break;}	
                                                                if(t == 2){
                                                                        direction_t = 3;
                                                                        go = now->next_qian;
                                                                        now->qian_view = max_node_view();
                                                                    break;}
                                                                if (t == 3){
                                                                        direction_t = 0;
                                                                        go = now->next_right;
                                                                        now->right_view = max_node_view();
                                                                    break;}	
                                                        }
                                                        case 2:{
                                                                if(t == 0){
                                                                        direction_t = 0;
                                                                        go = now->next_hou;
                                                                        now->hou_view = max_node_view();
                                                                    break;}
                                                                if(t == 1){
                                                                        direction_t = 1;
                                                                        go = now->next_left;
                                                                        now->left_view = max_node_view();
                                                                    break;}	
                                                                if(t == 2){
                                                                        direction_t = 2;
                                                                        go = now->next_qian;
                                                                        now->qian_view = max_node_view();
                                                                    break;}
                                                                if (t == 3){
                                                                        direction_t = 3;
                                                                        go = now->next_right;
                                                                        now->right_view = max_node_view();
                                                                    break;}	
                                                        }
                                                        case 3:{
                                                                if(t == 0){
                                                                        direction_t = 3;
                                                                        go = now->next_hou;
                                                                        now->hou_view = max_node_view();
                                                                    break;}
                                                                if(t == 1){
                                                                        direction_t = 0;
                                                                        go = now->next_left;
                                                                        now->left_view = max_node_view();
                                                                    break;}	
                                                                if(t == 2){
                                                                        direction_t = 1;
                                                                        go = now->next_qian;
                                                                        now->qian_view = max_node_view();
                                                                    break;}
                                                                if (t == 3){
                                                                        direction_t = 2;
                                                                        go = now->next_right;
                                                                        now->right_view = max_node_view();
                                                                    break;}	
                                                        }
                                                    }						
    //												direction_t =  get_min_node_view(); //���ݸýڵ������������ȷ���û���ʹ��ķ������ȫ�����ʹ�������������ʵķ���
    //												switch(direction_t%4){								//���ݴ˴��н��ķ�����λ��Ӧ����ķ���ֵ
    //													case 0:{
    //														  go = now->next_hou;
    //															now->hou_view = max_node_view();
    //														break;
    //													}
    //													case 1:{
    //															go=now->next_left;
    //															now->left_view = max_node_view();
    //														break;
    //													}
    //													case 2:{
    //															go=now->next_qian;
    //															now->qian_view = max_node_view();
    //														break;
    //													}
    //													case 3:{
    //															go = now->next_right;
    //															now->right_view = max_node_view();
    //															break;
    //														}
    //												}	
    //												printf("canturn:l:%d;q:%d,r:%d,h:%d\r\n",now->left,now->qian,now->right,now->hou);
    //												printf("old_p;l:%d,q:%d,r:%d,h:%d\r\n",now->left_view,now->qian_view,now->right_view,now->hou_view);													
                                                    p_t++;							
                            now->no--;  				
                            last = now;	
                                                    printf("old_point:%d\r\n",now->no);
                            return direction_t;   //����ת����
                        }
                    }
                }
                p_point++;  									
            }
            // �����������ﵽ���ޣ����ش�����
            if (all_point == 100) {  					//������нڵ������ﵽ����100�������ر���99
                return 99;
            }
            //����ѭ��û�ҵ��ظ��㣬 �����µ�
            //printf("new_p");
                    // OLED_ShowString(36,36,(uint8_t *)"np",24,1);
                    new_point = all_point;						//������ж��Ƿ��Լ����ڲ��������������ڣ��������Ӹýڵ㣬�ýڵ��Ϊ��ǰ��������Ϊ�ڵ�0��ʱ������Ϊ1����ALL=new+1��
            all_point++;											//���нڵ��������1
            now = &point[new_point];					//map���͵�nowָ��ָ��ǰ�ڵ�
            now->x = get_x;										//��¼��ǰ�ڵ��x����
            now->y = get_y;										//��¼��ǰ�ڵ��y����
            now->qian = get_qian;							//��¼��ǰ�ڵ��Ƿ������ǰ��
            now->hou = get_hou;								//��¼��ǰ�ڵ��Ƿ���������
            now->left = get_left;							
            now->right = get_right;
                    calc_node_num();
    
            p_t++;														//��ǰ�ڵ��ż�1
            now->no = p_t;										//��ǰ�ڵ�ı��Ϊ��ǰ���
                    printf("new_point:%d",p_t);
            // ���ݵ�ǰ�������·����Ϣ
                    //judge_dir();
            switch (direction_t%4) {						//������һ���ƶ��ķ������·����Ϣ
                case 1:												//����ϴ�ת����Ϊ��
                                    last->next_left = now;		//��һ������󷽵���һ����Ϊ��ǰ��	
                    now->next_right = last;		//��ǰ����ҷ�����һ����Ϊ��һ����
                                    now->right_view = max_node_view();				//��ǰ����ҷ����ʹ�
                                    distance_t += abs(now->x - last->x); //·�����ȼ��ϵ�ǰ���x�����ȥ��һ�����x����ľ���ֵ
                    break;
                case 2:
                    last->next_qian = now;
                    now->next_hou = last;
                                    now->hou_view = max_node_view();
                    distance_t += abs(now->y - last->y);
                    break;
                case 3:
                    last->next_right = now;
                    now->next_left = last;
                                    now->left_view = max_node_view();
                    distance_t += abs(now->x - last->x);
                    break;
                case 0:
                    last->next_hou = now;
                    now->next_qian = last;
                                    now->qian_view = max_node_view();
                    distance_t += abs(now->y - last->y);
                    break;
            }
            // ����ҵ��յ㣬����·����Ϣ
          
            if (find_exit2 == 1) {//�ҵ��յ���
                printf("find\r\n");
                find_exit2 = 0;
                cflag_foundd = 1;
                cflag_found = 1;
                g_t = 0;
                if (distance_t < distance_min_t) {
                    distance_min_t = distance_t;
                    tail = now;
                    n_t = p_t;
                    //all_point --;
                    while (1) {
                        p_point = 0;
                        nomin = n_t;
                        while (p_point < all_point) {
                            if ((point[p_point].next_hou == tail) || (point[p_point].next_right == tail) || (point[p_point].next_qian == tail) || (point[p_point].next_left == tail)) {
    //							if(point[p_point].node_num == 1&&(p_point !=0) ){
    //									continue;//�������ڵ���һ������ͬ�ڵ㣬ֱ������
    //							}	
                                if (point[p_point].no < nomin) {
                                    nomin = point[p_point].no;
                                    point[p_point].next = tail;
                                    hp = &point[p_point];
                                }
                            }
                            p_point ++;
                        }
                        n_t = nomin;
                        tail = hp;
                        if (hp->next_left == tail->next) {
                            gogo[g_t] = 3;
                        }
                        if (hp->next_qian == tail->next) {
                            gogo[g_t] = 0;
                        }
                        if (hp->next_right == tail->next) {
                            gogo[g_t] = 1;
                        }
                        if (hp->next_hou == tail->next) {
                        gogo[g_t] = 2;	
                        }
                        length++;
                        g_t ++ ;
                        if (tail == &point[0]) {
                            break;
                        }
                    }
                    tail = &point[1];
                    hp = tail;
                    g_t =0;
                    printf("p_t:%d\r\n",p_t);//pt=4
        
                    for(int i =0;gogo[i]!=7;i++)
                    {
                        printf("%d",gogo[i]);
                    }
                    printf("\n");
                    return 7;
                }
                buzzer_turn_on_delay(200);
            }
        } 
                    //������go��¼������go��=save��ʼֵ�����
            else { //��û�����յ�
                        if (p_t < go->no - 1) {
                if (all_point == 100) {
                    return 99;
                }
                new_point = all_point;
                all_point++;
                now = &point[new_point];
                now->x = get_x;
                now->y = get_y;
                now->qian = get_qian;
                now->hou = get_hou;
                now->left = get_left;
                now->right = get_right;
                            calc_node_num();
                p_t++;
                now->no = p_t;
                            printf("go_point:%d\r\n",p_t);
                // ���ݵ�ǰ�������·����Ϣ
                switch (direction_t%4) {
                    case 1:
                        last->next_left = now;
                        now->next_right = last;
                                            now->right_view = max_node_view();
                        distance_t += abs(now->x - last->x);
                        break;
                    case 2:
                        last->next_qian = now;
                        now->next_hou = last;
                                            now->hou_view = max_node_view();
                        distance_t += abs(now->y - last->y);
                        break;
                    case 3:
                        last->next_right = now;
                        now->next_left = last;
                                            now->left_view = max_node_view();
                        distance_t += abs(now->x - last->x);
                        break;
                    case 0:
                        last->next_hou = now;
                        now->next_qian = last;
                                            now->qian_view = max_node_view();
                        distance_t += abs(now->y - last->y);
                        break;
                }
                        } 
                        else {
    //            last_locx = go->y;
    //            last_locy = go->x;
                            last_locx = go->x;
                last_locy = go->y;
                p_t--;
                last->no++;
                now = go;
    //						now->x = get_x;
    //						now->y =get_y; 
                            printf("delete\r\n");
                // ���ݵ�ǰ������¾���
                switch (direction_t%4) {
                    case 1:
                        distance_t -= abs(now->x - last->x);
                        break;
                    case 2:
                        distance_t -= abs(now->y - last->y);
                        break;
                    case 3:
                        distance_t -= abs(now->x - last->x);
                        break;
                    case 0:
                        distance_t -= abs(now->y - last->y);
                        break;
                }
                        }
                }
    
    //		direction_t = get_min_node_view();
    //			//printf("num:%d",now->node_num);
    //		//printf("l:%d,q:%d,r:%d,h:%d\r\n",now->left_view,now->qian_view,now->right_view,now->hou_view);
    //		switch(direction_t%4){								//���ݴ˴��н��ķ�����λ��Ӧ����ķ���ֵ
    //				case 0:{
    //						go = now->next_hou;
    //						now->hou_view = max_node_view();
    //						last = now;
    //						return direction_t;
    //					break;
    //				}
    //				case 1:{
    //						go=now->next_left;
    //						now->left_view = max_node_view();
    //						last = now;
    //						return direction_t;
    //					break;
    //				}
    //				case 2:{
    //						go=now->next_qian;
    //						now->qian_view = max_node_view();
    //						last = now;
    //						return direction_t;
    //					break;
    //				}
    //				case 3:{
    //						go = now->next_right;
    //						now->right_view = max_node_view();
    //						last = now;
    //						return direction_t;
    //						break;
    //					}
    //			}		
    //    return 777; // ����
    //direct=judge_dir();
    
                    if(judge_node(now) == 1){
                            //������ڵ�
                            switch (direct%4)
                            {
                            case 0:
                                go = now->next_hou;
                                now->hou_view = max_node_view();
                                last = now;
                                break;
                            case 1:
                                go = now->next_left;
                                now->left_view = max_node_view();
                            last = now;
                                break;
                            case 2:
                                go = now->next_qian;
                                now->qian_view = max_node_view();
                                last = now;	
                                break;
                            case 3:
                                go = now->next_right;
                                now->right_view = max_node_view();
                                last = now;
                                break;
                            
                            default:
                                break;
                            }
                            direction_t = 2;
                            return direction_t;
                    }
    
    switch(direct%4){
        case 0:{ //��ǰ�������
                if(now->right == 1){
                    if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
                    direction_t = 1;  //�����ת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return direction_t;}
                }
                if(now->hou==1){
                    if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
                    direction_t = 2;  //���ֱת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;					
                    return direction_t;}
                }
                if(now->left== 1){
                    if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
                    direction_t = 3;  //�����ת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;}
            }
                if(now->qian == 1){
                      if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
                    direction_t = 0;  //��Ե�ͷ
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;}
            }
                break;}
        
        case 1:{ //��ǰ��������
                if(now->hou == 1){
                        if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
                    direction_t = 1;  //�����ת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return direction_t;
                }
            }
                if(now->left==1){
                    if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
                    direction_t = 2;  //���ֱת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
            }
                if(now->qian== 1){
                     if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
                    direction_t = 3;  //�����ת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
            }
                if(now->right == 1){
                      if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
                    direction_t = 0;  //��Ե�ͷ
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
            }
            break;}
        case 2:{ //��ǰ������qian
                if(now->left == 1){
                        if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
                    direction_t = 1;  //�����ת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return direction_t;
                }
            }
                if(now->qian==1){
                    if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
                    direction_t = 2;  //���ֱת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
            }
                if(now->right== 1){
                     if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
                    direction_t = 3;  //�����ת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
                if(now->hou == 1){
                      if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
                    direction_t = 0;  //��Ե�ͷ
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
        }
        break;}
        case 3:{ //��ǰ��������
                if(now->qian == 1){
                    if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
                    direction_t = 1;  //�����ת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return direction_t;
                }
            }
                if(now->right==1){
                    if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
                    direction_t = 2;  //���ֱת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
            }
                if(now->hou== 1){
                     if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
                    direction_t = 3;  //�����ת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
                if(now->left == 1){
                     if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
                    direction_t = 0;  //��Ե�ͷ
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
        }
        break;}
        
        default:
            break;
    }
    now2 = now;
    switch(direct%4){
        case 0:{ //��ǰ�������
                if(now->right == 1){
                    if (now->no - 1 == now2->next_right->no) {
                    direction_t = 1;  //�����ת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;	
                    return direction_t;
                }}
                if(now->hou==1){
                    if (now->no - 1 == now2->next_hou->no) {
                    direction_t = 2;  //���ֱת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;					
                    return direction_t;
                }}
                if(now->left== 1){
                     if (now->no - 1 == now2->next_left->no) {
                    direction_t = 3;  //�����ת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
                if(now->qian == 1){
                      if (now->no - 1 == now2->next_qian->no) {
                    direction_t = 0;  //��Ե�ͷ
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
                break;}
        
        case 1:{ //��ǰ��������
                if(now->hou == 1){
                    if (now->no - 1 == now2->next_hou->no) {
                    direction_t = 1;  //�����ת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return direction_t;
                }}
                if(now->left==1){
                    if (now->no - 1 == now2->next_left->no) {
                    direction_t = 2;  //���ֱת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;					
                    return direction_t;
                }}
                if(now->qian== 1){
                     if (now->no - 1 == now2->next_qian->no) {
                    direction_t = 3;  //�����ת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
                if(now->right == 1){
                      if (now->no - 1 == now2->next_right->no) {
                    direction_t = 0;  //�����ת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
            break;}
        case 2:{ //��ǰ������qian
                if(now->left == 1){
                        if (now->no - 1 == now2->next_left->no) {
                    direction_t = 1;  //�����ת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return direction_t;
                }}
                if(now->qian==1){
                    if (now->no - 1 == now2->next_qian->no) {
                    direction_t = 2;  //���ֱת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;					
                    return direction_t;
                }}
                if(now->right== 1){
                     if (now->no - 1 == now2->next_right->no) {
                    direction_t = 3;  //�����ת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
                if(now->hou == 1){
                      if ((now->no - 1 == now2->next_hou->no)) {
                    direction_t = 0;  //��Ե�ת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
        break;}
        case 3:{ //��ǰ��������
            if(now->qian == 1){
                        if ((now->no - 1 == now2->next_qian->no)) {
                    direction_t = 1;  //�����ת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return direction_t;
                }}
                if(now->right==1){
                    if ((now->no - 1 == now2->next_right->no)) {
                    direction_t = 2;  //���ֱת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;					
                    return direction_t;
                }}
                if(now->hou== 1){
                     if (now->no - 1 == now2->next_hou->no) {
                    direction_t = 3;  //�����ת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
                if(now->left == 1){
                    if(now->no - 1 == now2->next_left->no) {
                    direction_t = 0;  //�����ת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;
                }}
        break;}
        
        default:
            break;
    }
    
    switch(direct%4){
        case 0:{ //��ǰ�������
                if(now->right == 1){
                    direction_t = 1;  //�����ת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;	
                    return direction_t;
                }
                if(now->hou==1){
                    
                    direction_t = 2;  //���ֱת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
                if(now->left== 1){
                     
                    direction_t = 3;  //�����ת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
                if(now->qian == 1){
                      
                    direction_t = 0;  //��Ե�ͷ
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
                break;}
        
        case 1:{ //��ǰ��������
                if(now->hou == 1){
                    
                    direction_t = 1;  //�����ת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return direction_t;
                }
                if(now->left==1){
                
                    direction_t = 2;  //���ֱת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
                if(now->qian== 1){
                     
                    direction_t = 3;  //�����ת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
                if(now->right == 1){
                      
                    direction_t = 0;  //�����ת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
            break;}
        case 2:{ //��ǰ������qian
                if(now->left == 1){
                    
                    direction_t = 1;  //�����ת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return direction_t;
                }
                if(now->qian==1){
                    
                    direction_t = 2;  //���ֱת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
                if(now->right== 1){
                     
                    direction_t = 3;  //�����ת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
                if(now->hou == 1){
                     
                    direction_t = 0;  //��Ե�ת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
        break;}
        case 3:{ //��ǰ��������
            if(now->qian == 1){
                        
                    direction_t = 1;  //�����ת
                    go = now->next_qian;
                    now->qian_view = max_node_view();
                    last = now;
                    return direction_t;
                }
                if(now->right==1){
                    
                    direction_t = 2;  //���ֱת
                    go = now->next_right;
                    now->right_view = max_node_view();
                    last = now;					
                    return direction_t;
                }
                if(now->hou== 1){
                     
                    direction_t = 3;  //�����ת
                    go = now->next_hou;
                    now->hou_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
                if(now->left == 1){
                    
                    direction_t = 0;  //�����ת
                    go = now->next_left;
                    now->left_view = max_node_view();
                    last = now;
                    return  direction_t;
                }
        break;}
        
        default:
            break;
    }
    ///...........///
    //if (now->left == 1) {
    //		if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
    //			direction_t = 1;
    //			go = now->next_left;
    //			last = now;
    //			return direction_t;
    //		}
    //	}
    //	if (now->qian == 1) {
    //		if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)) {
    //			direction_t = 2;
    //			go = now->next_qian;
    //			last = now;
    //			return direction_t;
    //		}
    //	}
    //	if (now->right == 1) {
    //		if ((now->next_right == &save) || (p_t < now->next_right->no - 2)) {
    //			direction_t = 3;
    //			go = now->next_right;
    //			last = now;
    //			return direction_t;
    //		}
    //	}
    //	if (now->hou == 1) {
    //		if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)) {
    //			direction_t = 0;
    //			go = now->next_hou;
    //			last = now;
    //			return direction_t;
    //		}
    //	}
    //	now2 = now;
    //	if (now->left == 1) {
    //		if (now->no - 1 == now2->next_left->no) {
    //			direction_t = 1;
    //			go = now->next_left;
    //			last = now 	 ;
    ////			distance_t -= abs(now->x - go->x);
    ////			p_t--;
    //			return direction_t;
    //		}
    //	}
    //	if (now->qian == 1) {
    //		if (now->no - 1 == now2->next_qian->no) {
    //			direction_t = 2;
    //			go = now->next_qian;
    //			last = now;
    ////			distance_t -= abs(now->y - go->y);
    ////			p_t--;
    //			return direction_t;
    //		}
    //	}
    //	if (now->right == 1) {
    //		if (now->no - 1 == now2->next_right->no) {
    //			direction_t = 3;
    //			go = now->next_right;
    //			last = now;
    ////			distance_t -= abs(now->x - go->x);
    ////			p_t--;
    //			return direction_t;
    //		}
    //	}
    //	if (now->hou == 1) {
    //		if (now->no - 1 == now2->next_hou->no) {
    //			direction_t = 0;
    //			go = now->next_hou;
    //			last = now;
    ////			distance_t -= abs(now->y - go->y);
    ////			p_t--;
    //			return direction_t;
    //		}
    //	}
    ///...........//
        return 777;//����
    }
    
    
    
    int gogogo (float get_x, float get_y, int get_left, int get_qian, int get_right, int get_hou)
    {
    //	if (g_t >= 100) {
    //		g_t = 0;
    //	}
        if (find_exit2 == 1) {
            return 7;
        }
        g_t++;
        if(gogo[g_t]==7){
                STOP_FLAG =1;
        }
        return gogo[g_t];  //����ǵ�����յ��
    }
    
    //���ת��flag
    void  Clear_FLAG(void){
        cflag_right=0;
        cflag_left=0;
        cflag_up=0;
        cflag_tn180=0;
    }
    
    void calc_node_num(void){           
        int num=0;
        if (now->qian){
                num++;
        }
        if(now->hou){
            num++;
        }
        if(now->left){
                num++;
        }
        if(now->right){
            num++;
        }
        now->node_num = num;
    }
    
    //Ѱ�ҵ�ǰ�ڵ����ֵ�����ֵ
    int max_node_view(void){
        int temp=0;
        if(now->left_view > temp){
                 temp = now->left_view;
        }
        if(now->qian_view>temp){
                temp = now->qian_view;
        }
        if(now->right_view > temp){
            temp = now->right_view;
        }
        if(now->hou_view > temp){
                 temp = now->hou_view;
        }
        temp ++;
        return temp;
    }
    
    //���ط������С�ڵ�
    int get_min_node_view(void){
        int temp = 100,record = 1;
        if(now->left){
                if(temp > now->left_view){
                    record = 1;
                    temp = now->left_view;
                }
        }
        if(now->qian){
                if(temp >now->qian_view){
                        record = 2;
                        temp = now->qian_view;
                }
        }
        if(now->right){
                if(temp > now->right_view){
                    record = 3;
                    temp = now->right_view;
                }
        }
        if(now->hou){
                if(temp > now->hou_view){
                    record = 0;
                    temp = now->hou_view;
                }
        }
        return record;
    }
    
    int judge_node(struct map *node){
            uint8_t dir = direct;
            switch(dir%4){
                case 0:{
                    if(last->hou == 1&& last->left == 1&&last->right == 0&&last->qian ==0){  //�Ƕ��ڵ�L��·��
                        if(now->right == 1&&now-> hou == 1&&now->left == 0&&now->qian == 1){
                            return 1;
                        }}
                    else 
                        return 0;
                    break;}
                case 1:{
                    if(last->left == 1&& last->qian == 1&&last->right == 0&&last->hou ==0){  //�Ƕ��ڵ�L��·��
                        if(now->left == 1&&now-> hou == 1&&now->right == 1&&now->qian == 0){
                            return 1;
                        }}
                    else 
                        return 0;
                    break;}
                case 2:{
                    if (last->qian == 1&& last->right == 1&&last->left == 0&&last->hou ==0){  //�Ƕ��ڵ�L��·��
                        if(now->right == 0&&now-> hou == 1&&now->left == 1&&now->qian == 1){
                            return 1;
                        }}
                    else 
                        return 0;
                    break;}
                case 3:{
                    if(last->right == 1&& last->hou == 1&&last->left == 0&&last->qian ==0){  //�Ƕ��ڵ�L��·��
                        if(now->left == 1&&now-> hou == 0&&now->right == 1&&now->qian == 1){
                            return 1;   
                        }}
                    else 
                        return 0;
                    break;}
    
                    default :
                        return 0;
            }
                return 0;
        }