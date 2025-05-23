
#include "task.h"

u8 find_exit = 0, cflag_found = 0, cflag_foundd = 0;
int length = 0;
// #define yaw_error_allow 50 // 允许yaw误差5度
/*************************以下是关于构建地图的方法***************************/
#define yunxvwucha 30              // 允许误差    单位cm
int gogo[100], g_t;                // 最短路径，最终顺序取出，指针g_t为路径的长度 ,保存路径长度
int new_point, all_point, p_point; // 新点，所有点数量，在遍历循环判断是否重复时的当前点（另一个局部使用的循环变量）
int i_t, n_t, nomin;               // 循环变量（用于初始化所有的节点），循环变量，最小值
int p_t;                           // 最新点的标号
int direction_t = 2;               // 保存下一步移动的方向1：左，2：前，3：右，4：后（绝对方向，与绘制地图的方向
float distance_t, distance_min_t;  // 路径长度，最小路径长度
int startvelo = 40;
int last_turn_dir = -1;
JunctionType lastJunctionType = JUNCTION_NONE;
char *junctionNames[] = {"None", "Left", "Right", "Both", "Front", "Back", "End"};
// 在文件顶部添加
uint8_t last_junction_dirs = 0;  // 按位存储上一个路口可转向方向 (bit0:前, bit1:左, bit2:右, bit3:后)

struct map point[100];                                 // 存放地图节点数组
struct map save;                                       // 存放当前点信息
struct map *go, *now, *last, *now2, *head, *hp, *tail; // 下一个点，当前点，上一个点，当前点，头节点，当前点，尾节点

//***硬件相关变量/标志***//
uint8_t START_FLAG = 0;        // 开始标志
uint8_t STATE = 0, STATE2 = 0; // task函数state状态，初始状态为0
int find_exit2;                // 找到终点的标志位
int delay_count;               // 识别路口后延时计数

//***地图相关变量/标志***//
uint8_t cflag_right = 0, cflag_left = 0, cflag_tn180 = 0, cflag_stop = 0, cflag_up = 0; // 右左转标志，掉头，停止
uint8_t cflag_turn = 1;                                                                 // 允许转弯标志
extern uint8_t cflag_right, cflag_left;                                                 // 右左转标志
extern float LocX, LocY, last_locx, last_locy, RE_LocX, RE_LocY;                          // 位置参数 初始点为基准点，x轴距离代的距离参数   //用于记录临Locx和LocY
extern int direct;
// int yaw_record; //记录yaw值
// int yaw_target; //设定目标yaw值，用于转弯
// extern int yaw_adjust ;
// extern int yaw_int;
// uint8_t dir; //方向
extern uint8_t RECORED_DOWN; // 节点记录完成标志
                             // 终点标志
int ab_direction;            // 当前节点行进方向
uint8_t Start_Command;       // 调试命令可从中出改变
int node2_list[50];          // task2的转弯数组
int node2_num = 0;

void task1(void)
{
    switch (STATE)
    {
    case 0: // 初始化
    {
        cflag_turn = 1;
        start_point(); // 初始化节点
        STATE++;
        break;
    }

    case 1:                  // 等待启动、转状态
    {                        // 等待启动、转状态
        if (start_flag == 1) // 按键第二次确定启动
        {
            STATE++;
        }
        break;
    }

    case 2: // 收到开始开始行走状态
    {
        if (start_flag == 1)
        {
            start_flag = 0;
            // start_move(40);
            // OLED_ShowString(0,32,(uint8_t *)"start",8,1);
            // OLED_Refresh();
            gray_dir_allow = 1;
            gray_allow = 1;
            STATE++;
        }
        break;
    }

    case 3: // 巡线
    {       // 循环状态 1：识别终点  2：识别到黑复合道路
        start_move(startvelo);
        if (NODE_DETECT_FLAG) // 发现 节点
        {
            gray_dir_allow = 0; // 不允许灰度判断路口类型
            gray_allow = 0;     // 不允许灰度
            buzzerTurnOnDelay(10);
            error = 0;
            Locationhold(); // 修正坐标值（灰度位置和电机位置的差距）
            lock_Loc(); // 锁定坐标,避免减速带来的影响
            STATE++;
            // if (!TURN_BACK_FLAG)
            // {
            //     // delay_ms(200);                       // 往前走 //可能增加一个延时计数，定时器使用
            //     Locationhold();                      // 更新方向与节点参数
            //     STATE++;
            //     // int levels = read_infrared_sensor(); // 以下判断该路口前进方向是否可行
            //     // if (levels & 0b01110)
            //     // {                     // 此时levels不为零，说明前方有路
            //     //     TURN_UP_FLAG = 1; // 允许前进
            //     //     buzzerTurnOnDelay(10);
            //     //     STATE++;
            //     // }
            //     // else
            //     // {
            //     //     TURN_UP_FLAG = 0;
            //     //     STATE++;
            //     // }
            // }
            // else
            // {
            //     delay_ms(200);
            //     Locationhold(); // 更新方向与节点参数
            //     STATE++;
            // }
            // delay_count = 0; // 重置计数器
        }
        break;
    }

    case 4: // 判断为节点或者终点
    {
        /*****************第一阶段：判断路口类型并记录********************/
        NODE_DETECT_FLAG = 0; // 清除收节点标志
        last_junction_dirs = 0;
        // blue_node_num++;
        set_corner_dir(direct + 2); // 此时路口为通，原来方向可通
        if (STOP_FLAG == 1){
            find_exit2 = 1;
            // INTERRUPT_FLAG = 1;
            STATE++;
            Clear_NODEflag(); // 清除左右方向
        }else{
            if (TURN_BACK_FLAG == 1){ // 掉头
                TURN_BACK_FLAG = 0;
                last_junction_dirs |= (1<<3); // 后方可行
            }
            if (BOTH_FLAG == 1){ // 双
                BOTH_FLAG = 0;
                buzzerTurnOnDelay(10);
                set_corner_dir(direct + 1);
                set_corner_dir(direct - 1);
                last_junction_dirs |= (1<<1);  // 左方可行
                last_junction_dirs |= (1<<2); // 右方可行
            }
            if (TURN_LEFT_FLAG == 1){ // 左
                TURN_LEFT_FLAG = 0;
                set_corner_dir(direct - 1); // 设置当前左边的绝对方向可行
                last_junction_dirs |= (1<<1);  // 左方可行
            }
            if (TURN_UP_FLAG == 1){
                TURN_UP_FLAG = 0;
                set_corner_dir(direct);
                last_junction_dirs |= (1<<0);
            }
            if (TURN_RIGHT_FLAG == 1){ // 右
                TURN_RIGHT_FLAG = 0;
                set_corner_dir(direct + 1); // 设置当前右边的绝对方向可行
                last_junction_dirs |= (1<<2); // 右方可行
            }
            Clear_NODEflag();
            // char msg[64];
            // snprintf(msg, sizeof(msg), "NodeNo:%d, X:%d, Y:%d\r\n", now->no, now->x, now->y);
            // HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            // memset(msg, 0, sizeof(msg));
        }
        if (END_FLAG == 0){                                                                         // 如果没有到终点
            ab_direction = add_point(RE_LocY, RE_LocX, ab_fy, ab_x, ab_y, ab_fx); // 记录节点，并返回行驶方向（返回的是绝对方向
        }
        else{                                                                      // 到达终点
            ab_direction = gogogo(RE_LocY, RE_LocX, ab_fy, ab_x, ab_y, ab_fx); // 回溯路径
        }
        clear_corner_dir();
        // 绝对方向转换为当前转向（绝对方向
        if (END_FLAG == 0){ // 找到终点前的转弯
            switch (ab_direction){ 
                case turnLeft:{ // 向左 1
                    lastJunctionType = JUNCTION_LEFT;
                    stop_move(-500);
                    cflag_left = 1;
                    break;
                }
                case turnRight:{ // 向右  3
                    // printf("turn_right\r\n");
                    lastJunctionType = JUNCTION_RIGHT;
                    stop_move(-500);
                    cflag_right = 1;
                    break;
                }
                case turnBack:{ // 后退
                    lastJunctionType = JUNCTION_BACK;
                    cflag_tn180 = 1;
                    stop_move(2000);
                    break;
                }
                case turnUp:{
                    lastJunctionType = JUNCTION_FRONT;
                    cflag_up = 1;
                    break;
                }
                default:break;
            }
        }
        else{
            switch (get_dir(ab_direction)){
                case turnLeft:{ // 向左 1
                    lastJunctionType = JUNCTION_LEFT;
                    stop_move(0);
                    cflag_left = 1;
                    break;
                }
                case turnRight:{ // 向右  3
                    lastJunctionType = JUNCTION_RIGHT;
                    stop_move(0);
                    cflag_right = 1;
                    break;
                }
                case turnBack:{ // 后退
                    lastJunctionType = JUNCTION_BACK;
                    stop_move(2000);
                    cflag_tn180 = 1;
                    break;
                }
                case turnUp:{
                    lastJunctionType = JUNCTION_FRONT;
                    cflag_up = 1;
                    break;
                }
                default:break;
                }
        }
        /****************第二阶段：根据记录的路口，计算一个行驶方向并执行转弯***************/
        // 执行转动作
        // printf("go:%d\r\n", go->no);
        if (STOP_FLAG == 0)
        { // 没收到停止标识  、或者节点不是终点时继续走
            //***执行各转动作***//  实际测试，由玩车的安装方式导致转角度比小
            if (cflag_left == 1)
            {
                Turn_left();
            }
            else if (cflag_right == 1)
            {
                Turn_right();
            }
            else if (cflag_tn180 == 1)
            {
                Turn_back();
            }
            else if (cflag_up == 1)
            {
                Turn_up();
            }
            Clear_FLAG();
            if (turn_in_progress == 0)
            {
                STATE = 3; // 回到循环状态
                buzzerTurnOnDelay(10);
                start_move(startvelo);
            }
            gray_allow = 1;
            gray_dir_allow = 1; // 允许灰度
        }
        else
        { // 到终点时，不需要置cflag，暂停turn，只用黑边
            // lock_Loc(); //锁定终点位置
            stop_move(1500);
            // printf("STOP");
        }
        break;
    }

    case 5:{ // 到达终点置位置标志
        END_FLAG = 1;
        STATE = 5;
        gray_dir_allow = 0;
        gray_allow = 0;
        error = 0;
        cflag_turn = 1; // 停止巡黑边
        // INTERRUPT_FLAG = 1;
        // Set_Pwm(0, 0);
        // find_exit2 = 1;
        // STOP_FLAG = 0;
        // buzzerTurnOnDelay(100);
        start_flag = 0;
        STATE++;
        break;
    }

    case 6:{ // 到达终点等待启动状态
        if (start_flag == 1)
        {
            start_flag = 0;
            STATE++;
        }
        break;
    }

    case 7:{ // 从终点脱离
      // printf("state7\r\n");
        STOP_FLAG = 0;
        find_exit2 = 0;
        // judge_dir();
        //  NODE_DETECT_FLAG=1;//重置终点标志
        // lock_Loc(); // 这里和turn函数back中也都有用，所以在此调用
        // Set_Pwm(0, 0);
        // stop_move(-3000);
        Clear_levels();
        Turn_back();
        gray_dir_allow = 1;
        gray_allow = 1;
        // INTERRUPT_FLAG = 0;
        // PID_init();
        cflag_turn = 0;
        // judge_dir();
        STATE = 3;
        break;
    }

    case 8:{ // 从终点返回巡线
        break;
    }

    default:break;
    }
}

//****处理各个路口判断进行继续行走****//
int level;
u8 wasteTime;
void Turn_right(void){ // 右转
    last_turn_dir = 3;
    cflag_turn = 1;
    // lock_Loc();
    
    changeTurnAgle(90);
    while (turn_in_progress)
    {
        delay_ms(1);
    }; // 等待转弯完成
    direct++;
    buzzerTurnAndClose();
    lock_Loc();
    start_move(startvelo);
    gray_allow = 1;  //允许灰度读误差但不读路口
    gray_dir_allow = 0;
    delay_count_10ms = 0;
    while(delay_count_10ms<130){
        delay_ms(1);
    };
    cflag_turn = 0;
    // start_move(40);
    // gray_allow = 1;  //允许灰度读误差但不读路口
    // gray_dir_allow = 0;
    // delay_ms(500);
}

void Turn_left(void){ // 左转
    last_turn_dir = 1;
    cflag_turn = 1;
    // lock_Loc();
    changeTurnAgle(-90);
    while (turn_in_progress){
        delay_ms(1);
    }; // 等待转弯完成
    direct--;
    buzzerTurnAndClose();
    lock_Loc();
    start_move(startvelo);
    gray_allow = 1;  //允许灰度读误差但不读路口
    gray_dir_allow = 0;
    delay_count_10ms = 0;
    while(delay_count_10ms<130){
        delay_ms(1);
    };
    cflag_turn = 0;
    // start_move(40);
    // gray_allow = 1;  //允许灰度读误差但不读路口
    // gray_dir_allow = 0;
    // delay_ms(500);
}

void Turn_back(void){
    last_turn_dir = 2;
    cflag_turn = 1;
    // lock_Loc();
    
    changeTurnAgle(180);
    while (turn_in_progress){
       delay_ms(1);
    }; // 等待转弯完成
    direct += 2;
    buzzerTurnAndClose();
    lock_Loc();
    // Set_Pwm(0, 0);
    start_move(startvelo);
    gray_allow = 1;  //允许灰度读误差但不读路口
    gray_dir_allow = 0;
    delay_count_10ms = 0;
    while(delay_count_10ms<170){
        delay_ms(1);
    };
    cflag_turn = 0;
}

void Turn_up(void){
    last_turn_dir = 0;
    cflag_turn = 1;
    lock_Loc();
    delay_ms(10);
    cflag_turn = 0;
}

void start_point(void){
    point[0].x = 0;
    point[0].y = 0;
    point[0].qian = 0;
    point[0].hou = 0;
    point[0].left = 0;
    point[0].right = 0;
    point[0].node_num = 0;
    point[0].qian_view = 0;
    point[0].hou_view = 0;
    point[0].left_view = 0;
    point[0].right_view = 0;
    point[0].leftAccessible = 0;
    point[0].rightAccessible = 0;
    point[0].qianAccessible = 0;
    point[0].houAccessible = 0;
    point[0].no = 0;
    point[0].next_qian = &save;
    point[0].next_hou = &save;
    point[0].next_left = &save;
    point[0].next_right = &save;
    point[0].next = &save;
    save = point[0];
    save.qian = 0;
    point[0].qian = 1;
    point[0].node_num = 1;
    point[0].qian_view = 1;
    point[0].qianAccessible = 1;
    all_point = 1;            // 此时所有的点数量 就只有1个 也就是第一个点
    direction_t = 2;          // 方向为2，朝前，初始的绝对方向为向前
    find_exit2 = 0;           // 找到终点的标志位为0
    distance_t = 0;           // 路径长度为0
    distance_min_t = 9999999; // 最小路径长度为9999999
    p_t = 0;                  // 当前点的标号为0
    i_t = 0;                  // 循环变量为0
    for (; i_t < 100; i_t++)
    {
        point[i_t] = save;
    }
    last = &point[0];     // 上一个点为第一个点 因为第一个节点没有上一个点
    now = &point[0];      // 当前点为第一个点
    go = last->next_qian; // 当前点为第一个点的前方的下一个点 ,此时的go==save
    for (i_t = 0; i_t < 100; i_t++)
    {
        gogo[i_t] = 7; // 7表示无用
    }
    
    buzzerTurnOnDelay(50);
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
int add_point(float get_y, float get_x, int get_left, int get_qian, int get_right, int get_hou){
    direction_t = direct;
    if (go == &save){ // save是map用于初始化的结构体，所以当go==一个初始化节点，即未记录的节点时
        // printf("save\r\n");
        p_point = 0;
        // 遍历所有点，检查是否存在相同坐标的点
        while (p_point < all_point - 1){
            if ((get_x > (point[p_point].x - yunxvwucha)) && (get_x < (point[p_point].x + yunxvwucha)))
            { // get_x 在当前点的误差范围内
                if ((get_y > (point[p_point].y - yunxvwucha)) && (get_y < (point[p_point].y + yunxvwucha)))
                { // 在某个点的误差范围内
                    // printf("cf\r\n");
                    last_locx = get_y; // 记录最新的坐标
                    last_locy = get_x; // last_locy代表暂存的坐标
                    if (p_t >= point[p_point].no - 1){ // 最新点的编号大于等于当前点的编号
                        last = now;     
                        now = &point[p_point]; // 当前点更新为找到的点
                        now->x = get_x;
                        now->y = get_y;
                        // go = last; // 掉头，go即目标点变成下一个点
                        // printf("dir:%d", direction_t);
                        switch (direction_t % 4){ // 判断上一次的转向方向，根据上次的转向方向记录当前点（重复点）和上一个点的连接关系
                             case 0:
                                last->next_hou = now;
                                now->next_qian = last;
                                now->qian_view = max_node_view();
                                // direction_t = 0;
                                distance_t += ABS_int(now->y - last->y);
                                break;
                            case 1:
                                last->next_left = now;
                                now->next_right = last;
                                now->right_view = max_node_view();
                                // direction_t = 0;
                                distance_t += ABS_int(now->x - last->x);
                                break;
                            case 2:
                                last->next_qian = now; // 上一个点的前方的下一个点为当前点
                                now->next_hou = last;  // 相反的，这个点的后方的下一个点为上一个点
                                // direction_t = 0;					//方向为后
                                now->hou_view = max_node_view();
                                distance_t += ABS_int(now->y - last->y);
                                break;
                            case 3:
                                last->next_right = now;
                                now->next_left = last;
                                now->left_view = max_node_view();
                                // direction_t = 0;
                                distance_t += ABS_int(now->x - last->x);
                                break;
                            default:
                                break;
                        }  
                        // setPointer0AccessibleZero();
                        // int t = get_min_node_view(direct);  //获取一个值最小的方向,绝对方向；

                        // int relativbeDir = (t+6-direct)%4; // 计算相对方向,即要转弯的方向
                        // switch(t){   //置位对应的方向访问值
                        //     case 0:
                        //         go = now->next_hou;
                        //         now->hou_view = max_node_view();
                        //         break;
                        //     case 1:
                        //         go = now->next_left;
                        //         now->left_view = max_node_view();
                        //         break;
                        //     case 2:
                        //         go = now->next_qian;
                        //         now->qian_view = max_node_view();
                        //         break;
                        //     case 3:
                        //         go = now->next_right;
                        //         now->right_view = max_node_view();
                        //         break;
                        //     default:
                        //         break;
                        // }
                        int t = get_min_node_view(direct);  //获取一个值最小的方向
                        switch (direct % 4){
                        case 0:
                        {
                            if (t == 0)
                            {
                                direction_t = 2;
                                go = now->next_hou;
                                now->hou_view = max_node_view();
                                break;
                            }
                            if (t == 1)
                            {
                                direction_t = 3;
                                go = now->next_left;
                                now->left_view = max_node_view();
                                break;
                            }
                            if (t == 2)
                            {
                                direction_t = 0;
                                go = now->next_qian;
                                now->qian_view = max_node_view();
                                break;
                            }
                            if (t == 3)
                            {
                                direction_t = 1;
                                go = now->next_right;
                                now->right_view = max_node_view();
                                break;
                            }
                        }
                        case 1:
                        {
                            if (t == 0)
                            {
                                direction_t = 1;
                                go = now->next_hou;
                                now->hou_view = max_node_view();
                                break;
                            }
                            if (t == 1)
                            {
                                direction_t = 2;
                                go = now->next_left;
                                now->left_view = max_node_view();
                                break;
                            }
                            if (t == 2)
                            {
                                direction_t = 3;
                                go = now->next_qian;
                                now->qian_view = max_node_view();
                                break;
                            }
                            if (t == 3)
                            {
                                direction_t = 0;
                                go = now->next_right;
                                now->right_view = max_node_view();
                                break;
                            }
                        }
                        case 2:
                        {
                            if (t == 0)
                            {
                                direction_t = 0;
                                go = now->next_hou;
                                now->hou_view = max_node_view();
                                break;
                            }
                            if (t == 1)
                            {
                                direction_t = 1;
                                go = now->next_left;
                                now->left_view = max_node_view();
                                break;
                            }
                            if (t == 2)
                            {
                                direction_t = 2;
                                go = now->next_qian;
                                now->qian_view = max_node_view();
                                break;
                            }
                            if (t == 3)
                            {
                                direction_t = 3;
                                go = now->next_right;
                                now->right_view = max_node_view();
                                break;
                            }
                        }
                        case 3:
                        {
                            if (t == 0)
                            {
                                direction_t = 3;
                                go = now->next_hou;
                                now->hou_view = max_node_view();
                                break;
                            }
                            if (t == 1)
                            {
                                direction_t = 0;
                                go = now->next_left;
                                now->left_view = max_node_view();
                                break;
                            }
                            if (t == 2)
                            {
                                direction_t = 1;
                                go = now->next_qian;
                                now->qian_view = max_node_view();
                                break;
                            }
                            if (t == 3)
                            {
                                direction_t = 2;
                                go = now->next_right;
                                now->right_view = max_node_view();
                                break;
                            }
                            }
                        }
                        return direction_t; // 返回转向方向
                    }
                }
            }
            p_point++;
        }
        // 如果点的数量达到上限，返回错误码
        if (all_point == 100)
        { // 如果所有节点数量达到上限100个，返回报错99
            return 99;
        }
        // 上述循环没找到重复点， 添加新点
        last =now;
        new_point = all_point;   // 上面的判断是否以及存在不成立，即不存在，可以添加该节点，该节点号为当前数量（因为节点0的时候总数为1，即ALL=new+1）
        all_point++;             // 所有节点的数量加1
        now = &point[new_point]; // map类型的now指针指向当前节点
        now->x = get_x;          // 记录当前节点的x坐标
        now->y = get_y;          // 记录当前节点的y坐标
        now->qian = get_qian;    // 记录当前节点是否可以向前走
        now->hou = get_hou;      // 记录当前节点是否可以向后走
        now->left = get_left;
        now->right = get_right;
        now->qianAccessible = get_qian;  //新创建节点时，设置对应可访问方向
        now->houAccessible = get_hou;
        now->leftAccessible = get_left;
        now->rightAccessible = get_right;
        calc_node_num();
        p_t++;         // 当前节点编号加1
        now->no = p_t; // 当前节点的编号为当前编号
        if(now->no == 1){
            now->houAccessible = 0;
            now->hou_view = 999;
        } 
        printf("new_point:%d", p_t);
        // 根据当前方向更新路径信息
        // judge_dir();
        switch (direction_t % 4){                                            // 根据上一次移动的方向更新路径信息
            case 0:
                last->next_hou = now;
                now->next_qian = last;
                now->qian_view = max_node_view();
                distance_t += ABS_int(now->y - last->y);
                break;
            case 1:                                      // 如果上次转向方向为左
                last->next_left = now;                   // 上一个点的左方的下一个点为当前点
                now->next_right = last;                  // 当前点的右方的下一个点为上一个点
                now->right_view = max_node_view();       // 当前点的右方访问过
                distance_t += ABS_int(now->x - last->x); // 路径长度加上当前点的x坐标减去上一个点的x坐标的绝对值
                break;
            case 2:
                last->next_qian = now;
                now->next_hou = last;
                now->hou_view = max_node_view();
                distance_t += ABS_int(now->y - last->y);
                break;
            case 3:
                last->next_right = now;
                now->next_left = last;
                now->left_view = max_node_view();
                distance_t += ABS_int(now->x - last->x);
                break;
            
        }
        // 如果找到终点，更新路径信息
        if (find_exit2 == 1){ // 找到终点了
            // printf("find\r\n");
            find_exit2 = 0;
            cflag_foundd = 1;
            cflag_found = 1;
            g_t = 0;
            if (distance_t < distance_min_t)
            {
                distance_min_t = distance_t;
                tail = now;
                n_t = p_t;
                // all_point --;
                while (1){
                    p_point = 0;
                    nomin = n_t;
                    while (p_point < all_point)
                    {
                        if ((point[p_point].next_hou == tail) || (point[p_point].next_right == tail) || (point[p_point].next_qian == tail) || (point[p_point].next_left == tail))
                        {
							// if(point[p_point].node_num == 1&&(p_point !=0) ){
							// 		continue;//如果这个节点是一个死胡同节点，直接跳过
							// }
                            if (point[p_point].no < nomin)
                            {
                                nomin = point[p_point].no;
                                point[p_point].next = tail;
                                hp = &point[p_point];
                            }
                        }
                        p_point++;
                    }
                    n_t = nomin;
                    tail = hp;
                    if (hp->next_left == tail->next)
                    {
                        gogo[g_t] = 3;
                    }
                    if (hp->next_qian == tail->next)
                    {
                        gogo[g_t] = 0;
                    }
                    if (hp->next_right == tail->next)
                    {
                        gogo[g_t] = 1;
                    }
                    if (hp->next_hou == tail->next)
                    {
                        gogo[g_t] = 2;
                    }
                    length++;
                    g_t++;
                    if (tail == &point[0])
                    {
                        break;
                    }
                }
                tail = &point[1];
                hp = tail;
                g_t = 0;
                printf("p_t:%d\r\n", p_t); // pt=4

                for (int i = 0; gogo[i] != 7; i++)
                {
                    printf("%d", gogo[i]);
                }
                // printf("\n");
                buzzerTurnOnDelay(200);
                return 7;
            }
        
        }
    }
    else{ // 还没到达终点  // 以下是go记录过，即go！=save初始值的情况
        if (p_t < go->no - 1){  //
            if (all_point == 100){
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
            now->qianAccessible = get_qian;
            now->houAccessible = get_hou;
            now->leftAccessible = get_left;
            now->rightAccessible = get_right;
            calc_node_num();
            p_t++;
            now->no = p_t;
            // 根据当前方向更新路径信息
            switch (direction_t % 4){
            case 1:
                last->next_left = now;
                now->next_right = last;
                now->right_view = max_node_view();
                distance_t += ABS_int(now->x - last->x);
                break;
            case 2:
                last->next_qian = now;
                now->next_hou = last;
                now->hou_view = max_node_view();
                distance_t += ABS_int(now->y - last->y);
                break;
            case 3:
                last->next_right = now;
                now->next_left = last;
                now->left_view = max_node_view();
                distance_t += ABS_int(now->x - last->x);
                break;
            case 0:
                last->next_hou = now;
                now->next_qian = last;
                now->qian_view = max_node_view();
                distance_t += ABS_int(now->y - last->y);
                break;
            }
        }
        else{  //当前点是save过的老点，探索到死胡同掉头后的情况
            last_locx = go->x;
            last_locy = go->y;
            p_t--;
            last->no++;
            now = go;
            // now->x = get_x;
            // now->y =get_y;
            printf("delete\r\n");
            // 根据当前方向更新距离
            switch (direction_t % 4)
            {
                case 1:
                    last->leftAccessible = 0;
                    distance_t -= ABS_int(now->x - last->x);
                    break;
                case 2:
                    last->qianAccessible = 0;
                    distance_t -= ABS_int(now->y - last->y);
                    break;
                case 3:
                    last->rightAccessible = 0;
                    distance_t -= ABS_int(now->x - last->x);
                    break;
                case 0:
                    last->houAccessible = 0;
                    distance_t -= ABS_int(now->y - last->y);
                    break;
            }
        }
    }


    if (judge_node(now) == 1){
        // 是特殊节点
        switch (direct % 4){
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

    // u8 t = get_min_node_view();
    // u8 relativbeDir = (2-direct+t+4)%4; // 计算相对方向,即要转弯的方向
    // switch (t)
    // {
    //     case 0:
    //         go = now->next_hou;
    //         now->hou_view = max_node_view();
    //         last = now;
    //         return relativbeDir;
    //     case 1:
    //         go = now->next_left;
    //         now->left_view = max_node_view();
    //         last = now;
    //         return relativbeDir;
    //     case 2:
    //         go = now->next_qian;
    //         now->qian_view = max_node_view();
    //         last = now;
    //         return relativbeDir;
    //     case 3:
    //         go = now->next_right;
    //         now->right_view = max_node_view();
    //         last = now;
    //         return relativbeDir;

    // }

    switch (direct % 4){ 
    case 0:{ // 当前方向向后
        if (now->right == 1 && now->rightAccessible){
            if ((now->next_right == &save) || (p_t < now->next_right->no - 2 )){//右边是save代表没有探索过，
                direction_t = 1; // 相对左转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->hou == 1 && now->houAccessible){
            if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2 )){
                direction_t = 2; // 相对直转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->left == 1 && now->leftAccessible){
            if ((now->next_left == &save) || (p_t < now->next_left->no - 2 )){
                direction_t = 3; // 相对右转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->qian == 1 && now->qianAccessible){
            if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)){
                direction_t = 0; // 相对掉头
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        break;
    }

    case 1:{ // 当前方向向左
        if (now->hou == 1  && now->houAccessible){
            if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)){
                direction_t = 1; // 相对左转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->left == 1 && now->leftAccessible){
            if ((now->next_left == &save) || (p_t < now->next_left->no - 2)){
                direction_t = 2; // 相对直转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->qian == 1 && now->qianAccessible){
            if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2)){
                direction_t = 3; // 相对右转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->right == 1 && now->rightAccessible){
            if ((now->next_right == &save) || (p_t < now->next_right->no - 2)){
                direction_t = 0; // 相对掉头
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        break;
    }
    case 2:{ // 当前方向向qian
        if (now->left == 1 && now->leftAccessible){
            if ((now->next_left == &save) || (p_t < now->next_left->no - 2 )){
                direction_t = 1; // 相对左转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->qian == 1 && now->qianAccessible){
            if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2 )){
                direction_t = 2; // 相对直转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->right == 1 && now->rightAccessible){
            if ((now->next_right == &save) || (p_t < now->next_right->no - 2 )){
                direction_t = 3; // 相对右转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->hou == 1 && now->houAccessible){
            if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)){
                direction_t = 0; // 相对掉头
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        break;
    }
    case 3:{ // 当前方向向右
        if (now->qian == 1 && now->qianAccessible){
            if ((now->next_qian == &save) || (p_t < now->next_qian->no - 2 )){
                direction_t = 1; // 相对左转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->right == 1 && now->rightAccessible){
            if ((now->next_right == &save) || (p_t < now->next_right->no - 2)){
                direction_t = 2; // 相对直转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->hou == 1 && now->houAccessible){
            if ((now->next_hou == &save) || (p_t < now->next_hou->no - 2)){
                direction_t = 3; // 相对右转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->left == 1  && now->leftAccessible){
            if ((now->next_left == &save) || (p_t < now->next_left->no - 2)){
                direction_t = 0; // 相对掉头
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        break;
    }

    default:
        break;
    }
    now2 = now;
    switch (direct % 4){  //删除单边死胡同
    case 0:{ // 当前方向向后
        if (now->right == 1 && now->rightAccessible){
            if (now->no - 1 == now2->next_right->no){
                direction_t = 1; // 相对左转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->hou == 1 && now->houAccessible){
            if (now->no - 1 == now2->next_hou->no && now->no - 1 != 0){
                direction_t = 2; // 相对直转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->left == 1 && now->leftAccessible){
            if (now->no - 1 == now2->next_left->no && now->no - 1 != 0){
                direction_t = 3; // 相对右转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->qian == 1 && now->qianAccessible){
            if (now->no - 1 == now2->next_qian->no && now->no - 1 != 0){
                direction_t = 0; // 相对掉头
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        break;
    }

    case 1:{ // 当前方向向左
        if (now->hou == 1   && now->houAccessible){
            if (now->no - 1 == now2->next_hou->no && now->no - 1 != 0){
                direction_t = 1; // 相对左转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->left == 1 && now->leftAccessible){
            if (now->no - 1 == now2->next_left->no && now->no - 1 != 0){
                direction_t = 2; // 相对直转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->qian == 1 && now->qianAccessible){
            if (now->no - 1 == now2->next_qian->no && now->no - 1 != 0){
                direction_t = 3; // 相对右转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->right == 1 && now->rightAccessible){
            if (now->no - 1 == now2->next_right->no && now->no - 1 != 0){
                direction_t = 0; // 相对右转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        break;
    }
    case 2:{ // 当前方向向qian
        if (now->left == 1 && now->leftAccessible){
            if (now->no - 1 == now2->next_left->no  && now->no - 1 != 0){
                direction_t = 1; // 相对左转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->qian == 1 && now->qianAccessible){
            if (now->no - 1 == now2->next_qian->no  && now->no - 1 != 0){
                direction_t = 2; // 相对直转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->right == 1 && now->rightAccessible){
            if (now->no - 1 == now2->next_right->no  && now->no - 1 != 0){
                direction_t = 3; // 相对右转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->hou == 1 && now->houAccessible){
            if ((now->no - 1 == now2->next_hou->no  && now->no - 1 != 0)){
                direction_t = 0; // 相对掉转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        break;
    }
    case 3:{ // 当前方向向右
        if (now->qian == 1 && now->qianAccessible){
            if ((now->no - 1 == now2->next_qian->no  && now->no - 1 != 0)){
                direction_t = 1; // 相对左转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->right == 1 && now->rightAccessible){
            if ((now->no - 1 == now2->next_right->no  && now->no - 1 != 0)){
                direction_t = 2; // 相对直转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->hou == 1 && now->houAccessible){
            if (now->no - 1 == now2->next_hou->no  && now->no - 1 != 0){
                direction_t = 3; // 相对右转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        if (now->left == 1 && now->leftAccessible){
            if (now->no - 1 == now2->next_left->no  && now->no - 1 != 0){
                direction_t = 0; // 相对右转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
        }
        break;
    }

    default:
        break;
    }

    switch (direct % 4){
        case 0:{ // 当前方向向后
            if (now->right == 1 && now->rightAccessible){
                direction_t = 1; // 相对左转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->hou == 1 && now->houAccessible){
                
            }
            {

                direction_t = 2; // 相对直转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->left == 1 && now->leftAccessible)
            {

                direction_t = 3; // 相对右转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->qian == 1 && now->qianAccessible)
            {

                direction_t = 3; // 相对右转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->qian == 1  && now->qianAccessible)
            {

                direction_t = 0; // 相对掉头
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
            break;
        }

        case 1:
        { // 当前方向向左
            if (now->hou == 1 && now->houAccessible)
            {

                direction_t = 1; // 相对左转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->left == 1 && now->leftAccessible)
            {

                direction_t = 2; // 相对直转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->qian == 1 && now->qianAccessible)
            {

                direction_t = 3; // 相对右转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->right == 1 && now->rightAccessible)
            {

                direction_t = 0; // 相对右转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
            break;
        }
        case 2:
        { // 当前方向向qian
            if (now->left == 1 && now->leftAccessible)
            {

                direction_t = 1; // 相对左转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->qian == 1 && now->qianAccessible)
            {

                direction_t = 2; // 相对直转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->right == 1 && now->rightAccessible)
            {

                direction_t = 3; // 相对右转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->hou == 1 && now->houAccessible)
            {

                direction_t = 0; // 相对掉转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
            break;
        }
        case 3:
        { // 当前方向向右
            if (now->qian == 1  && now->qianAccessible)
            {
                direction_t = 1; // 相对左转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->right == 1 && now->rightAccessible)
            {

                direction_t = 1; // 相对左转
                go = now->next_qian;
                now->qian_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->right == 1 && now->rightAccessible)
            {

                direction_t = 2; // 相对直转
                go = now->next_right;
                now->right_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->hou == 1 && now->houAccessible)
            {

                direction_t = 3; // 相对右转
                go = now->next_hou;
                now->hou_view = max_node_view();
                last = now;
                return direction_t;
            }
            if (now->left == 1 && now->leftAccessible)
            {

                direction_t = 0; // 相对右转
                go = now->next_left;
                now->left_view = max_node_view();
                last = now;
                return direction_t;
            }
            break;
        }

        default:
            break;
    }
    ///...........///
    // if (now->left == 1) {
    //		if ((now->next_left == &save) || (p_t < now->next_left->no - 2)) {
    //			direction_t = 1;
    //			go = now->next_left;
    //			last = now;
    //			return direction_t;
    //		}
    //	
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
    ////			distance_t -= ABS_int(now->x - go->x);
    ////			p_t--;
    //			return direction_t;
    //		}
    //	}
    //	if (now->qian == 1) {
    //		if (now->no - 1 == now2->next_qian->no) {
    //			direction_t = 2;
    //			go = now->next_qian;
    //			last = now;
    ////			distance_t -= ABS_int(now->y - go->y);
    ////			p_t--;
    //			return direction_t;
    //		}
    //	}
    //	if (now->right == 1) {
    //		if (now->no - 1 == now2->next_right->no) {
    //			direction_t = 3;
    //			go = now->next_right;
    //			last = now;
    ////			distance_t -= ABS_int(now->x - go->x);
    ////			p_t--;
    //			return direction_t;
    //		}
    //	}
    //	if (now->hou == 1) {
    //		if (now->no - 1 == now2->next_hou->no) {
    //			direction_t = 0;
    //			go = now->next_hou;
    //			last = now;
    ////			distance_t -= ABS_int(now->y - go->y);
    ////			p_t--;
    //			return direction_t;
    //		}
    //	}
    ///...........//
    return 777; // 报错
}

int gogogo(float get_x, float get_y, int get_left, int get_qian, int get_right, int get_hou)
{
    //	if (g_t >= 100) {
    //		g_t = 0;
    //	}
    if (find_exit2 == 1)
    {
        return 7;
    }
    g_t++;
    if (gogo[g_t] == 7)
    {
        STOP_FLAG = 1;
    }
    return gogo[g_t]; // 这个是到达过终点后
}

// 清除转弯flag
void Clear_FLAG(void)
{
    cflag_right = 0;
    cflag_left = 0;
    cflag_up = 0;
    cflag_tn180 = 0;
}

void calc_node_num(void){
    int num = 0;
    if (now->qian)
    {
        num++;
    }
    if (now->hou)
    {
        num++;
    }
    if (now->left)
    {
        num++;
    }
    if (now->right)
    {
        num++;
    }
    now->node_num = num;
}

// 寻找当前节点最大值，返回值
int max_node_view(void){
    int temp = 0;
    if (now->left_view > temp)
    {
        temp = now->left_view;
    }
    if (now->qian_view > temp)
    {
        temp = now->qian_view;
    }
    if (now->right_view > temp)
    {
        temp = now->right_view;
    }
    if (now->hou_view > temp)
    {
        temp = now->hou_view;
    }
    temp++;
    return temp;
}

// 返回方向的最小节点,且该方向不是死胡同
int get_min_node_view(int direct){
    int temp = 100, record = 1;
    if (now->left && now->leftAccessible == 1){
        if (temp > now->left_view && now->left_view != 999){
            record = 1;
            temp = now->left_view;
        }
    }
    if (now->qian && now->qianAccessible == 1){
        if (temp > now->qian_view && now->qian_view != 999){
            record = 2;
            temp = now->qian_view;
        }
    }
    if (now->right && now->rightAccessible == 1){
        if (temp > now->right_view && now->right_view!= 999){
            record = 3;
            temp = now->right_view;
        }
    }
    if (now->hou && now->houAccessible == 1) {
        if (temp > now->hou_view && now->hou_view!= 999){
            record = 0;
            temp = now->hou_view;
        }
    }
    return record;
}

int judge_node(struct map *node)
{
    uint8_t dir = direct;
    switch (dir % 4)
    {
    case 0:
    {
        if (last->hou == 1 && last->left == 1 && last->right == 0 && last->qian == 0)
        { // 判断节点L型路口
            if (now->right == 1 && now->hou == 1 && now->left == 0 && now->qian == 1)
            {
                return 1;
            }
        }
        else
            return 0;
        break;
    }
    case 1:
    {
        if (last->left == 1 && last->qian == 1 && last->right == 0 && last->hou == 0)
        { // 判断节点L型路口
            if (now->left == 1 && now->hou == 1 && now->right == 1 && now->qian == 0)
            {
                return 1;
            }
        }
        else
            return 0;
        break;
    }
    case 2:
    {
        if (last->qian == 1 && last->right == 1 && last->left == 0 && last->hou == 0)
        { // 判断节点L型路口
            if (now->right == 0 && now->hou == 1 && now->left == 1 && now->qian == 1)
            {
                return 1;
            }
        }
        else
            return 0;
        break;
    }
    case 3:
    {
        if (last->right == 1 && last->hou == 1 && last->left == 0 && last->qian == 0)
        { // 判断节点L型路口
            if (now->left == 1 && now->hou == 0 && now->right == 1 && now->qian == 1)
            {
                return 1;
            }
        }
        else
            return 0;
        break;
    }

    default:
        return 0;
    }
    return 0;
}

void setPointer0AccessibleZero(void){
    static u8 zeroflag = 0;
    if(zeroflag == 1){
        return;
    }
    if(now->qian == 1)
    {
        if(now->next_qian->no == 0)
        {
            now->qianAccessible = 0;
            zeroflag = 1;
            return;
        }
    }
    if(now->left == 1)
    {
        if(now->next_left->no == 0)
        {
            now->leftAccessible = 0;
            zeroflag = 1;
            return;
        }
    }
    if(now->right == 1)
    {
        if(now->next_right->no == 0)
        {
            now->rightAccessible = 0;
            zeroflag = 1;
            return;
        }
    }
    if(now->hou == 1)
    {
        if(now->next_hou->no == 0)
        {
            now->houAccessible = 0;
            zeroflag = 1;
            return;
        }
    }
}

void task2(void){
    // if(send_data_flag){
    //     send_data_flag = 0;
    //     ANO_Direct_distance(direct,blue_node_num);
    // }

    switch (STATE2)
    {
    case 0:{ // 初始化
        // start_point(); // 初始化节点    
        STATE2++;
        break;
    }

    case 1:{ // 等待启动、转状态
    if (start_flag == 1){
        STATE2++;
    }
    break;
    }

    case 2:
    { // 收到开始开始行走状态
    if (start_flag == 1)
    {
        start_flag = 0;
        gray_dir_allow = 1;
        gray_allow = 1;
        STATE2++;
    }
    break;
    }

    case 3:{ // 循环状态 1:检测障碍物 2：识别终点  3：识别到黑复合道路
        start_move(40);
        if (NODE_DETECT_FLAG){
            gray_dir_allow = 0; // 不允许灰度判断路口类型
            gray_allow = 0;     // 不允许灰度
            buzzerTurnOnDelay(10);
            error = 0;
            Locationhold(); // 修正坐标值（灰度位置和电机位置的差距）
            STATE2++;
        }
        break;
    }

    case 4:
    {                         // 循环接收节点，记录并执行转弯
        NODE_DETECT_FLAG = 0; // 清除收节点标志
        if(node2_list[node2_num])
        {
            switch (node2_list[node2_num++]){            //get_dir(ab_direction)
            case 1:{ //向左 1
                stop_move(0);
                cflag_left =1;
                break;}	
            case 3:{ //向右  3
                stop_move(0);
                cflag_right = 1;
                break;}	
            case 2:{
                // stop_move(0);
                cflag_up=1;				
                break; }
            default:
                break;}
        }else if(node2_list[node2_num] == 0){
            STOP_FLAG = 1;	
        }
        if(STOP_FLAG == 1){
            STATE2 =5;
            Clear_NODEflag();//需测试是否会有误
        }
        if(STOP_FLAG==0){  //没有终点停止标识  考虑在到达终点时如何停下
        //printf("turn");
                    //***执行右转操作***//  以yaw角度为目标  ,实际测试，按照现有的安装方式，右转角度变小
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
            gray_allow = 1;
            gray_dir_allow =1;
            STATE2 --; //回到循迹状态
        }else{ //返回时还需要清除cflag——turn启用灰度
            stop_move(1500);
            END_FLAG = 1;
            STATE=5;
        }
        break;
    }

    case 5:{  //到达终点置位相关标志
        gray_allow = 0;
        gray_dir_allow = 0;
		lock_Loc();
		stop_move(1500);    
		//find_exit2 = 1;                        
		END_FLAG = 1;       //置为终点标志
		//STOP_FLAG = 0;
		buzzerTurnOnDelay(10);
		STATE2++;
    break;}

    default:
    break;
    }
    }