/*
 * File: SHA_SW.c
 * Embedded hardware selection: Infineon->TriCore
 */

#include "SHA_SW.h"
#include "SHA_SW_private.h"
#include "intrp1d_la.h"
#include "intrp3d_la.h"
#include "look1_If_linlca.h"
#include "look1_binlca.h"
#include "look1_iu16lftf_linlca.h"
#include "look1_tfId_linlca.h"
#include "look1_tf_binlca.h"
#include "look1_tu16DdIf_binlca.h"
#include "look1_tu16Dd_binlca.h"
#include "look1_yftfId_linlca.h"
#include "look2_binlca.h"
#include "plook_binca.h"
#include "plook_evenca.h"

/* 宏定义 */
#define SHA_IN_Close                   ((uint8_T)1U)
#define SHA_IN_Open                    ((uint8_T)2U)

/* 宏定义 */
#define SHA_IN_NO_ACTIVE_CHILD_pv      ((uint8_T)0U)
#define SHA_IN_OFF                     ((uint8_T)1U)
#define SHA_IN_ON                      ((uint8_T)2U)

/* 宏定义 */
#define SHA_IN_AlarmCancle             ((uint8_T)1U)
#define SHA_IN_AlarmHappend            ((uint8_T)2U)
#define SHA_IN_AlarmHold               ((uint8_T)1U)
#define SHA_IN_AlarmType               ((uint8_T)2U)
#define SHA_IN_Disable                 ((uint8_T)3U)
#define SHA_IN_NO_ACTIVE_CHILD_m       ((uint8_T)0U)

/* 宏定义 */
#define SHA_IN_AlarmCancle_n           ((uint8_T)1U)
#define SHA_IN_AlarmHappend_j          ((uint8_T)2U)
#define SHA_IN_AlarmHold_i             ((uint8_T)1U)
#define SHA_IN_AlarmType_p             ((uint8_T)2U)
#define SHA_IN_Disable_b               ((uint8_T)3U)
#define SHA_IN_NO_ACTIVE_CHILD_ev      ((uint8_T)0U)

/* 宏定义 */
#define SHA_IN_Open1                   ((uint8_T)1U)
#define SHA_IN_Open2                   ((uint8_T)2U)
#define SHA_IN_Open3                   ((uint8_T)3U)
#define SHA_IN_Open4                   ((uint8_T)4U)
#define SHA_IN_Open5                   ((uint8_T)5U)
#define SHA_IN_Open6                   ((uint8_T)6U)

/* 宏定义 */
#define SHA_IN_DECREASE                ((uint8_T)1U)
#define SHA_IN_INCREASE                ((uint8_T)2U)

/* 宏定义 */
#define SHA_IN_Close_k                 ((uint8_T)1U)
#define SHA_IN_Open_o                  ((uint8_T)2U)

/* 宏定义 */
#define SHA_IN_COCV                    ((uint8_T)1U)
#define SHA_IN_COLD_STARTUP            ((uint8_T)1U)
#define SHA_IN_CPRE                    ((uint8_T)2U)
#define SHA_IN_CREF                    ((uint8_T)3U)
#define SHA_IN_IDLE                    ((uint8_T)1U)
#define SHA_IN_INIT                    ((uint8_T)2U)
#define SHA_IN_NORMAL_STARTUP          ((uint8_T)2U)
#define SHA_IN_OCV                     ((uint8_T)1U)
#define SHA_IN_POWERON                 ((uint8_T)3U)
#define SHA_IN_PRE                     ((uint8_T)2U)
#define SHA_IN_REDUCED                 ((uint8_T)1U)
#define SHA_IN_REDUCED_DRY             ((uint8_T)2U)
#define SHA_IN_REDUCED_WET             ((uint8_T)3U)
#define SHA_IN_REF                     ((uint8_T)3U)
#define SHA_IN_RUNNING                 ((uint8_T)4U)
#define SHA_IN_RUN_NORMAL              ((uint8_T)1U)
#define SHA_IN_RUN_REDUCED             ((uint8_T)2U)
#define SHA_IN_SHUTDOWN                ((uint8_T)5U)
#define SHA_IN_SHUT_OCV                ((uint8_T)1U)
#define SHA_IN_SHUT_OFF                ((uint8_T)2U)
#define SHA_IN_SHUT_REF                ((uint8_T)3U)
#define SHA_IN_STARTUP                 ((uint8_T)6U)
#define SHA_IN_SYSFAULT                ((uint8_T)7U)
#define SHA_IN_ShutHoldoff             ((uint8_T)1U)
#define SHA_IN_ShutHoldon              ((uint8_T)2U)
#define SHA_event_ENTER_IDLE           (0)
#define SHA_event_ENTER_SHUTDOWN       (1)
#define SHA_event_OCV_FAULT            (2)
#define SHA_event_SHUT_FAIL            (3)
#define SHA_event_START_FAULT          (4)

#ifdef __GNUC__
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#define unused          __attribute__((unused))
#else
#define likely(x)       x
#define unlikely(x)     x
#define unused
#pragma warning(disable : 4996) /* For fscanf */
#endif

#ifndef genann_act
#define genann_act_hidden genann_act_hidden_indirect
#define genann_act_output genann_act_output_indirect
#else
#define genann_act_hidden genann_act
#define genann_act_output genann_act
#endif

#define LOOKUP_SIZE 4096

const real32_T sigmoid_dom_min = -15.0;
const real32_T sigmoid_dom_max = 15.0;
real32_T real8_Terval;
real32_T lookup[LOOKUP_SIZE];

/* 全局量定义 */
SHA_DW rtSHA_DW;

/* 函数声明 */
static void SHA_SystemProp_matlabCodegenSet(SHA_dsp_simulink_MovingAverage
    *SHA_obj, boolean_T SHA_value);
static void SHA_SystemCore_release(SHA_dsp_simulink_MovingAverage *SHA_obj);
static void SHA_SystemCore_delete(SHA_dsp_simulink_MovingAverage *SHA_obj);
static void SHA_matlabCodegenHandle_matlabC(SHA_dsp_simulink_MovingAverage
    *SHA_obj);

/* 函数声明 */
static void SHA_AlarmType(uint8_T rtSHA_u_Enable, real_T rtSHA_u_SignalValue,
    real_T rtSHA_u_CancelDiff, real_T rtSHA_u_CancelCount, real_T
    rtSHA_u_HappendCount, uint8_T rtSHA_u_JudegType, real_T rtSHA_u_LimitConfig1,
    real_T rtSHA_u_LimitConfig2, real_T rtSHA_u_LimitConfig3, real_T
    *rtSHA_y_AlarmLevel, SHA_DW_AlarmJudgeChart *localDW);
static void SHA_enter_internal_AlarmType(real_T rtSHA_u_SignalValue, uint8_T
    rtSHA_u_JudegType, real_T rtSHA_u_LimitConfig1, real_T rtSHA_u_LimitConfig2,
    real_T rtSHA_u_LimitConfig3, SHA_DW_AlarmJudgeChart *localDW);

/* 函数声明 */
static void SHA_AlarmType_l(uint8_T rtSHA_u_Enable, real_T rtSHA_u_SignalValue,
    real_T rtSHA_u_CancelDiff, real_T rtSHA_u_CancelCount, real_T
    rtSHA_u_HappendCount, uint8_T rtSHA_u_JudegType, real_T rtSHA_u_LimitConfig1,
    real_T rtSHA_u_LimitConfig2, real_T rtSHA_u_LimitConfig3, real32_T
    *rtSHA_y_AlarmLevel, SHA_DW_AlarmJudgeChart_m *localDW);
static void SHA_enter_internal_AlarmType_a(real_T rtSHA_u_SignalValue, uint8_T
    rtSHA_u_JudegType, real_T rtSHA_u_LimitConfig1, real_T rtSHA_u_LimitConfig2,
    real_T rtSHA_u_LimitConfig3, SHA_DW_AlarmJudgeChart_m *localDW);

/* 函数声明 */
static void SHA_IDLE(void);
static void SHA_INIT(void);
static void SHA_broadcast_START_FAULT(void);
static void SHA_inner_default_PRE(void);
static void SHA_broadcast_SHUT_FAIL(void);
static void SHA_broadcast_OCV_FAULT(void);
static void SHA_broadcast_ENTER_IDLE(void);
static void SHA_broadcast_ENTER_SHUTDOWN(void);
static void SHA_inner_default_COCV(void);
static void SHA_inner_default_CPRE(void);
static void SHA_inner_default_CREF(void);
static void SHA_inner_default_OCV(void);
static void SHA_inner_default_REF(void);
static void SHA_c13_SHA_SW(void);
static void SHA_inner_default_Para_i(void);
static void SHA_inner_default_Para(void);
static void SHA_GetMaxVolt(void);
static void SHA_GetMinVolt(void);
static void SHA_NorAndUnnVolt(void);

// 使用间接调用调用隐藏层激活函数
real32_T genann_act_hidden_indirect(const struct genann *ann, real32_T a) 
{
    return ann->activation_hidden(ann, a);
}

// 使用间接调用调用输出层激活函数
real32_T genann_act_output_indirect(const struct genann *ann, real32_T a) 
{
    return ann->activation_output(ann, a);
}

// 实现sigmoid激活函数
real32_T genann_act_sigmoid(const genann *ann unused, real32_T a) 
{
    // 限制输入值的范围，避免指数爆炸和梯度消失
    if (a < -45.0) return 0;
    if (a > 45.0) return 1;

    // 计算并返回sigmoid函数的结果
    return 1.0 / (1 + exp(-a));
}

// 初始化sigmoid激活函数的查找表
void genann_init_sigmoid_lookup(const genann *ann) 
{
    // 计算查找表的步长
    const real32_T f = (sigmoid_dom_max - sigmoid_dom_min) / LOOKUP_SIZE;
    real8_T i;
    
    // 计算步长间隔
    real8_Terval = LOOKUP_SIZE / (sigmoid_dom_max - sigmoid_dom_min);
    
    // 填充查找表，用于缓存sigmoid函数的计算结果
    for (i = 0; i < LOOKUP_SIZE; ++i) 
	{
        lookup[i] = genann_act_sigmoid(ann, sigmoid_dom_min + f * i);
    }
}

// 使用查找表来计算sigmoid激活函数的值
real32_T genann_act_sigmoid_cached(const genann *ann unused, real32_T a) 
{
    // 确保输入值不是NaN（非数值）
    assert(!isnan(a));

    // 如果输入值小于sigmoid函数定义域的最小值，返回查找表中的第一个值
    if (a < sigmoid_dom_min) return lookup[0];

    // 如果输入值大于等于sigmoid函数定义域的最大值，返回查找表中的最后一个值
    if (a >= sigmoid_dom_max) return lookup[LOOKUP_SIZE - 1];

    // 使用查找表来计算sigmoid函数的值并返回结果
    size_t j = (size_t)((a - sigmoid_dom_min) * real8_Terval + 0.5);

    // 确保查找表索引不越界，并返回结果
    if (unlikely(j >= LOOKUP_SIZE)) return lookup[LOOKUP_SIZE - 1];

    return lookup[j];
}

// 实现线性激活函数
real32_T genann_act_linear(const struct genann *ann unused, real32_T a) 
{
    // 直接返回输入值
    return a;
}

// 实现阈值激活函数
real32_T genann_act_threshold(const struct genann *ann unused, real32_T a) 
{
    // 如果输入值大于0，返回1；否则返回0
    return a > 0;
}

// 初始化一个前馈神经网络并返回指向它的指针
genann *genann_init(real8_T inputs, real8_T hidden_layers, real8_T hidden, real8_T outputs) 
{
    // 确保输入参数合法
    if (hidden_layers < 0 || inputs < 1 || outputs < 1 || (hidden_layers > 0 && hidden < 1))
        return 0;

    // 计算隐藏层权重数和输出层权重数
    const real8_T hidden_weights = hidden_layers ? (inputs+1) * hidden + (hidden_layers-1) * (hidden+1) * hidden : 0;
    const real8_T output_weights = (hidden_layers ? (hidden+1) : (inputs+1)) * outputs;

    // 计算总权重数和总神经元数
    const real8_T total_weights = (hidden_weights + output_weights);
    const real8_T total_neurons = (inputs + hidden * hidden_layers + outputs);

    // 计算分配额外内存所需的大小
    const real8_T size = sizeof(genann) + sizeof(real32_T) * (total_weights + total_neurons + (total_neurons - inputs));
    genann *ret = malloc(size);
    if (!ret) return 0;

    // 初始化神经网络结构参数
    ret->inputs = inputs;
    ret->hidden_layers = hidden_layers;
    ret->hidden = hidden;
    ret->outputs = outputs;

    ret->total_weights = total_weights;
    ret->total_neurons = total_neurons;

    // 设置指针
    ret->weight = (real32_T*)((char*)ret + sizeof(genann));
    ret->output = ret->weight + ret->total_weights;
    ret->delta = ret->output + ret->total_neurons;

    // 随机初始化权重
    genann_randomize(ret);

    // 设置激活函数为sigmoid
    ret->activation_hidden = genann_act_sigmoid_cached;
    ret->activation_output = genann_act_sigmoid_cached;

    // 初始化sigmoid激活函数的查找表
    genann_init_sigmoid_lookup(ret);

    // 返回初始化后的神经网络指针
    return ret;
}

// 从文件中读取神经网络的参数并创建一个对应的前馈神经网络
genann *genann_read(FILE *in) {
    real8_T inputs, hidden_layers, hidden, outputs;
    real8_T rc;

    // 从文件中读取输入参数
    errno = 0;
    rc = fscanf(in, "%d %d %d %d", &inputs, &hidden_layers, &hidden, &outputs);
    if (rc < 4 || errno != 0) 
	{
        perror("fscanf");
        return NULL;
    }

    // 使用读取的参数初始化神经网络
    genann *ann = genann_init(inputs, hidden_layers, hidden, outputs);

    // 读取神经网络的权重
    real8_T i;
    for (i = 0; i < ann->total_weights; ++i) 
	{
        errno = 0;
        rc = fscanf(in, " %le", ann->weight + i);
        if (rc < 1 || errno != 0) 
		{
            perror("fscanf");
            genann_free(ann);

            return NULL;
        }
    }

    // 返回初始化后的神经网络指针
    return ann;
}


// 复制一个神经网络并返回指向它的指针
genann *genann_copy(genann const *ann) 
{
    // 计算分配额外内存所需的大小
    const real8_T size = sizeof(genann) + sizeof(real32_T) * (ann->total_weights + ann->total_neurons + (ann->total_neurons - ann->inputs));
    genann *ret = malloc(size);
    if (!ret) return 0;

    // 将原神经网络的内容复制到新的神经网络中
    memcpy(ret, ann, size);

    // 设置指针
    ret->weight = (real32_T*)((char*)ret + sizeof(genann));
    ret->output = ret->weight + ret->total_weights;
    ret->delta = ret->output + ret->total_neurons;

    // 返回复制后的神经网络指针
    return ret;
}

// 随机初始化神经网络的权重
void genann_randomize(genann *ann) 
{
    real8_T i;
    for (i = 0; i < ann->total_weights; ++i) 
	{
        // 生成一个范围在[-0.5, 0.5]之间的随机数作为权重
        real32_T r = GENANN_RANDOM();
        ann->weight[i] = r - 0.5;
    }
}

// 释放神经网络的内存
void genann_free(genann *ann) 
{
    // 释放神经网络内存
    free(ann);
}

// 运行神经网络，使用给定的输入并返回输出
real32_T const *genann_run(genann const *ann, real32_T const *inputs) 
{
    // 指向权重、输出和输入的指针
    real32_T const *w = ann->weight;
    real32_T *o = ann->output + ann->inputs;
    real32_T const *i = ann->output;

    // 复制输入到神经网络的输入层
    memcpy(ann->output, inputs, sizeof(real32_T) * ann->inputs);

    real8_T h, j, k;

    if (!ann->hidden_layers) 
	{
        // 如果没有隐藏层，直接计算输出层的输出
        real32_T *ret = o;
        for (j = 0; j < ann->outputs; ++j) 
		{
            real32_T sum = *w++ * -1.0;
            for (k = 0; k < ann->inputs; ++k) 
			{
                sum += *w++ * i[k];
            }
            *o++ = genann_act_output(ann, sum);
        }

        return ret;
    }

    // 输入层到第一个隐藏层
    for (j = 0; j < ann->hidden; ++j) 
	{
        real32_T sum = *w++ * -1.0;
        for (k = 0; k < ann->inputs; ++k) 
		{
            sum += *w++ * i[k];
        }
        *o++ = genann_act_hidden(ann, sum);
    }

    i += ann->inputs;

    // 多个隐藏层
    for (h = 1; h < ann->hidden_layers; ++h) 
	{
        for (j = 0; j < ann->hidden; ++j) 
		{
            real32_T sum = *w++ * -1.0;
            for (k = 0; k < ann->hidden; ++k) 
			{
                sum += *w++ * i[k];
            }
            *o++ = genann_act_hidden(ann, sum);
        }

        i += ann->hidden;
    }

    // 最后一个隐藏层到输出层
    real32_T const *ret = o;
    for (j = 0; j < ann->outputs; ++j) 
	{
        real32_T sum = *w++ * -1.0;
        for (k = 0; k < ann->hidden; ++k) 
		{
            sum += *w++ * i[k];
        }
        *o++ = genann_act_output(ann, sum);
    }

    // 完整性检查使用所有权重并编写了所有输出
    assert(w - ann->weight == ann->total_weights);
    assert(o - ann->output == ann->total_neurons);

    return ret;
}



// 训练前馈神经网络
void genann_train(genann const *ann, real32_T const *inputs, real32_T const *desired_outputs, real32_T learning_rate) {
    /* 使网络向前运行，获取输出结果 */
    genann_run(ann, inputs);

    real8_T h, j, k;

    /* 首先设置输出层的增量 */
    {
        // 获取指向第一个输出、第一个增量和第一个期望输出的指针
        real32_T const *o = ann->output + ann->inputs + ann->hidden * ann->hidden_layers; /* 第一个输出 */
        real32_T *d = ann->delta + ann->hidden * ann->hidden_layers; /* 第一个增量 */
        real32_T const *t = desired_outputs; /* 第一个期望输出 */

        /* 设置输出层增量 */
        if (genann_act_output == genann_act_linear ||
                ann->activation_output == genann_act_linear) 
		{
            // 对于线性激活函数，计算输出层的增量
            for (j = 0; j < ann->outputs; ++j) 
			{
                *d++ = *t++ - *o++;
            }
        } 
		else 
		{
            // 对于非线性激活函数，计算输出层的增量
            for (j = 0; j < ann->outputs; ++j) 
			{
                *d++ = (*t - *o) * *o * (1.0 - *o);
                ++o; ++t;
            }
        }
    }

    /* 设置隐藏层delta，从最后一层开始，并向前计算 */
    for (h = ann->hidden_layers - 1; h >= 0; --h) {

        /* 找到这一层的第一个输出和增量 */
        real32_T const *o = ann->output + ann->inputs + (h * ann->hidden);
        real32_T *d = ann->delta + (h * ann->hidden);

        /* 找到下一层的第一个增量(可能是隐藏层或输出层) */
        real32_T const * const dd = ann->delta + ((h+1) * ann->hidden);

        /* 找到下一层的第一个权重(可能是隐藏层或输出层) */
        real32_T const * const ww = ann->weight + ((ann->inputs+1) * ann->hidden) + ((ann->hidden+1) * ann->hidden * (h));

        for (j = 0; j < ann->hidden; ++j) 
		{
            real32_T delta = 0;

            for (k = 0; k < (h == ann->hidden_layers-1 ? ann->outputs : ann->hidden); ++k) 
			{
                const real32_T forward_delta = dd[k];
                const real8_T windex = k * (ann->hidden + 1) + (j + 1);
                const real32_T forward_weight = ww[windex];
                delta += forward_delta * forward_weight;
            }
            *d = *o * (1.0-*o) * delta;
            ++d; ++o;
        }
    }

    /* 训练输出 */
    {
        /* 找到第一个输出增量 */
        real32_T const *d = ann->delta + ann->hidden * ann->hidden_layers; /* First output delta. */

        /* 找到第一个输出delta的第一个权值 */
        real32_T *w = ann->weight + (ann->hidden_layers
                ? ((ann->inputs+1) * ann->hidden + (ann->hidden+1) * ann->hidden * (ann->hidden_layers-1))
                : (0));

        /* 找到前一层的第一个输出. */
        real32_T const * const i = ann->output + (ann->hidden_layers
                ? (ann->inputs + (ann->hidden) * (ann->hidden_layers-1))
                : 0);
				
        /* 设置输出层权重 */
        for (j = 0; j < ann->outputs; ++j) 
		{
            *w++ += *d * learning_rate * -1.0;
            for (k = 1; k < (ann->hidden_layers ? ann->hidden : ann->inputs) + 1; ++k) 
			{
                *w++ += *d * learning_rate * i[k-1];
            }
            ++d;
        }
        assert(w - ann->weight == ann->total_weights);
    }

    /* 训练隐藏层 */
    for (h = ann->hidden_layers - 1; h >= 0; --h) 
	{
        /* 找到这一层的第一个 */
        real32_T const *d = ann->delta + (h * ann->hidden);

        /* 找到这个层的第一个输入 */
        real32_T const *i = ann->output + (h
                ? (ann->inputs + ann->hidden * (h-1))
                : 0);

        /* 找到这个图层的第一个权重 */
        real32_T *w = ann->weight + (h
                ? ((ann->inputs+1) * ann->hidden + (ann->hidden+1) * (ann->hidden) * (h-1))
                : 0);

        for (j = 0; j < ann->hidden; ++j) 
		{
            *w++ += *d * learning_rate * -1.0;
            for (k = 1; k < (h == 0 ? ann->inputs : ann->hidden) + 1; ++k) 
			{
                *w++ += *d * learning_rate * i[k-1];
            }
            ++d;
        }
    }
}

// 将神经网络的参数写入文件
void genann_write(genann const *ann, FILE *out) 
{
    // 将输入、隐藏层数、隐藏层大小和输出的数量写入文件
    fprreal8_Tf(out, "%d %d %d %d", ann->inputs, ann->hidden_layers, ann->hidden, ann->outputs);

    real8_T i;
    // 将神经网络的权重写入文件
    for (i = 0; i < ann->total_weights; ++i) 
	{
        fprreal8_Tf(out, " %.20e", ann->weight[i]);
    }
}

/*
 * 原子系统的初始化函数：
 * 此函数用于初始化SHA图表的控制类型。
 * 默认情况下，它将控制类型设置为0.0。
 */
void SHA_Chart_Init(real_T *rtSHA_y_CtrlType)
{
    *rtSHA_y_CtrlType = 0.0;
}

/*
 * 原子系统的复位函数：
 * 此函数用于复位SHA图表的控制类型。
 * 将控制类型设置为0.0，以清除任何先前的状态。
 */
void SHA_Chart_Reset(real_T *rtSHA_y_CtrlType)
{
    *rtSHA_y_CtrlType = 0.0;
}

/*
 * 原子系统的输出和更新函数：
 * 此函数是SHA图表的主要逻辑。
 * 它接收输入值rtSHA_u_In，并相应地设置控制类型rtSHA_y_CtrlType。
 * 如果rtSHA_u_In为1.0，则将控制类型设置为1.0。
 * 如果rtSHA_u_In为2.0，则将控制类型设置为-1.0。
 * 对于其他任何rtSHA_u_In的值，将控制类型设置为0.0。
 */
void SHA_Chart(real_T rtSHA_u_In, real_T *rtSHA_y_CtrlType)
{
    if (rtSHA_u_In == 1.0)
    {
        *rtSHA_y_CtrlType = 1.0;
    }
    else if (rtSHA_u_In == 2.0)
    {
        *rtSHA_y_CtrlType = -1.0;
    }
    else
    {
        *rtSHA_y_CtrlType = 0.0;
    }
}

/*
 * 原子系统的初始化函数：
 * 'OnOff' (':52046')系统的初始化函数。
 * 将输出变量 rtSHA_y_SetDutyOut 设置为0.0F。
 */
void SHA_OnOff_Init(real32_T *rtSHA_y_SetDutyOut)
{
    *rtSHA_y_SetDutyOut = 0.0F;
}

/*
 * 原子系统的输出和更新函数：
 * 'OnOff' (':52046')系统的输出和更新函数。
 * 这个函数用于控制OnOff开关逻辑。
 * 它接收输入 rtSHA_u_SetDutyIn，并根据当前状态和输入值更新输出变量 rtSHA_y_SetDutyOut。
 */
void SHA_OnOff(real_T rtSHA_u_SetDutyIn, real32_T *rtSHA_y_SetDutyOut,
               SHA_DW_OnOff *localDW)
{
    /* 图表：'OnOff' (':52046') */
    if (localDW->SHA_bitsForTID0.SHA_is_active_c8_SHA_SW == 0U)
    {
        localDW->SHA_bitsForTID0.SHA_is_active_c8_SHA_SW = 1;
        localDW->SHA_bitsForTID0.SHA_is_c8_SHA_SW = SHA_IN_Open;
        if (rtSHA_u_SetDutyIn < 0.07)
        {
            localDW->SHA_Cnt++;
            *rtSHA_y_SetDutyOut = 0.07F;
        }
        else
        {
            localDW->SHA_Cnt = 0.0;
            *rtSHA_y_SetDutyOut = (real32_T)rtSHA_u_SetDutyIn;
        }
    }
    else if (localDW->SHA_bitsForTID0.SHA_is_c8_SHA_SW == SHA_IN_Close)
    {
        if (rtSHA_u_SetDutyIn >= 0.07)
        {
            localDW->SHA_Cnt = 0.0;
            localDW->SHA_bitsForTID0.SHA_is_c8_SHA_SW = SHA_IN_Open;
            if (rtSHA_u_SetDutyIn < 0.07)
            {
                localDW->SHA_Cnt++;
                *rtSHA_y_SetDutyOut = 0.07F;
            }
            else
            {
                localDW->SHA_Cnt = 0.0;
                *rtSHA_y_SetDutyOut = (real32_T)rtSHA_u_SetDutyIn;
            }
        }
        else
        {
            *rtSHA_y_SetDutyOut = (real32_T)rtSHA_u_SetDutyIn;
        }
    }
    else if (localDW->SHA_Cnt >= 50.0)
    {
        localDW->SHA_bitsForTID0.SHA_is_c8_SHA_SW = SHA_IN_Close;
    }
    else if (rtSHA_u_SetDutyIn < 0.07)
    {
        localDW->SHA_Cnt++;
        *rtSHA_y_SetDutyOut = 0.07F;
    }
    else
    {
        localDW->SHA_Cnt = 0.0;
        *rtSHA_y_SetDutyOut = (real32_T)rtSHA_u_SetDutyIn;
    }

    /* 图表结束：'OnOff' (':52046') */
}

/*
 * 原子系统的初始化函数：
 * 此函数用于初始化SHA图表_n的输出变量。
 * 将输出变量 rtSHA_y_Output 设置为0.0。
 */
void SHA_Chart_n_Init(real_T *rtSHA_y_Output)
{
    *rtSHA_y_Output = 0.0;
}

/*
 * 原子系统的输出和更新函数：
 * 此函数是SHA图表_b的主要逻辑。
 * 它接收输入 rtSHA_u_Target、rtSHA_u_UpStep、rtSHA_u_DownStep、rtSHA_u_Uplimit 和 rtSHA_u_Downlimit，
 * 并根据这些输入值以及当前状态更新输出变量 rtSHA_y_Output。
 */
void SHA_Chart_b(real_T rtSHA_u_Target, real_T rtSHA_u_UpStep, real_T rtSHA_u_DownStep,
                 real_T rtSHA_u_Uplimit, real_T rtSHA_u_Downlimit, real_T *rtSHA_y_Output, SHA_DW_Chart_m *localDW)
{
    if (localDW->SHA_TempValue <= (rtSHA_u_Target - rtSHA_u_UpStep))
    {
        localDW->SHA_TempValue += rtSHA_u_UpStep;
    }
    else if (localDW->SHA_TempValue >= (rtSHA_u_Target + rtSHA_u_UpStep))
    {
        localDW->SHA_TempValue -= rtSHA_u_DownStep;
    }
    else
    {
        localDW->SHA_TempValue = rtSHA_u_Target;
    }

    if (!(localDW->SHA_TempValue > rtSHA_u_Downlimit))
    {
        localDW->SHA_TempValue = rtSHA_u_Downlimit;
    }

    if (!(localDW->SHA_TempValue < rtSHA_u_Uplimit))
    {
        localDW->SHA_TempValue = rtSHA_u_Uplimit;
    }

    *rtSHA_y_Output = localDW->SHA_TempValue;

}

/*
 * 原子系统的初始化函数：
 * 此函数用于初始化SHA图表_n的输出变量。
 * 将输出变量 rtSHA_y_Output 设置为0.0。
 */
void SHA_Chart_n_Init(real_T *rtSHA_y_Output)
{
    *rtSHA_y_Output = 0.0;
}

/*
 * 原子系统的输出和更新函数：
 * 此函数是SHA图表_b的主要逻辑。
 * 它接收输入 rtSHA_u_Target、rtSHA_u_UpStep、rtSHA_u_DownStep、rtSHA_u_Uplimit 和 rtSHA_u_Downlimit，
 * 并根据这些输入值以及当前状态更新输出变量 rtSHA_y_Output。
 */
void SHA_Chart_b(real_T rtSHA_u_Target, real_T rtSHA_u_UpStep, real_T rtSHA_u_DownStep,
                 real_T rtSHA_u_Uplimit, real_T rtSHA_u_Downlimit, real_T *rtSHA_y_Output, SHA_DW_Chart_m *localDW)
{
    if (localDW->SHA_TempValue <= (rtSHA_u_Target - rtSHA_u_UpStep))
    {
        localDW->SHA_TempValue += rtSHA_u_UpStep;
    }
    else if (localDW->SHA_TempValue >= (rtSHA_u_Target + rtSHA_u_UpStep))
    {
        localDW->SHA_TempValue -= rtSHA_u_DownStep;
    }
    else
    {
        localDW->SHA_TempValue = rtSHA_u_Target;
    }

    if (!(localDW->SHA_TempValue > rtSHA_u_Downlimit))
    {
        localDW->SHA_TempValue = rtSHA_u_Downlimit;
    }

    if (!(localDW->SHA_TempValue < rtSHA_u_Uplimit))
    {
        localDW->SHA_TempValue = rtSHA_u_Uplimit;
    }

    *rtSHA_y_Output = localDW->SHA_TempValue;

}

static void SHA_SystemCore_delete(SHA_dsp_simulink_MovingAverage *SHA_obj)
{
    /* MATLABSystem：“移动平均”(‘:55056’)的启动 */
    SHA_SystemCore_release(SHA_obj);
}

static void SHA_matlabCodegenHandle_matlabC(SHA_dsp_simulink_MovingAverage *SHA_obj)
{
    /* MATLABSystem：“移动平均”(‘:55056’)的启动 */
    if (!SHA_obj->matlabCodegenIsDeleted)
    {
        SHA_SystemProp_matlabCodegenSet(SHA_obj, TRUE);
        SHA_SystemCore_delete(SHA_obj);
    }

    /* 结束MATLABSystem：“移动平均”(‘:55056’)的启动 */
}

/*
 * 原子系统的初始化函数：
 * 此函数用于初始化SHA_MovingAverage图表的本地状态 localDW。
 * 启动MATLABSystem：“移动平均”(‘:55056’)。
 */
void SHA_MovingAverage_Init(SHA_DW_MovingAverage *localDW)
{
    /* 启动MATLABSystem：“移动平均”(‘:55056’) */
    if (localDW->SHA_obj.pStatistic->isInitialized == 1)
    {
        localDW->SHA_obj.pStatistic->pCumSum = 0.0;
        memset(&localDW->SHA_obj.pStatistic->pCumSumRev[0], 0, 19U * (sizeof(real_T)));
        localDW->SHA_obj.pStatistic->pCumRevIndex = 1.0;
    }

    /* 结束MATLABSystem：“移动平均”(‘:55056’)的启动 */
}

/*
 * 原子系统的启动函数：
 * 此函数用于启动SHA_MovingAverage图表的本地状态 localDW。
 * 启动MATLABSystem：“移动平均”(‘:55056’)。
 */
void SHA_MovingAverage_Start(SHA_DW_MovingAverage *localDW)
{
    /* 启动MATLABSystem：“移动平均”(‘:55056’) */
    localDW->SHA_obj.matlabCodegenIsDeleted = TRUE; // 标记MATLAB代码生成是否已删除
    localDW->SHA_obj.isInitialized = 0; // 标记MATLAB系统是否已初始化
    localDW->SHA_obj.NumChannels = -1; // 设置通道数为-1
    localDW->SHA_obj.matlabCodegenIsDeleted = FALSE; // 取消标记MATLAB代码生成是否已删除
    localDW->SHA_bitsForTID0.SHA_objisempty = TRUE; // 判断SHA对象是否为空
    localDW->SHA_obj.isSetupComplete = FALSE; // 标记MATLAB系统设置是否已完成
    localDW->SHA_obj.isInitialized = 1; // 标记MATLAB系统已初始化
    localDW->SHA_obj.NumChannels = 1; // 设置通道数为1
    localDW->SHA_gobj_0.isInitialized = 0; // 标记gobj对象是否已初始化
    localDW->SHA_obj.pStatistic = &localDW->SHA_gobj_0; // 设置pStatistic属性为gobj对象
    localDW->SHA_obj.isSetupComplete = TRUE; // 标记MATLAB系统设置已完成
    localDW->SHA_obj.TunablePropsChanged = FALSE; // 标记MATLAB系统的可调属性是否已更改
}


/*
 * 原子系统的输出和更新函数：
 * 此函数是SHA_MovingAverage图表的主要逻辑。
 * 它接收输入 rtSHA_u_0 和本地状态 localDW，
 * 并根据这些输入值以及当前状态更新本地状态 localDW 中的属性。
 */
void SHA_MovingAverage(real_T rtSHA_u_0, SHA_DW_MovingAverage *localDW)
{    real_T SHA_cumRevIndex; // 累计逆向索引
    real_T SHA_csum; // 累计和
    real_T SHA_csumrev[19]; // 累计和数组
    real_T SHA_z; // 中间变量
    int32_T SHA_z_tmp; // 中间变量

    /* Start for MATLABSystem: &apos;Moving Average&apos; (&apos;:55056&apos;) */
    if (localDW->SHA_obj.TunablePropsChanged)
    {
        localDW->SHA_obj.TunablePropsChanged = FALSE;
    }

    // 检查是否已初始化统计数据
    if (localDW->SHA_obj.pStatistic->isInitialized != 1)
    {
        localDW->SHA_obj.pStatistic->isSetupComplete = FALSE;
        localDW->SHA_obj.pStatistic->isInitialized = 1;
        localDW->SHA_obj.pStatistic->pCumSum = 0.0;
        localDW->SHA_obj.pStatistic->pCumRevIndex = 1.0;
        localDW->SHA_obj.pStatistic->isSetupComplete = TRUE;
        localDW->SHA_obj.pStatistic->pCumSum = 0.0;
        memset(&localDW->SHA_obj.pStatistic->pCumSumRev[0], 0, 19U * (sizeof(real_T)));
        localDW->SHA_obj.pStatistic->pCumRevIndex = 1.0;
    }

    // 获取统计数据
    SHA_cumRevIndex = localDW->SHA_obj.pStatistic->pCumRevIndex;
    SHA_csum = localDW->SHA_obj.pStatistic->pCumSum;
    for (SHA_z_tmp = 0; SHA_z_tmp < 19; SHA_z_tmp++)
    {
        SHA_csumrev[SHA_z_tmp] = localDW->SHA_obj.pStatistic->pCumSumRev[SHA_z_tmp];
    }

    // 计算累计和及累计和数组
    SHA_csum += rtSHA_u_0;
    SHA_z_tmp = ((int32_T)SHA_cumRevIndex) - 1;
    SHA_z = SHA_csumrev[SHA_z_tmp] + SHA_csum;
    SHA_csumrev[SHA_z_tmp] = rtSHA_u_0;

    // 更新累计逆向索引
    if (SHA_cumRevIndex != 19.0)
    {
        SHA_cumRevIndex++;
    }
    else
    {
        SHA_cumRevIndex = 1.0;
        SHA_csum = 0.0;
        for (SHA_z_tmp = 17; SHA_z_tmp >= 0; SHA_z_tmp--)
        {
            SHA_csumrev[SHA_z_tmp] += SHA_csumrev[SHA_z_tmp + 1];
        }
    }

    // 更新统计数据
    localDW->SHA_obj.pStatistic->pCumSum = SHA_csum;
    memcpy(&localDW->SHA_obj.pStatistic->pCumSumRev[0], &SHA_csumrev[0], 19U * (sizeof(real_T)));
    localDW->SHA_obj.pStatistic->pCumRevIndex = SHA_cumRevIndex;

    /* MATLABSystem: &apos;Moving Average&apos; (&apos;:55056&apos;) */
    localDW->SHA_MovingAverage_d = SHA_z; // 将计算结果保存到本地状态 localDW 中的属性 SHA_MovingAverage_d
}

/*
 * 原子系统的初始化函数：
 *    此函数用于初始化 &apos;StateControl5&apos; 图表。
 *    通过调用子系统 &apos;Subsystem2&apos; 的初始化函数 &apos;SHA_MovingAverage_Init&apos;，对子系统进行初始化。
 */
void SHA_StateControl5_Init(SHA_DW_StateControl5 *localDW)
{    /* 子系统 &apos;Subsystem2&apos; (&apos;:55053&apos;) 的初始化 */
    SHA_MovingAverage_Init(&localDW->SHA_MovingAverage_p);
}
/*
 * 原子系统的启动函数：
 *    此函数用于启动 &apos;StateControl5&apos; 图表。
 *    通过调用子系统 &apos;Subsystem2&apos; 的启动函数 &apos;SHA_MovingAverage_Start&apos;，对子系统进行启动。
 */
void SHA_StateControl5_Start(SHA_DW_StateControl5 *localDW)
{    /* 子系统 &apos;Subsystem2&apos; (&apos;:55053&apos;) 的启动 */
    SHA_MovingAverage_Start(&localDW->SHA_MovingAverage_p);
}
/*
 * 原子系统的输出和更新函数：
 *    此函数包含 &apos;StateControl5&apos; 图表的主要逻辑。
 *    它接收输入 rtSHA_u_Timer 和 rtSHA_u_1，以及本地状态 localDW，
 *    并根据这些输入值以及当前状态更新图表的输出和本地状态。
 */
void SHA_StateControl5(uint32_T rtSHA_u_Timer, real_T rtSHA_u_1,
                       SHA_DW_StateControl5 *localDW)
{    /* Chart: &apos;StateControl5&apos; (&apos;:55058:97&apos;) */
    if (localDW->SHA_temporalCounter_i1 < rtSHA_u_Timer)
    {
        localDW->SHA_temporalCounter_i1++;
    }

    if (localDW->SHA_bitsForTID0.SHA_is_active_c55_SHA_SW_LIB == 0U)
    {
        localDW->SHA_bitsForTID0.SHA_is_active_c55_SHA_SW_LIB = 1;
        localDW->SHA_temporalCounter_i1 = 0U;
    }
    else
    {
        if (localDW->SHA_temporalCounter_i1 == rtSHA_u_Timer)
        {
            /* 子系统 &apos;Subsystem2&apos; (&apos;:55053&apos;) 的输出和更新 */
            SHA_MovingAverage(rtSHA_u_1, &localDW->SHA_MovingAverage_p);
        }
    }

    if (localDW->SHA_temporalCounter_i1 == rtSHA_u_Timer)
    {
        localDW->SHA_temporalCounter_i1 = 0U;
    }

    /* End of Chart: &apos;StateControl5&apos; (&apos;:55058:97&apos;) */
}
/*
 * 原子系统的输出和更新函数：
 *    此函数包含 &apos;StateControl5&apos; 图表的主要逻辑。
 *    它接收输入 rtSHA_u_Timer 和本地状态 localDW，
 *    并根据这些输入值以及当前状态更新图表的输出和本地状态。
 */
void SHA_StateControl5_j(uint32_T rtSHA_u_Timer, SHA_DW_StateControl5_a *localDW)
{    /* Chart: &apos;StateControl5&apos; (&apos;:49635:97&apos;) */
    if (localDW->SHA_temporalCounter_i1 < rtSHA_u_Timer)
    {
        localDW->SHA_temporalCounter_i1++;
    }

    if (localDW->SHA_bitsForTID0.SHA_is_active_c55_SHA_SW_LIB == 0U)
    {
        localDW->SHA_bitsForTID0.SHA_is_active_c55_SHA_SW_LIB = 1;
        localDW->SHA_temporalCounter_i1 = 0U;
    }

    if (localDW->SHA_temporalCounter_i1 == rtSHA_u_Timer)
    {
        localDW->SHA_temporalCounter_i1 = 0U;
    }

    /* End of Chart: &apos;StateControl5&apos; (&apos;:49635:97&apos;) */
}
/*
 * 输出和更新函数：&apos;AirFlowAdc&apos; (&apos;:52021&apos;)
 *    此函数包含 &apos;AirFlowAdc&apos; 模块的主要逻辑。
 *    它接收输入 rtSHA_u_u，并根据此输入值计算输出 rtSHA_y_y。
 */
void SHA_AirFlowAdc(uint16_T rtSHA_u_u, real_T *rtSHA_y_y)
{    uint32_T SHA_bpIdx;
    real_T SHA_frac;

    /* 1-D 查找表 &apos;1-D Lookup Table3&apos; (&apos;:52593&apos;) 的计算：
     *    使用常量 &apos;Constant1&apos; (&apos;:52018&apos;)、增益 &apos;Gain2&apos; (&apos;:52016&apos;) 和除法 &apos;Divide&apos; (&apos;:52017&apos;) 对输入 rtSHA_u_u 进行预处理，
     *    然后在查找表 &apos;1-D Lookup Table3&apos; (&apos;:52593&apos;) 中查找对应的输出。
     *    该输出将用于信号转换 &apos;TmpSignal ConversionAtuOutport1&apos; (&apos;:0&apos;)。
     */
    SHA_bpIdx = plook_evenca(((40960.0 * ((real_T)rtSHA_u_u)) * 0.0001220703125) / 4095.0, 0.0, 5.0, 1U, &SHA_frac);

    /* 信号转换 &apos;TmpSignal ConversionAtyInport1&apos; (&apos;:0&apos;) 的计算：
     *    使用查找表 &apos;1-D Lookup Table3&apos; (&apos;:52593&apos;) 和查找表 &apos;1-D Lookup Table7&apos; (&apos;:52019&apos;) 的输出，查找对应的输出。
     */
    *rtSHA_y_y = look1_binlca(intrp1d_la(SHA_bpIdx, SHA_frac, rtCP_SHA_uDLookupTable3_tableDa, 1U), rtCP_SHA_uDLookupTable7_bp01D_e, rtCP_SHA_uDLookupTable7_table_b, 7U);
}
/*
 * 输出和更新函数：&apos;H2FlowAdc&apos; (&apos;:52090&apos;)
 *    此函数包含 &apos;H2FlowAdc&apos; 模块的主要逻辑。
 *    它接收输入 rtSHA_u_u，并根据此输入值计算输出 rtSHA_y_y。
 */
void SHA_H2FlowAdc(uint16_T rtSHA_u_u, real_T *rtSHA_y_y)
{    /* 信号转换 &apos;TmpSignal ConversionAtyInport1&apos; (&apos;:0&apos;) 的计算：
     *    使用常量 &apos;Constant1&apos; (&apos;:52094&apos;)、增益 &apos;Gain2&apos; (&apos;:52096&apos;) 和查找表 &apos;1-D Lookup Table6&apos; (&apos;:52092&apos;) 的输出，
     *    查找对应的输出。
     */
    *rtSHA_y_y = look1_binlca(((40960.0 * ((real_T)rtSHA_u_u)) * 0.0001220703125) / 4095.0, rtCP_SHA_uDLookupTable6_bp01D_d, rtCP_SHA_uDLookupTable6_table_o, 1U);
}
/*
 * 输出和更新函数：&apos;LowPresAdc&apos; (&apos;:49007&apos;)
 *    此函数包含 &apos;LowPresAdc&apos; 模块的主要逻辑。
 *    它接收输入 rtSHA_u_u，并根据此输入值计算输出 rtSHA_y_y。
 */
void SHA_LowPresAdc(uint16_T rtSHA_u_u, real_T *rtSHA_y_y)
{    /* 开关 &apos;Switch&apos; (&apos;:49013&apos;) 的计算：
     *    使用常量 &apos;Constant&apos; (&apos;:49009:3&apos;) 和 &apos;Constant&apos; (&apos;:49010:3&apos;)、逻辑 &apos;Logical Operator&apos; (&apos;:49012&apos;)、
     *    比较 &apos;Compare&apos; (&apos;:49009:2&apos;) 和 &apos;Compare&apos; (&apos;:49010:2&apos;) 的结果，根据条件进行输出。
     */
    if ((rtSHA_u_u < 410) || (rtSHA_u_u > 3686))
    {
        *rtSHA_y_y = 0.0;
    }
    else
    {
        /* 信号转换 &apos;TmpSignal ConversionAtyInport1&apos; (&apos;:0&apos;) 的计算：
         *    使用查找表 &apos;Table_Ntc_3&apos; (&apos;:49014&apos;) 的输出，查找对应的输出。
         */
        *rtSHA_y_y = look1_iu16lftf_linlca(rtSHA_u_u, rtCP_SHA_Table_Ntc_3_bp01Data_a, rtCP_SHA_Table_Ntc_3_tableData, 1U);
    }

    /* 结束开关 &apos;Switch&apos; (&apos;:49013&apos;) 的计算。 */
}
/*
 * 输出和更新函数：&apos;Pt1000&apos; (&apos;:48989&apos;)
 *    此函数包含 &apos;Pt1000&apos; 模块的主要逻辑。
 *    它接收输入 rtSHA_u_u，并根据此输入值计算输出 rtSHA_y_y。
 */
void SHA_Pt1000(uint16_T rtSHA_u_u, real_T *rtSHA_y_y)
{    uint16_T SHA_rtSHA_u_u;

    /* 饱和 &apos;Saturation&apos; (&apos;:49585&apos;) 的计算：
     *    使用常量 &apos;Constant2&apos; (&apos;:49577&apos;) 对输入 rtSHA_u_u 进行饱和处理，
     *    然后进行增益 &apos;Gain1&apos; (&apos;:49578&apos;) 和除法 &apos;Divide&apos; (&apos;:49583&apos;) 的计算，
     *    最后进行减法和乘法的计算，输出结果 rtSHA_y_y。
     */
    if (rtSHA_u_u < 4095)
    {
        SHA_rtSHA_u_u = rtSHA_u_u;
    }
    else
    {
        SHA_rtSHA_u_u = 4095U;
    }

    *rtSHA_y_y = ((((real32_T)(rtSHA_u_u * 2000U)) / (4095.0F - ((real32_T)SHA_rtSHA_u_u))) - 1000.0) * 0.25974;
}
/*
 * 原子系统的初始化函数：
 *    此函数用于初始化 &apos;Chart1&apos; 图表的输出变量。
 *   
 将输出变量 rtSHA_y_OutData 设置为 0.0。
 */
void SHA_Chart1_Init(real_T *rtSHA_y_OutData)
{    *rtSHA_y_OutData = 0.0;
}
/*
 * 输出和更新函数：
 *    此函数包含 &apos;Chart1&apos; 图表的主要逻辑。
 *    它接收输入 rtSHA_u_InData、rtSHA_u_MaxDepth 和本地状态 localDW，
 *    并根据这些输入值以及当前状态更新图表的输出变量 rtSHA_y_OutData 和本地状态 localDW。
 */
void SHA_Chart1(real32_T rtSHA_u_InData, uint8_T rtSHA_u_MaxDepth, real_T *rtSHA_y_OutData, SHA_DW_Chart1 *localDW)
{    real_T SHA_DataSum;
    uint8_T SHA_Loop1;
    int32_T SHA_SHA_AverageFilterBuff_tmp;

    /* 更新滑动平均滤波器的状态变量 localDW->SHA_Depth 和 localDW->SHA_AverageFilterBuff。 */
    if ((localDW->SHA_Depth < rtSHA_u_MaxDepth) && (localDW->SHA_Depth < ((sizeof(real_T [30])) / (sizeof(real_T)))))
    {
        SHA_SHA_AverageFilterBuff_tmp = localDW->SHA_Depth + 1;
        if (SHA_SHA_AverageFilterBuff_tmp > 255)
        {
            SHA_SHA_AverageFilterBuff_tmp = 255;
        }

        localDW->SHA_Depth = (uint8_T)SHA_SHA_AverageFilterBuff_tmp;
    }

    SHA_SHA_AverageFilterBuff_tmp = localDW->SHA_Depth - 1;
    if (SHA_SHA_AverageFilterBuff_tmp < 0)
    {
        SHA_SHA_AverageFilterBuff_tmp = 0;
    }

    SHA_Loop1 = (uint8_T)SHA_SHA_AverageFilterBuff_tmp;
    while (SHA_Loop1 > 0)
    {
        SHA_SHA_AverageFilterBuff_tmp = SHA_Loop1 - 1;
        localDW->SHA_AverageFilterBuff[SHA_Loop1] = localDW->SHA_AverageFilterBuff[SHA_SHA_AverageFilterBuff_tmp];
        SHA_Loop1 = (uint8_T)SHA_SHA_AverageFilterBuff_tmp;
    }

    /* 计算滑动平均值。 */
    SHA_Loop1 = 0U;
    SHA_DataSum = 0.0;
    localDW->SHA_AverageFilterBuff[0] = rtSHA_u_InData;
    while (SHA_Loop1 < localDW->SHA_Depth)
    {
        SHA_DataSum += localDW->SHA_AverageFilterBuff[SHA_Loop1];
        SHA_Loop1++;
    }

    if (localDW->SHA_Depth != 0)
    {
        *rtSHA_y_OutData = SHA_DataSum / ((real_T)localDW->SHA_Depth);
    }
}
/*
 * 输出和更新函数：&apos;Simulink Function5&apos; (&apos;:43827&apos;)
 *    此函数包含 &apos;Simulink Function5&apos; 模块的主要逻辑。
 *    它接收输入 rtSHA_u_u，并根据此输入值计算输出 rtSHA_y_y。
 */
void SHA_Pt1000_c(uint16_T rtSHA_u_u, real_T *rtSHA_y_y)
{    uint16_T SHA_rtSHA_u_u;

    /* 饱和 &apos;Saturation&apos; (&apos;:43835&apos;) 的计算：
     *    使用常量 &apos;Constant2&apos; (&apos;:49574&apos;) 对输入 rtSHA_u_u 进行饱和处理，
     *    然后进行增益 &apos;Gain1&apos; (&apos;:49576&apos;) 和除法 &apos;Divide&apos; (&apos;:43833&apos;) 的计算，
     *    最后进行减法和乘法的计算，输出结果 rtSHA_y_y。
     */
    if (rtSHA_u_u < 4095)
    {
        SHA_rtSHA_u_u = rtSHA_u_u;
    }
    else
    {
        SHA_rtSHA_u_u = 4095U;
    }

    *rtSHA_y_y = ((((real32_T)(rtSHA_u_u * 2000U)) / (4095.0F - ((real32_T)SHA_rtSHA_u_u))) - 1000.0) * 0.25974;
}
/*
 * 输出和更新函数：&apos;Simulink Function7&apos; (&apos;:43979&apos;)
 *    此函数包含 &apos;Simulink Function7&apos; 模块的主要逻辑。
 *    它接收输入 rtSHA_u_u，并根据此输入值计算输出 rtSHA_y_y。
 */
void SHA_LowPresAdc_n(uint16_T rtSHA_u_u, real_T *rtSHA_y_y)
{    /* 开关 &apos;Switch&apos; (&apos;:43985&apos;) 的计算：
     *    使用常量 &apos;Constant&apos; (&apos;:43981:3&apos;) 和 &apos;Constant&apos; (&apos;:43982:3&apos;)、逻辑 &apos;Logical Operator&apos; (&apos;:43984&apos;)、
     *    比较 &apos;Compare&apos; (&apos;:43981:2&apos;) 和 &apos;Compare&apos; (&apos;:43982:2&apos;) 的结果，根据条件进行输出。
     */
    if ((rtSHA_u_u < 410) || (rtSHA_u_u > 3686))
    {
        /* 信号转换 &apos;TmpSignal ConversionAtyInport1&apos; (&apos;:0&apos;) 的计算：
         *    使用常量 &apos;Constant&apos; (&apos;:43983&apos;)，输出结果为 0.0。
         */
        *rtSHA_y_y = 0.0;
    }
    else
    {
        /* 信号转换 &apos;TmpSignal ConversionAtyInport1&apos; (&apos;:0&apos;) 的计算：
         *    使用查找表 &apos;Table_Ntc_3&apos; (&apos;:43986&apos;) 的输出，查找对应的输出。
         */
        *rtSHA_y_y = look1_iu16lftf_linlca(rtSHA_u_u, rtCP_SHA_Table_Ntc_3_bp01Data_p, rtCP_SHA_Table_Ntc_3_tableData_, 1U);
    }

    /* 结束开关 &apos;Switch&apos; (&apos;:43985&apos;) 的计算。 */
}
/*
 * 原子系统的初始化函数：
 *    此函数用于初始化 &apos;StateControl5&apos; 图表的输出标志变量。
 *    将输出标志变量 rtSHA_y_OutFlag 设置为 FALSE。
 */
void SHA_StateControl5_c_Init(boolean_T *rtSHA_y_OutFlag)
{    *rtSHA_y_OutFlag = FALSE;
}
/*
 * 原子系统的复位函数：
 *    此函数用于复位 &apos;StateControl5&apos; 图表的状态变量和输出标志变量。
 *    将输出标志变量 rtSHA_y_OutFlag 设置为 FALSE，
 *    并将状态变量 localDW 的其他字段进行复位。
 */
void SHA_StateControl5_a_Reset(boolean_T *rtSHA_y_OutFlag, SHA_DW_StateControl5_o *localDW)
{    localDW->SHA_bitsForTID0.SHA_is_active_c2_SHA_SW_LIB = 0;
    localDW->SHA_bitsForTID0.SHA_is_c2_SHA_SW_LIB = SHA_IN_NO_ACTIVE_CHILD_pv;
    localDW->SHA_Timer = 0.0;
    *rtSHA_y_OutFlag = FALSE;
}

/* 故障判断状态机的内部状态转换函数 */
static void SHA_enter_internal_AlarmType(real_T rtSHA_u_SignalValue, uint8_T rtSHA_u_JudegType,
                                        real_T rtSHA_u_LimitConfig1, real_T rtSHA_u_LimitConfig2,
                                        real_T rtSHA_u_LimitConfig3, SHA_DW_AlarmJudgeChart *localDW)
{    int32_T SHA_tmp;

    // 初始化状态为“报警取消”
    localDW->SHA_bitsForTID0.SHA_is_JudgeLevel_1 = SHA_IN_AlarmCancle;

    // 根据不同的判断类型进行故障判断
    switch (rtSHA_u_JudegType)
    {
        case 0:
            // 判断信号值是否小于等于阈值1
            if (rtSHA_u_SignalValue <= rtSHA_u_LimitConfig1)
            {
                // 如果满足条件，增加时间计数器1并进行上限限制
                SHA_tmp = localDW->SHA_TimeCount1 + 1;
                if (SHA_tmp > 65535)
                {
                    SHA_tmp = 65535;
                }
                localDW->SHA_TimeCount1 = (uint16_T)SHA_tmp;
            }
            else
            {
                // 如果不满足条件，将时间计数器1重置为0
                localDW->SHA_TimeCount1 = 0U;
            }
            break;

        case 1:
            // 判断信号值是否大于等于阈值1
            if (rtSHA_u_SignalValue >= rtSHA_u_LimitConfig1)
            {
                // 如果满足条件，增加时间计数器1并进行上限限制
                SHA_tmp = localDW->SHA_TimeCount1 + 1;
                if (SHA_tmp > 65535)
                {
                    SHA_tmp = 65535;
                }
                localDW->SHA_TimeCount1 = (uint16_T)SHA_tmp;
            }
            else
            {
                // 如果不满足条件，将时间计数器1重置为0
                localDW->SHA_TimeCount1 = 0U;
            }
            break;
    }

    // 初始化状态为“报警取消”
    localDW->SHA_bitsForTID0.SHA_is_JudgeLevel_2 = SHA_IN_AlarmCancle;

    // 根据不同的判断类型进行故障判断
    switch (rtSHA_u_JudegType)
    {
        case 0:
            // 判断信号值是否小于等于阈值2
            if (rtSHA_u_SignalValue <= rtSHA_u_LimitConfig2)
            {
                // 如果满足条件，增加时间计数器2并进行上限限制
                SHA_tmp = localDW->SHA_TimeCount2 + 1;
                if (SHA_tmp > 65535)
                {
                    SHA_tmp = 65535;
                }
                localDW->SHA_TimeCount2 = (uint16_T)SHA_tmp;
            }
            else
            {
                // 如果不满足条件，将时间计数器2重置为0
                localDW->SHA_TimeCount2 = 0U;
            }
            break;

        case 1:
            // 判断信号值是否大于等于阈值2
            if (rtSHA_u_SignalValue >= rtSHA_u_LimitConfig2)
            {
                // 如果满足条件，增加时间计数器2并进行上限限制
                SHA_tmp = localDW->SHA_TimeCount2 + 1;
                if (SHA_tmp > 65535)
                {
                    SHA_tmp = 65535;
                }
                localDW->SHA_TimeCount2 = (uint16_T)SHA_tmp;
            }
            else
            {
                // 如果不满足条件，将时间计数器2重置为0
                localDW->SHA_TimeCount2 = 0U;
            }
            break;
    }

    // 初始化状态为“报警取消”
    localDW->SHA_bitsForTID0.SHA_is_JudgeLevel_3 = SHA_IN_AlarmCancle;

    // 根据不同的判断类型进行故障判断
    switch (rtSHA_u_JudegType)
    {
        case 0:
            // 判断信号值是否小于等于阈值3
            if (rtSHA_u_SignalValue <= rtSHA_u_LimitConfig3)
            {
                // 如果满足条件，增加时间计数器3并进行上限限制
                SHA_tmp = localDW->SHA_TimeCount3 + 1;
                if (SHA_tmp > 65535)
                {
                    SHA_tmp = 65535;
                }
                localDW->SHA_TimeCount3 = (uint16_T)SHA_tmp;
            }
            else
            {
                // 如果不满足条件，将时间计数器3重置为0
                localDW->SHA_TimeCount3 = 0U;
            }
            break;

        case 1:
            // 判断信号值是否大于等于阈值3
            if (rtSHA_u_SignalValue >= rtSHA_u_LimitConfig3)
            {
                // 如果满足条件，增加时间计数器3并进行上限限制
                SHA_tmp = localDW->SHA_TimeCount3 + 1;
                if (SHA_tmp > 65535)
                {
                    SHA_tmp = 65535;
                }
                localDW->SHA_TimeCount3 = (uint16_T)SHA_tmp;
            }
            else
            {
                // 如果不满足条件，将时间计数器3重置为0
                localDW->SHA_TimeCount3 = 0U;
            }
            break;
    }
}

/*
 * 固态电池健康度监测的故障判断状态机的输出和更新函数
 */
void SHA_AlarmJudgeChart(uint8_T rtSHA_u_Enable, real_T rtSHA_u_SignalValue,
    real_T rtSHA_u_CancelDiff, real_T rtSHA_u_CancelCount, real_T
    rtSHA_u_HappendCount, uint8_T rtSHA_u_JudegType, real_T rtSHA_u_LimitConfig1,
    real_T rtSHA_u_LimitConfig2, real_T rtSHA_u_LimitConfig3, real_T
    *rtSHA_y_AlarmLevel, SHA_DW_AlarmJudgeChart *localDW)
{    if (localDW->SHA_bitsForTID0.SHA_is_active_c7_sqdCRW88tuDwsS == 0U)
    {
        // 若状态机首次激活，进行初始化操作
        localDW->SHA_bitsForTID0.SHA_is_active_c7_sqdCRW88tuDwsS = 1;
        localDW->SHA_bitsForTID0.SHA_is_c7_sqdCRW88tuDwsS6wS1mlw =
            SHA_IN_Disable;
        localDW->SHA_AlarmFlag1 = 0U;
        localDW->SHA_AlarmFlag2 = 0U;
        localDW->SHA_AlarmFlag3 = 0U;
        localDW->SHA_TimeCount1 = 0U;
        localDW->SHA_TimeCount2 = 0U;
        localDW->SHA_TimeCount3 = 0U;
        *rtSHA_y_AlarmLevel = 0.0;
    }
    else
    {
        // 根据当前状态进行状态转换和故障判断
        switch (localDW->SHA_bitsForTID0.SHA_is_c7_sqdCRW88tuDwsS6wS1mlw)
        {
            case SHA_IN_AlarmHold:
                // 当前状态为“报警保持”，根据输入的使能信号进行状态转换
                switch (rtSHA_u_Enable)
                {
                    case 1:
                        // 如果使能信号为1，则状态转换到“故障判断”
                        localDW->SHA_bitsForTID0.SHA_is_c7_sqdCRW88tuDwsS6wS1mlw =
                            SHA_IN_AlarmType;
                        SHA_enter_internal_AlarmType(rtSHA_u_SignalValue,
                            rtSHA_u_JudegType, rtSHA_u_LimitConfig1,
                            rtSHA_u_LimitConfig2, rtSHA_u_LimitConfig3, localDW);
                        break;

                    case 0:
                        // 如果使能信号为0，则状态转换到“禁用”，并将报警标志和时间计数器重置
                        localDW->SHA_bitsForTID0.SHA_is_c7_sqdCRW88tuDwsS6wS1mlw =
                            SHA_IN_Disable;
                        localDW->SHA_AlarmFlag1 = 0U;
                        localDW->SHA_AlarmFlag2 = 0U;
                        localDW->SHA_AlarmFlag3 = 0U;
                        localDW->SHA_TimeCount1 = 0U;
                        localDW->SHA_TimeCount2 = 0U;
                        localDW->SHA_TimeCount3 = 0U;
                        *rtSHA_y_AlarmLevel = 0.0;
                        break;
                }
                break;

            case SHA_IN_AlarmType:
                // 当前状态为“故障判断”，调用故障判断状态机函数进行故障判断
                SHA_AlarmType(rtSHA_u_Enable, rtSHA_u_SignalValue,
                              rtSHA_u_CancelDiff, rtSHA_u_CancelCount,
                              rtSHA_u_HappendCount, rtSHA_u_JudegType,
                              rtSHA_u_LimitConfig1, rtSHA_u_LimitConfig2,
                              rtSHA_u_LimitConfig3, rtSHA_y_AlarmLevel, localDW);
                break;

            default:
                // 其他情况下，根据输入的使能信号进行状态转换
                if (rtSHA_u_Enable == 1)
                {
                    // 如果使能信号为1，则状态转换到“故障判断”
                    localDW->SHA_bitsForTID0.SHA_is_c7_sqdCRW88tuDwsS6wS1mlw =
                        SHA_IN_AlarmType;
                    SHA_enter_internal_AlarmType(rtSHA_u_SignalValue,
                        rtSHA_u_JudegType, rtSHA_u_LimitConfig1,
                        rtSHA_u_LimitConfig2, rtSHA_u_LimitConfig3, localDW);
                }
                break;
        }
    }
}

/*
 * 状态机中的空闲状态处理函数
 */
static void SHA_IDLE(void)
{    // 将系统状态设置为IDLE（空闲状态）
    rtSHA_DW.SHA_SysState_e = IDLE;
    rtSHA_DW.SHA_CtrState_i = CTR_NULL;

    // 判断故障检测标志并执行相应的状态转换
    if (rtSHA_DW.SHA_bitsForTID0.SHA_LogicalOperator_l)
    {
        // 如果故障检测标志为真，则执行系统故障处理
        rtSHA_DW.SHA_FaultCheckFlag = 6U;
        rtSHA_DW.SHA_bitsForTID0.SHA_is_IDLE = 0;
        rtSHA_DW.SHA_bitsForTID0.SHA_is_c13_SHA_SW = SHA_IN_SYSFAULT;
        rtSHA_DW.SHA_SysState_e = SYSFAULT;
        rtSHA_DW.SHA_CtrState_i = CTR_NULL;
    }
    else if (((rtSHA_DW.SHA_temporalCounter_i1_fg >= 500) &&
              (rtSHA_DW.SHA_bitsForTID0.SHA_LogicalOperator_g)) &&
             (rtSHA_DW.SHA_Switch == 0.0))
    {
        // 如果满足条件，则执行系统运行状态的转换
        rtSHA_DW.SHA_bitsForTID0.SHA_is_IDLE = 0;
        rtSHA_DW.SHA_bitsForTID0.SHA_is_c13_SHA_SW = SHA_IN_RUNNING;
        rtSHA_DW.SHA_SysState_e = RUNNING;
        if (rtSHA_DW.SHA_bitsForTID0.SHA_is_RUNNING != SHA_IN_RUN_NORMAL)
        {
            rtSHA_DW.SHA_bitsForTID0.SHA_is_RUNNING = SHA_IN_RUN_NORMAL;
            rtSHA_DW.SHA_CtrState_i = CTR_RUN_NOR;
        }
    }
    else
    {
        // 根据当前状态执行状态转换和故障检测处理
        switch (rtSHA_DW.SHA_bitsForTID0.SHA_is_IDLE)
        {
            case SHA_IN_ShutHoldoff:
                // 状态为“关机保持关闭”，根据条件进行状态转换
                if ((rtSHA_DW.SHA_MinMax2 >= 2.0) || (rtSHA_DW.SHA_Switch != 0.0))
                {
                    rtSHA_DW.SHA_bitsForTID0.SHA_is_IDLE = SHA_IN_ShutHoldon;
                    rtSHA_DW.SHA_temporalCounter_i2 = 0U;
                }
                break;

            case SHA_IN_ShutHoldon:
                // 状态为“关机保持打开”，根据条件进行状态转换
                if ((rtSHA_DW.SHA_MinMax2 < 2.0) && (rtSHA_DW.SHA_Switch == 0.0))
                {
                    rtSHA_DW.SHA_bitsForTID0.SHA_is_IDLE = SHA_IN_ShutHoldoff;
                }
                else
                {
                    // 根据条件进行关机故障检测和状态转换
                    if (((rtSHA_DW.SHA_Gain <= 45.0) &&
                         (rtSHA_DW.SHA_temporalCounter_i2 >= 100)) ||
                        ((rtSHA_DW.SHA_Gain > 45.0) &&
                         (rtSHA_DW.SHA_temporalCounter_i2 >= 1500)))
                    {
                        rtSHA_DW.SHA_FaultCheckFlag = 11U;
                        rtSHA_DW.SHA_bitsForTID0.SHA_is_IDLE = 0;
                        rtSHA_DW.SHA_bitsForTID0.SHA_is_c13_SHA_SW = SHA_IN_SHUTDOWN;
                        rtSHA_DW.SHA_SysState_e = SHUTDOWN;
                        if (rtSHA_DW.SHA_bitsForTID0.SHA_is_SHUTDOWN !=
                            SHA_IN_SHUT_OCV)
                        {
                            rtSHA_DW.SHA_bitsForTID0.SHA_is_SHUTDOWN =
                                SHA_IN_SHUT_OCV;
                            rtSHA_DW.SHA_temporalCounter_i1_fg = 0U;
                            rtSHA_DW.SHA_CtrState_i = CTR_SHT_OCV;
                        }
                    }
                }
                break;
        }
    }
}
