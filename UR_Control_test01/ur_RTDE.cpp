#include "stdafx.h" // 有预编译头要求的时候加这个，没有的话注释掉
#include "ur_RTDE.h"

ur_RTDE::ur_RTDE()
{
	// 默认的文件名和IP地址
	xml_config_file_name = "control_loop_configuration.xml";
	IP_str = "192.168.1.5";
}

ur_RTDE::~ur_RTDE()
{
	;
}

void ur_RTDE::RTDE_Initialize(void)
{
    // 后面不会用到的变量将以该函数的局部变量的形式被定义

    PyObject* PM_rtde = PyImport_Import(PyString_FromString("rtde.rtde"));
    PyObject* PM_rtde_config = PyImport_Import(PyString_FromString("rtde.rtde_config"));

    PyObject* pDict_rtde = PyModule_GetDict(PM_rtde);
    PyObject* pDict_config = PyModule_GetDict(PM_rtde_config);

    PyObject* pClass_conf = PyDict_GetItemString(pDict_config, "ConfigFile");
    PyObject* pArgs_init_conf = Py_BuildValue(
		"(s)", 
		xml_config_file_name.c_str()); // 这个 文件名 表示我们自己配置的文件
    
    PyObject* pInstance_conf = PyObject_CallObject(pClass_conf, pArgs_init_conf);
    PyObject* p_Func_get_recipe = PyObject_GetAttrString(pInstance_conf, "get_recipe");

    // 这部分直接和配置文件相关，需要根据不同情况进行修改！
    // 当前类共有一个接收两个发送
    // 接收：
    // state - 其中存的是 output_bit_registers0_to_31，在UR端，这个变量实际上是按照32个bool变量进行操作的。
    //         script的语法是：write_output_boolean_register(number, bool) number 的取值为0-31，bool是
    //         true 或者 false
    // 发送：
    // 1. setp - 可以包含一个或多个double类型变量，但是最多 24 个，作为指定 UR 目标位姿的一种方式，可以在xml
    //           中添加的变量名为：input_double_register_X，其中X为 0-23 任意一个整数。UR端读取的方法是：
    //           read_input_float_register(number)
    // 2. ctrlflag - input_bit_registers0_to_31，在UR端同样是32个bool变量，只不过UR端只读。可以作为PC端
    //               触发UR端动作的32个标志位。UR端读取相应位的script代码语法是：
    //               read_input_boolean_register(number)，输入参数 number 取值0-31，返回值为 true or 
    //               false
	// 注：所有的script语法都可以在坚果云上的scriptManual.pdf中找到，但是很奇怪直接search搜不到


	// 接收状态通常是标志位，实时控制确实不好弄
    PyObject* p_res_state = PyEval_CallObject(p_Func_get_recipe, Py_BuildValue("(s)", "state"));

	// setp 里面包含多少个double，直接和xml文件的设置相关，但是最多只允许24个
    PyObject* p_res_setp = PyEval_CallObject(p_Func_get_recipe, Py_BuildValue("(s)", "setp"));

	// 发送同样需要发送标志位
	PyObject* p_res_ctrlflag = PyEval_CallObject(p_Func_get_recipe, Py_BuildValue("(s)", "ctrlflag"));
    
	//
    PyObject* pClass_RTDE = PyDict_GetItemString(pDict_rtde, "RTDE");
    PyObject* pArgs_init_RTDE = PyTuple_New(2);

    // 第一个参数看实际IP情况，第二个参数是固定端口30004
    PyTuple_SetItem(pArgs_init_RTDE, 0, PyString_FromString(IP_str.c_str()));		//这个要根据实际的计算机IP进行设置，目前所里的 UR3 是 .6，UR5 是 .5
	PyTuple_SetItem(pArgs_init_RTDE, 1, PyLong_FromLong(30004));
	pInstance_con = PyObject_CallObject(pClass_RTDE, pArgs_init_RTDE);

    // =======  这部分可以通过宏定义更加简洁 ============
    // 建立与机器人的连接
    PyEval_CallObject(
        PyObject_GetAttrString(pInstance_con, "connect"),	
        NULL);

    // 照搬Python，感觉没有意义
    PyEval_CallObject(
        PyObject_GetAttrString(pInstance_con, "get_controller_version"), 
        NULL);

    // 54设置输入输出 - 这里实际上也和具体应用有关系，有几个输入输出就调用几次
	PyEval_CallObject(
		PyObject_GetAttrString(pInstance_con, "send_output_setup"),
		p_res_state);
	
    setp = PyEval_CallObject(
		PyObject_GetAttrString(pInstance_con, "send_input_setup"),
		p_res_setp);
	
    ctrlflag = PyEval_CallObject(
		PyObject_GetAttrString(pInstance_con, "send_input_setup"),
		p_res_ctrlflag);
	
    /*watchdog = PyEval_CallObject(
		PyObject_GetAttrString(pInstance_con, "send_input_setup"),
		p_res_watchdog);*/
    
    // 至此，初始化完成

}

int ur_RTDE::RTDE_Send_Start(void)
{
    PyObject *res;
    res = PyEval_CallObject(
					PyObject_GetAttrString(pInstance_con, "send_start"),
					NULL);
    return PyInt_AsLong(res);
}

void ur_RTDE::RTDE_Send_Stop(void)
{
    PyEval_CallObject(
                PyObject_GetAttrString(pInstance_con, "send_pause"),
                NULL);

    PyEval_CallObject(
                PyObject_GetAttrString(pInstance_con, "disconnect"),
                NULL);
}

void ur_RTDE::RTDE_Send_BIT32(UINT32 BIT32_Value)
{
    int res;
    res = PyObject_SetAttrString(
        ctrlflag,
        "input_bit_registers0_to_31",
        PyInt_FromLong(BIT32_Value));

    PyObject *tuple_ctrlflag = PyTuple_New(1);
    Py_INCREF(ctrlflag);
    PyTuple_SetItem(tuple_ctrlflag, 0, ctrlflag);
    PyEval_CallObject(
				PyObject_GetAttrString(pInstance_con, "send"),
				tuple_ctrlflag);
}

void ur_RTDE::RTDE_Send_POINT(double *pt_seq, int pt_num)
{
    int res;
    memset(input_str, 0, INPUT_STR_LENGTH);
    for(int i=0; i<pt_num; i++)
    {
        sprintf_s(input_str, "input_double_register_%d", i);
        res = PyObject_SetAttrString(
            setp,
            input_str,
            PyFloat_FromDouble(pt_seq[i])
        );
    }
	PyObject *tuple_setp = PyTuple_New(1);
	Py_INCREF(setp);
	PyTuple_SetItem(tuple_setp, 0, setp);

	PyEval_CallObject(
		PyObject_GetAttrString(pInstance_con, "send"),
		tuple_setp);
}

void ur_RTDE::RTDE_Recv_BIT32(void)
{
    state = PyEval_CallObject(
					PyObject_GetAttrString(pInstance_con, "receive"),
					NULL);
	//Py_INCREF(state);
    state_output = 
        PyObject_GetAttrString(state,
        "output_bit_registers0_to_31"
        );
        
    state_res = PyInt_AsLong(state_output);
}