/**
******************************************************************************
* Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
* @file    openlog.h
* @author  Lutoo e19135693@163.com
* @brief   Header file of openlog driver.
* @date    2021-11-3
* @version 1.1
* @par Change Log：
* <table>
* <tr><th>Date        <th>Version  <th>Author    		<th>Description
* <tr><td>2021-05-15  <td> 1.0     <td>Lutoo	 	    <td>Creator
* <tr><td>2021-11-3   <td> 1.1     <td>Lutoo            <td>make it easy to use
* </table>
*
*==============================================================================
					##### How to use this driver #####
==============================================================================
  @note
		please see https://git.scutbot.cn/WFshuli/openlog
******************************************************************************
* @attention
*
* if you had modified this file, please make sure your code does not have many
* bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
* through your new brief.
*
* <h2><center>&copy; Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
* All rights reserved.</center></h2>
******************************************************************************
*/

#pragma once

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string>
#include <string.h>
#include <stdarg.h>
/* Private define ------------------------------------------------------------*/
#define OP_MAX_BUFFSIZE 512
/* Exported types ------------------------------------------------------------*/
using op_user_transmit = bool (*)(uint8_t *buff, uint16_t len);

/**
 * @brief buff for openlog
 */
typedef struct{
    char        buff[OP_MAX_BUFFSIZE];
    uint16_t    write_size;
}op_buff;

/**
 * @brief openlog calss
 */
template<uint8_t buff_num>  
class openlog_classdef 
{
private:
	op_buff  	        mul_buff[buff_num];
	op_buff*	        buff_ptr;
    uint8_t 	        current_pos;
    uint8_t 	        pending_pos;
    uint8_t             pending_buff_num;
    op_user_transmit    output;
public:

     /**
     * @brief constructor
     */
    openlog_classdef(op_user_transmit _out)
    {
        static_assert(  (buff_num>0)&&(buff_num<=16),
                        "openlog buff number should be in [1,16]");
        output              = _out;
		buff_ptr            = mul_buff;
        current_pos         = 0;
        pending_pos         = 0;
        pending_buff_num    = 0;
    }

    /**
     * @brief destructor
     */
    ~openlog_classdef()
    {
		memset(mul_buff,0, buff_num * sizeof(op_buff));
        buff_ptr    = NULL;
        output      = NULL;
    }

	/**
     * @brief write string to the current buff
     */
	void write_string(const char *_str)
    {
        if(pending_buff_num == buff_num)
            return ;    // ERROR: no more space to write
        while(*_str != 0 && buff_ptr->write_size <= OP_MAX_BUFFSIZE)
        {
            buff_ptr->buff[buff_ptr->write_size ++] = *_str++;
        }
    }
		
	/**
     * @brief write multiple strings to the current buff
     */
    template<typename ... Args> void write_string(const char *_str, Args& ... args)
    {
        write_string(_str);
        write_string(args...);
    }
		
	/**
     * @brief push the current buff to the pending buff stream
     */
    uint8_t push_buff(void)
    {
        if(pending_buff_num == buff_num)
            return 1; // ERROR: no more buff to push
        if( ++current_pos == buff_num)
            current_pos = 0;
        pending_buff_num++;
        buff_ptr = (mul_buff + current_pos);
        buff_ptr->write_size = 0;   // clear buff
        return 0;
    }
		
	/**
     * @brief   fetch the first pending buff 
     * @return  pointer to the op_buff
     */
   op_buff* fetch_buff(void)
    {
        if(pending_buff_num == 0)
            return NULL;    // ERROR: no pending buff
        uint8_t pos = pending_pos;
        if( ++pending_pos == buff_num)
            pending_pos = 0;
        pending_buff_num--;
        return (mul_buff + pos);
    }
		
    /**
     * @brief send pending buff
     */
    uint8_t Send()
    {
        op_buff* ptr = NULL;
        ptr = fetch_buff();
        if( ptr == NULL) 
            return 1;
        output((uint8_t*)ptr->buff, ptr->write_size);
        return 0;
    }

    /**
     * @brief record string to the current buff
     */
    void record(const char* str, ...)
    {
        va_list args;
        va_start(args, str);
        record(str, args);
        va_end(args);
    }

    /**
     * @brief record string to the current buff
     */
    void record(const char* str, va_list args)
    {
        if(pending_buff_num == buff_num)
            return; // ERROR: no more buff to write
        buff_ptr->write_size += vsnprintf(buff_ptr->buff + buff_ptr->write_size,
                                            OP_MAX_BUFFSIZE - buff_ptr->write_size,
                                            str, args);
    }

    /* ----- File Manipulation ----- */
    /**
     * @brief creat new file
     */
    void new_file(const char* filename, ...)
    {
        va_list args;
        va_start(args, filename);
        write_string("new ");
        record(filename,args);
        va_end(args);
        write_string("\r");
        push_buff();
    }

    /**
     * @brief delect file
     */
    void rm_file(const char* filename, ...)
    {
        va_list args;
        va_start(args, filename);
        write_string("rm ");
        record(filename,args);
        va_end(args);
        write_string("\r");
        push_buff();
    }

    /**
     * @brief get the size of the file
     */
    void size_file(const char* filename, ...)
    {
        va_list args;
        va_start(args, filename);
        write_string("size ");
        record(filename,args);
        va_end(args);
        write_string("\r");
        push_buff();
    }

    /**
     * @brief contine write the file
     */
    void append_file(const char* filename, ...)
    {
        va_list args;
        va_start(args, filename);
        write_string("append ");
        record(filename,args);
        va_end(args);
        write_string("\r");
        push_buff();
    }

    /**
     * @brief overwrite the file
     */
    void write_file(uint32_t offset, const char* filename, ...)
    {
        va_list args;
        va_start(args, filename);
        write_string("write ");
        record(filename,args);
        va_end(args);
        record(" %d",offset);
        write_string("\r");
        push_buff();
    }

    /**
     * @brief read the file
     */
    void read_file()
    {
        // TODO
    }

    /* ----- Directory Manipulation ----- */
    /**
     * @brief enter command mode
     */
    void EnterCommandMode()
    {
        char com[3]={0x1A,0x1A,0x1A};
        write_string(com, "\r");
        push_buff();
    }

    /**
     * @brief ls
     */
    void ls()
    {
        write_string("ls\r");
        push_buff();
    }

    /**
     * @brief mkdir
     */
    void md(const char* subdirectory, ...)
    {
        va_list args;
        va_start(args, subdirectory);
        write_string("md ");
        record(subdirectory,args);
        va_end(args);
        write_string("\r");
        push_buff();
    }

    /**
     * @brief cd
     */
    void cd(const char* subdirectory, ...)
    {
        va_list args;
        va_start(args, subdirectory);
        write_string("cd ");
        record(subdirectory,args);
        va_end(args);
        write_string("\r");
        push_buff();
    }

    /**
     * @brief rm
     */
    void rm(const char* subdirectory, ...)
    {
        va_list args;
        va_start(args, subdirectory);
        write_string("rm ");
        record(subdirectory,args);
        va_end(args);
        write_string("\r");
        push_buff();
    }

    /**
     * @brief rm-rf
     */
    void rm_rf();void rm_rf(const char* subdirectory, ...)
    {
        va_list args;
        va_start(args, subdirectory);
        write_string("rm -rf ");
        record(subdirectory,args);
        va_end(args);
        write_string("\r");
        push_buff();
    }
    /* ----- Low Level Function Commands ----- */
    /**
     * @brief -h
     */
    void help()
    {
        write_string("?\r");
        push_buff();
    }

    /**
     * @brief get the disk message
     */
    void disk()
    {
        write_string("disk\r");
        push_buff();
    }

    /**
     * @brief Reinitialize the system and reopen the SD card.
     */
    void init()
    {
        write_string("init\r");
        push_buff();
    }

    /**
     * @brief Synchronizes the current contents of the buffer to the SD card.
     */
    void sync()
    {
        write_string("sync\r");
        push_buff();
    }

    /**
     * @brief Jumps OpenLog to location zero, reruns bootloader and then init code.
     */
    void reset()
    {
        write_string("reset\r");
        push_buff();
    }
};


#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
