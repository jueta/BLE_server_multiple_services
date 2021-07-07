/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC_A,

    IDX_CHAR_SVC_A_CHAR1,
    IDX_CHAR_VAL_SVC_A_CHAR1,
	IDX_CHAR_DESC_SVC_A_CHAR1,
    IDX_CHAR_CFG_SVC_A_CHAR1,

    IDX_CHAR_SVC_A_CHAR2,
    IDX_CHAR_VAL_SVC_A_CHAR2,
	IDX_CHAR_DESC_SVC_A_CHAR2,
    IDX_CHAR_CFG_SVC_A_CHAR2,

    SVC_A_IDX_NB,
};

enum
{
    IDX_SVC_B,

    IDX_CHAR_SVC_B_CHAR1,
    IDX_CHAR_VAL_SVC_B_CHAR1,
	IDX_CHAR_DESC_SVC_B_CHAR1,
    IDX_CHAR_CFG_SVC_B_CHAR1,

    IDX_CHAR_SVC_B_CHAR2,
    IDX_CHAR_VAL_SVC_B_CHAR2,
	IDX_CHAR_DESC_SVC_B_CHAR2,
    IDX_CHAR_CFG_SVC_B_CHAR2,

    SVC_B_IDX_NB,
};

void getBuffer(char * ret, const void *buffer, uint16_t buff_len);
void rx_task_uart0(void *arg);
void initUART0(void);
