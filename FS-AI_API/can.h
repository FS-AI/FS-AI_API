/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   can.h
 * Author: ian
 *
 * Created on 25 April 2018, 22:13
 */

#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/can.h>	

int can_init(const char *port);
int can_send(struct can_frame *frame);
int can_read(struct can_frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */

